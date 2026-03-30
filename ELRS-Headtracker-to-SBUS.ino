/*
 * ELRS Head Tracker -> SBUS Bridge
 *
 * Emulates an ELRS Backpack to receive MSP_ELRS_BACKPACK_SET_PTR
 * messages from a VRx module via ESP-NOW, converts pan/tilt/roll into
 * RC channel values, and outputs them as an SBUS stream on Serial1.
 *
 * Requires ESP32-family (ESP32, S2, S3, C3, C6) — uses ESP-IDF APIs
 * (ESP-NOW, mbedtls, HardwareSerial pin remapping) not available on ESP8266.
 *
 * Protocol details derived from:
 *   github.com/ExpressLRS/Backpack   (ESP-NOW + MSP framing)
 *   github.com/ExpressLRS/ExpressLRS (SBUS output + CRSF channel encoding)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <mbedtls/md5.h>
#include "msp.h"
#include "sbus.h"

// ======================== OPTIONS ====================================
#define BINDING_PHRASE   "BINDPHRASE"  // Must match your ELRS / Backpack setup
#define PTR_CH_START     1                   // First channel for Pan (Tilt=CH2, Roll=CH3)
#define DEBUG_MODE                         // Uncomment to enable Serial debug output

// ======================== PIN ASSIGNMENT =============================
#define SBUS_TX_PIN      14                   // Serial1 TX GPIO for SBUS output, select any pin

static_assert(PTR_CH_START >= 1 && PTR_CH_START <= 14, "PTR_CH_START must be 1-14 (needs 3 channels)");

// ======================== DEBUG MACROS ===============================
#ifdef DEBUG_MODE
  #define DBG_INIT()  Serial.begin(115200)
  #define DBG(...)    Serial.printf(__VA_ARGS__)
#else
  #define DBG_INIT()
  #define DBG(...)
#endif

// ======================== CONSTANTS =================================
constexpr uint32_t SBUS_INTERVAL_MS = 20;    // 50 Hz SBUS frame rate
constexpr uint32_t PTR_TIMEOUT_MS   = 1000;  // Failsafe after 1 s silence

// ======================== GLOBALS ===================================
uint8_t  uid[6];
uint16_t channels[SBUS_NUM_CHANNELS];
MspParser msp;

// Written from ESP-NOW callback, read from loop — guarded by ptrMux
portMUX_TYPE ptrMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint16_t ptrCh[3];
volatile uint32_t lastPtrMs;
volatile bool     sendHTEnable;   // Flag: VRx requested cached packets
volatile uint32_t ptrPacketCount; // Total PTR packets received
uint32_t lastSbusMs;
uint32_t lastEnableMs;
bool     wasActive;

// ======================== UID FROM BINDING PHRASE ====================
// Replicates ELRS build system: MD5('-DMY_BINDING_PHRASE="<phrase>"')[0:6]
void generateUID(const char *phrase, uint8_t *out) {
    char buf[128];
    snprintf(buf, sizeof(buf), "-DMY_BINDING_PHRASE=\"%s\"", phrase);

    uint8_t hash[16];
    mbedtls_md5_context ctx;
    mbedtls_md5_init(&ctx);
    mbedtls_md5_starts(&ctx);
    mbedtls_md5_update(&ctx, (const uint8_t *)buf, strlen(buf));
    mbedtls_md5_finish(&ctx, hash);
    mbedtls_md5_free(&ctx);

    memcpy(out, hash, 6);
    out[0] &= ~0x01;  // Ensure unicast (clear LSB of first byte)
}

// ======================== SEND MSP VIA ESP-NOW =======================
void sendMspEspNow(uint16_t function, const uint8_t *data, uint16_t len) {
    uint8_t frame[32];
    uint8_t frameLen = mspBuildCommand(frame, sizeof(frame), function, data, len);
    if (frameLen) esp_now_send(uid, frame, frameLen);
}

void sendHeadTrackingEnable() {
    uint8_t enable = 1;
    sendMspEspNow(MSP_ELRS_SET_HEAD_TRACKING, &enable, 1);
    DBG("Sent SET_HEAD_TRACKING enable\n");
}

// ======================== ESP-NOW RECEIVE CALLBACK ===================
void onEspNowRecv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int len) {
    const uint8_t *mac = info->src_addr;
    if (memcmp(mac, uid, 6) != 0) return;

    // Each ESP-NOW packet is one complete MSP frame
    msp.reset();
    for (int i = 0; i < len; i++) {
        if (msp.feed(data[i])) {
            if (msp.function == MSP_ELRS_SET_PTR && msp.size == 6) {
                portENTER_CRITICAL(&ptrMux);
                ptrCh[0] = min(msp.payloadU16(0), (uint16_t)2047);  // Pan
                ptrCh[1] = min(msp.payloadU16(2), (uint16_t)2047);  // Tilt
                ptrCh[2] = min(msp.payloadU16(4), (uint16_t)2047);  // Roll
                lastPtrMs = millis();
                portEXIT_CRITICAL(&ptrMux);
                ptrPacketCount++;
            }
            else if (msp.function == MSP_ELRS_REQU_VTX_PKT) {
                sendHTEnable = true;
                DBG("VRx requested cached state\n");
            }
        }
    }
}

//filtering outputs
uint32_t sanitizeCh(uint32_t localPtrMs) {
    
    int base = PTR_CH_START - 1;

    for (int i = 0; i < ESP_NUM_CH; i++) {
        if (localPtrMs[i] < CRSF_CHANNEL_MIN) { localPtrMs[i]=CRSF_CHANNEL_MIN;}
        if (localPtrMs[i] > CRSF_CHANNEL_MAX) { localPtrMs[i]=CRSF_CHANNEL_MAX;}

        if (channels[base + i] == CRSF_CHANNEL_MIN && localPtrMs[i] == CRSF_CHANNEL_MAX) {
            localPtrMs[i] = CRSF_CHANNEL_MIN;
        }
        if (channels[base + i] == CRSF_CHANNEL_MAX && localPtrMs[i] == CRSF_CHANNEL_MIN) {
            localPtrMs[i] = CRSF_CHANNEL_MAX;
        }

    }    
}

// ======================== SETUP ======================================
void setup() {
    DBG_INIT();
    DBG("\nELRS Head Tracker -> SBUS Bridge\n");

    generateUID(BINDING_PHRASE, uid);
    DBG("UID: %02X:%02X:%02X:%02X:%02X:%02X\n",
        uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);

    for (int i = 0; i < SBUS_NUM_CHANNELS; i++)
        channels[i] = CRSF_CHANNEL_MID;

    // SBUS output on Serial1 — 100 kbaud, 8E2, non-inverted (idle-high), TX-only
    Serial1.begin(100000, SERIAL_8E2, -1, SBUS_TX_PIN, false);

    // WiFi STA mode for ESP-NOW on channel 1 (hardcoded in all Backpack modules)
    WiFi.mode(WIFI_STA);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);  // ~7 mW — plenty for short-range backpack link
    esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    WiFi.begin("", "", 1);
    WiFi.disconnect();

    esp_wifi_set_mac(WIFI_IF_STA, uid);

    if (esp_now_init() != ESP_OK) {
        DBG("ESP-NOW init failed — restarting\n");
        ESP.restart();
    }

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, uid, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK)
        DBG("ESP-NOW add peer failed\n");
    esp_now_register_recv_cb(onEspNowRecv);

    DBG("SBUS TX on GPIO %d  |  Channels CH%d-CH%d (Pan/Tilt/Roll)\n",
        SBUS_TX_PIN, PTR_CH_START, PTR_CH_START + 2);
    DBG("Waiting for head tracker...\n");
}

// ======================== LOOP =======================================
void loop() {
    uint32_t now = millis();

    // Respond to VRx requesting cached packets
    if (sendHTEnable) {
        sendHTEnable = false;
        sendHeadTrackingEnable();
        lastEnableMs = now;
    }

    // Snapshot shared state under lock
    portENTER_CRITICAL(&ptrMux);
    uint16_t localPtr[3] = { ptrCh[0], ptrCh[1], ptrCh[2] };
    uint32_t localPtrMs  = lastPtrMs;
    portEXIT_CRITICAL(&ptrMux);

    // Periodically send enable until we receive PTR data
    bool active = localPtrMs > 0 && ((now - localPtrMs) < PTR_TIMEOUT_MS || localPtrMs > now);
    if (!active && (now - lastEnableMs >= 1000)) {
        sendHeadTrackingEnable();
        lastEnableMs = now;
    }

    // Log state transitions
    if (active != wasActive) {
        wasActive = active;
        if (active)
            DBG("Head tracker connected (packets: %lu)\n", ptrPacketCount);
        else if (localPtrMs > 0)
            DBG("Head tracker lost — failsafe (last packet %lums ago)\n", now - localPtrMs);
    }

    // SBUS output
    if (now - lastSbusMs >= SBUS_INTERVAL_MS) {
        lastSbusMs = now;

        if (active) {
            int base = PTR_CH_START - 1;
            sanitizeCh(localPtr);
            channels[base + 0] = localPtr[0];
            channels[base + 1] = localPtr[1];
            channels[base + 2] = localPtr[2];
        }

        uint8_t flags = active ? 0 : (SBUS_FLAG_SIGLOSS | SBUS_FLAG_FAILSAFE);
        sbusWrite(Serial1, channels, flags);
    }
}
