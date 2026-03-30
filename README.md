# ELRS Head Tracker to SBUS Bridge

An ESP32 Arduino sketch that acts as an ELRS Backpack receiver, capturing head tracking data (pan/tilt/roll) sent via ESP-NOW from a VRx module and outputting it as a standard SBUS stream. This allows one to use Googles that support head tracking (e.g. HDZero) with radio links other than ELRS.

## How It Works

1. The ESP32 emulates an ELRS Backpack device
2. It receives `MSP_ELRS_BACKPACK_SET_PTR` messages from the VRx module over ESP-NOW
3. Pan, tilt, and roll values are mapped to three consecutive SBUS channels
4. An SBUS frame is transmitted at 50 Hz on a configurable GPIO pin

## Requirements

- **Hardware:** Any ESP32-family board (ESP32, S2, S3, C3, C6)
- **Software:** Arduino IDE with the ESP32 board package installed

## Setup

### 1. Install the ESP32 Board Package

In Arduino IDE, go to **File > Preferences** and add the ESP32 board manager URL, then install the **esp32** package via **Tools > Board > Boards Manager**.

### 2. Configure the Sketch

Open `ELRS-Headtracker-to-SBUS.ino` and edit the options at the top:

```cpp
#define BINDING_PHRASE   "BINDPHRASE"  // Must match your ELRS / Backpack binding phrase
#define PTR_CH_START     1             // First SBUS channel for Pan (Tilt = CH2, Roll = CH3)
#define DEBUG_MODE                     // Comment out to disable Serial debug output
#define SBUS_TX_PIN      14           // GPIO pin for SBUS TX output
```

- **`BINDING_PHRASE`** must exactly match the binding phrase configured in your ELRS/Backpack setup.
- **`PTR_CH_START`** sets which SBUS channel receives Pan data (valid range: 1-14). Tilt and Roll follow on the next two channels.
- **`SBUS_TX_PIN`** can be any available GPIO pin on your ESP32.

### 3. Upload

Select your ESP32 board and port in the IDE, then upload.

### 4. Wiring

Connect the `SBUS_TX_PIN` GPIO to the Rx pin of the Aux port on your radio.

### 5. EdgeTX Radio Setup

1. On your radio, go to **SYS > Hardware** and set **Aux 1/2** to **SBUS Trainer**
2. In **Model > Trainer**, set the mode to **Master / Serial**
3. In your **Mixes**, use the **TRx** inputs (TR1, TR2, TR3) to map the head tracker channels to your desired outputs

## License

GPL-3.0 -- see [LICENSE](LICENSE).
