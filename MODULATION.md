# Modulation Types in LR1121

## 1. **LoRa (Long Range)**
   - **Best For:** Long-range, low-power applications (IoT, remote sensors, asset tracking).
   - **Pros:**
     - Very long range (up to 15 km in rural areas).
     - Works well in noisy environments.
     - Low power consumption.
   - **Cons:**
     - Low data rates (<50 kbps).
     - Higher latency.

## 2. **FSK (Frequency Shift Keying)**
   - **Best For:** Medium-range, higher data rate applications (industrial telemetry, metering).
   - **Pros:**
     - Faster than LoRa (100-300 kbps).
     - Lower latency.
   - **Cons:**
     - Shorter range than LoRa.
     - More susceptible to interference.

## 3. **GFSK (Gaussian Frequency Shift Keying)**
   - **Best For:** Low-power, high-speed applications (BLE-like communication, industrial radio links).
   - **Pros:**
     - More efficient than standard FSK.
     - Can achieve very high data rates (>500 kbps).
   - **Cons:**
     - Shorter range than LoRa.
     - Requires more power for long-distance communication.

## **Comparison Table**
| Modulation | Best For | Range | Data Rate | Power Usage |
|------------|---------|-------|-----------|-------------|
| **LoRa** | Remote IoT, Smart Agriculture, GPS tracking | Long | Low (<50 kbps) | Low |
| **FSK** | Industrial sensors, Wireless metering | Medium | Medium (100-300 kbps) | Medium |
| **GFSK** | BLE-like communication, High-speed links | Short | High (500+ kbps) | Medium-High |

## **Which One to Use?**
- **LoRa** → If **range and low power** are most important.
- **FSK** → If **higher data rates** are needed with **moderate range**.
- **GFSK** → If **even faster speeds** are required and **range is not critical**.
