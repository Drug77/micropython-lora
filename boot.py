import machine
import network
import time
from machine import Pin

# ==============================================================================
# OTA mode: hold BOOT button (GPIO 0) for 2 seconds at power-on.
# LED blinks 6x to confirm entry → stays solid while WebREPL is active.
# Reset the device manually when done uploading files.
# Normal boot (button not held): falls through, MicroPython loads main.py.
#
# Deep sleep wake: skip OTA check entirely — button press is for wake,
# not OTA entry. Without this, pressing button to wake could accidentally
# enter OTA mode if held too long.
# ==============================================================================

OTA_TRIGGER_PIN = 0
OTA_HOLD_MS     = 2000
WIFI_SSID       = "Fold5"
WIFI_PASS       = "159632478"


def _ota_mode():
    led = Pin(37, Pin.OUT, value=0)

    # 6 rapid blinks = OTA mode confirmed
    for _ in range(6):
        led.value(1); time.sleep_ms(100)
        led.value(0); time.sleep_ms(100)

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)

    for _ in range(150):        # wait up to 15 s
        if wlan.isconnected():
            break
        time.sleep_ms(100)

    if not wlan.isconnected():
        # SOS blink, then reboot into normal mode
        for _ in range(9):
            led.value(1); time.sleep_ms(50)
            led.value(0); time.sleep_ms(50)
        machine.reset()

    import webrepl
    webrepl.start()             # reads password from webrepl_cfg.py

    ip = wlan.ifconfig()[0]
    print(f"[OTA] WebREPL active at {ip}:8266")
    print("[OTA] Upload files, then reset the device to boot normally.")
    print("[OTA] CLI example:")
    print(f"[OTA]   python webrepl_cli.py -p ota12345 ws://{ip}:8266/ file.py :/file.py")

    led.value(1)                # solid LED = OTA active
    while True:
        time.sleep(1)           # keep WebREPL alive; main.py never runs


# ── Entry point ───────────────────────────────────────────────────────────────

if machine.reset_cause() != machine.DEEPSLEEP_RESET:
    # Cold boot / hard reset — check for OTA entry
    btn = Pin(OTA_TRIGGER_PIN, Pin.IN, Pin.PULL_UP)

    if btn.value() == 0:
        # Button is pressed at boot — wait OTA_HOLD_MS to confirm intent
        deadline = time.ticks_add(time.ticks_ms(), OTA_HOLD_MS)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if btn.value() != 0:
                break           # released early → normal boot
            time.sleep_ms(50)
        else:
            _ota_mode()         # held the full duration → enter OTA

# Deep sleep wake or normal boot: fall through → MicroPython loads main.py
