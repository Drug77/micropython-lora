python -m esptool --port COM15 erase_flash
python -m esptool --port COM15 --baud 460800 write_flash 0 ESP32_GENERIC_S3-FLASH_4M-20241129-v1.24.1.bin