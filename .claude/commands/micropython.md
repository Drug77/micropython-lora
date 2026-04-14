# MicroPython — управление устройством ESP32-S3

Работа с устройством LilyGO T3S3 v1.1 (ESP32-S3) через esptool и mpremote.

Аргумент вызова: `$ARGUMENTS`

---

## Определение режима

Посмотри на `$ARGUMENTS`:

- Пусто или `flash` → выполни **[Режим: flash]**
- `chip` → выполни **[Режим: chip]**
- `version` → выполни **[Режим: version]**
- `memory` → выполни **[Режим: memory]**

Если аргумент не распознан — скажи пользователю:
```
Доступные команды:
  /micropython          — прошить firmware
  /micropython chip     — информация о чипе
  /micropython version  — версия MicroPython
  /micropython memory   — доступная RAM и диск
```

---

## [Общий шаг: поиск порта]

Запусти `python -m serial.tools.list_ports -v`. Найди порт с VID `303A` (Espressif, USB Serial Device). Запомни его как `<PORT>`. Если не найден — скажи пользователю подключить устройство и повторить.

---

## [Режим: flash]

Прошить MicroPython firmware на устройство.

1. Выполни **[Общий шаг: поиск порта]**.

2. Запусти `python -m esptool --port <PORT> --baud 460800 chip_id` и покажи пользователю результат.

3. Используй `Glob` для поиска файлов `*.bin` в корне проекта. Покажи пользователю нумерованный список всех найденных файлов и **обязательно спроси через `AskUserQuestion`**, какой файл прошивать — даже если файл один. Формат вопроса:
   ```
   Найдены файлы прошивки:
   1. ESP32_GENERIC_S3-...bin
   2. ...
   Какой файл прошить? (введи номер)
   ```

4. Запусти:
   ```
   python -m esptool --port <PORT> --baud 460800 write_flash 0 <ВЫБРАННЫЙ_ФАЙЛ.bin>
   ```
   Если ошибка "No serial data received" — объясни пользователю: удерживай BOOT (GPIO 0), нажми+отпусти RESET, отпусти BOOT — затем повтори.

5. После прошивки проверь версию:
   ```
   python -m mpremote connect <PORT> exec "import sys; print(sys.version)"
   ```
   Покажи результат пользователю.

---

## [Режим: chip]

Показать подробную информацию о чипе без прошивки.

1. Выполни **[Общий шаг: поиск порта]**.

2. Запусти `python -m esptool --port <PORT> --baud 460800 chip_id` и покажи результат (MAC, тип чипа).

3. Запусти `python -m esptool --port <PORT> --baud 460800 flash_id` и покажи результат (размер и тип Flash-памяти).

Если подключение не удаётся — напомни пользователю войти в режим загрузчика: удерживай BOOT, нажми+отпусти RESET, отпусти BOOT.

---

## [Режим: version]

Считать, какая версия MicroPython установлена на устройстве (без входа в режим загрузчика).

1. Выполни **[Общий шаг: поиск порта]**.

2. Запусти:
   ```
   python -m mpremote connect <PORT> exec "import sys; print(sys.version); print(sys.implementation)"
   ```
   Покажи результат пользователю: версию Python и реализацию (MicroPython + версия сборки).

Если устройство не отвечает — скажи пользователю: устройство может быть в режиме загрузчика, нажмите RESET для нормальной загрузки.

---

## [Режим: memory]

Проверить доступную RAM и место на файловой системе (диск).

1. Выполни **[Общий шаг: поиск порта]**.

2. Для RAM запусти:
   ```
   python -m mpremote connect <PORT> exec "import gc; gc.collect(); print('RAM free:', gc.mem_free(), 'bytes'); print('RAM alloc:', gc.mem_alloc(), 'bytes')"
   ```

3. Для диска запусти:
   ```
   python -m mpremote connect <PORT> exec "import os; st = os.statvfs('/'); print('Disk total:', st[0]*st[2], 'bytes'); print('Disk free:', st[0]*st[3], 'bytes')"
   ```

4. Покажи результаты в удобочитаемом виде (переведи байты в KB/MB где уместно).

Если устройство не отвечает — скажи пользователю нажать RESET для нормальной загрузки (не режим загрузчика).
