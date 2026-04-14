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
- `deploy` → выполни **[Режим: deploy]**
- Начинается с `logs` → выполни **[Режим: logs]** (передай остаток аргумента)

Если аргумент не распознан — скажи пользователю:
```
Доступные команды:
  /micropython          — прошить firmware
  /micropython chip     — информация о чипе
  /micropython version  — версия MicroPython
  /micropython memory   — доступная RAM и диск
  /micropython deploy   — залить последние изменения кода
  /micropython logs     — захватить лог загрузки (20 сек)
  /micropython logs 100 — первые 100 строк после сброса
  /micropython logs last — весь лог загрузки (синоним logs)
  /micropython logs stream — стримить serial в реальном времени
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

---

## [Режим: deploy]

Залить последние изменения кода на устройство (TX или RX блок).

1. Выполни **[Общий шаг: поиск порта]**.

2. Спроси через `AskUserQuestion`, какой блок подключён:
   - **TX — машина** (есть радар LD2410B, отправляет тревогу)
   - **RX — брелок** (принимает сигнал, без радара)

3. В зависимости от выбора скопируй соответствующий набор файлов:

   **TX (машина):**
   ```
   python -m mpremote connect <PORT> cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py ld2410b.py boot.py webrepl_cfg.py secret.key :
   ```

   **RX (брелок):**
   ```
   python -m mpremote connect <PORT> cp main.py lr1121.py crypto.py oled.py battery.py logging.py ssd1306.py boot.py webrepl_cfg.py secret.key :
   ```

4. Покажи пользователю, какие файлы обновились (строки без `Up to date:`), а какие уже были актуальны.

5. Перезагрузи устройство:
   ```
   python -m mpremote connect <PORT> reset
   ```

6. Проверь, что устройство запустилось:
   ```
   python -m mpremote connect <PORT> exec "import sys; print('OK:', sys.version)"
   ```
   Покажи результат пользователю.

Если устройство не отвечает после reset — скажи пользователю нажать RESET вручную.

---

## [Режим: logs]

Читать логи с устройства через serial-порт.

> Логи пишутся только в StreamHandler (serial). Файла на диске нет.
> «last N строк» = «первые N строк после перезагрузки».

### Разбор аргумента

Возьми часть `$ARGUMENTS` после слова `logs` и обрежь пробелы:

- Пусто, `all`, `last` → `mode=all`, `limit=None`
- Число (например `100`) → `mode=all`, `limit=<N>`
- `stream` или `-s` → `mode=stream`

---

### mode=all (захват с начала загрузки)

1. Выполни **[Общий шаг: поиск порта]**.

2. Захват + мягкий сброс одной командой. Подставь `<PORT>` и `<LIMIT>` (`None` если без ограничения).
   Порт открывается ДО сброса, поэтому начало лога не теряется:
   ```
   python -c "
   import serial, sys, time, io
   sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
   port = '<PORT>'
   limit = <LIMIT>
   timeout_s = 25
   s = serial.Serial(port, 115200, timeout=0.3)
   s.write(b'\r\x03\x03')
   time.sleep(0.2)
   s.reset_input_buffer()
   s.write(b'\x04')
   count = 0
   start = time.time()
   while time.time() - start < timeout_s:
       line = s.readline()
       if line:
           text = line.decode('utf-8', errors='replace').rstrip()
           if text:
               print(text)
               sys.stdout.flush()
               count += 1
               if limit is not None and count >= limit:
                   break
   s.close()
   print(f'--- {count} lines captured ---')
   "
   ```
   Пояснение: `\x03\x03` = Ctrl+C дважды (прерывает main.py), `\x04` = Ctrl+D (soft reset MicroPython).

3. Покажи вывод пользователю. ANSI escape-коды (цвета) присутствуют — это нормально.

---

### mode=stream (реальное время)

1. Выполни **[Общий шаг: поиск порта]**.

2. Скажи пользователю:
   ```
   Устройство НЕ перезагружается — подключаюсь к текущему выводу serial.
   Запустите команду ниже в терминале, Ctrl+C для остановки:

     python -c "
   import serial, sys
   s = serial.Serial('<PORT>', 115200, timeout=0.1)
   print('=== Streaming (Ctrl+C to stop) ===')
   try:
       while True:
           data = s.read(256)
           if data:
               sys.stdout.write(data.decode('utf-8', errors='replace'))
               sys.stdout.flush()
   except KeyboardInterrupt:
       pass
   finally:
       s.close()
       print()
       print('=== Stream stopped ===')
   "

   Или введи в prompt: ! python -c "..."
   ```

3. Подставь реальный `<PORT>` в команду выше перед показом пользователю.

   Не запускай эту команду через Bash-инструмент — она блокирующая.
