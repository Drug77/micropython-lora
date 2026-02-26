from micropython import const
import io
import sys
import time

CRITICAL = const(50)
ERROR = const(40)
WARNING = const(30)
INFO = const(20)
DEBUG = const(10)
NOTSET = const(0)

_DEFAULT_LEVEL = const(WARNING)

_level_dict = {
    CRITICAL: "CRITICAL",
    ERROR: "ERROR",
    WARNING: "WARNING",
    INFO: "INFO",
    DEBUG: "DEBUG",
    NOTSET: "NOTSET",
}

# ANSI цвета для консоли
_color_dict = {
    CRITICAL: "\033[1;31m", # Жирный красный
    ERROR: "\033[31m",      # Красный
    WARNING: "\033[33m",    # Желтый
    INFO: "\033[32m",       # Зеленый
    DEBUG: "\033[36m",      # Голубой
    NOTSET: "\033[0m",      # Сброс
}
_RESET_COLOR = "\033[0m"

_loggers = {}
_stream = sys.stderr
_default_fmt = "%(asctime)s %(name)s [%(levelname)s] - %(message)s"


class LogRecord:
    def set(self, name, level, message):
        self.name = name
        self.levelno = level
        self.levelname = _level_dict[level]
        self.message = message
        self.ct = time.time()
        self.msecs = int((self.ct - int(self.ct)) * 1000)
        self.asctime = None


class Handler:
    def __init__(self, level=NOTSET):
        self.level = level
        self.formatter = None

    def close(self):
        pass

    def setLevel(self, level):
        self.level = level

    def setFormatter(self, formatter):
        self.formatter = formatter

    def format(self, record):
        return self.formatter.format(record)


class StreamHandler(Handler):
    def __init__(self, stream=None, use_color=True):
        super().__init__()
        self.stream = _stream if stream is None else stream
        self.terminator = "\n"
        self.use_color = use_color

    def close(self):
        if hasattr(self.stream, "flush"):
            self.stream.flush()

    def emit(self, record):
        if record.levelno >= self.level:
            msg = self.format(record)
            # Добавляем цвет, если вывод идет в консоль
            if self.use_color and hasattr(self.stream, "write"):
                color = _color_dict.get(record.levelno, _RESET_COLOR)
                msg = f"{color}{msg}{_RESET_COLOR}"
            
            self.stream.write(msg + self.terminator)


class FileHandler(StreamHandler):
    def __init__(self, filename, mode="a", encoding="UTF-8"):
        # При записи в файл цвета отключаем (иначе будут кракозябры)
        super().__init__(stream=open(filename, mode=mode, encoding=encoding), use_color=False)

    def close(self):
        super().close()
        self.stream.close()


class Formatter:
    def __init__(self, fmt=None):
        self.fmt = _default_fmt if fmt is None else fmt

    def usesTime(self):
        return "%(asctime)s" in self.fmt

    def formatTime(self, record):
        t = time.localtime(record.ct)
        # MicroPython возвращает кортеж из 8 элементов
        return f"{t[0]:04}-{t[1]:02}-{t[2]:02} {t[3]:02}:{t[4]:02}:{t[5]:02}"

    def format(self, record):
        if self.usesTime():
            record.asctime = self.formatTime(record)
            
        # Формируем словарь только с нужными параметрами, чтобы избежать ошибок форматирования
        # Выглядит чуть сложнее, но зато %()s работает железобетонно
        return self.fmt % {
            "name": record.name,
            "message": record.message,
            "msecs": record.msecs,
            "asctime": record.asctime,
            "levelname": record.levelname,
        }


class Logger:
    def __init__(self, name, level=NOTSET):
        self.name = name
        self.level = level
        self.handlers = []
        self.record = LogRecord()

    def setLevel(self, level):
        self.level = level

    def isEnabledFor(self, level):
        return level >= self.getEffectiveLevel()

    def getEffectiveLevel(self):
        return self.level or getLogger().level or _DEFAULT_LEVEL

    def log(self, level, msg, *args):
        if self.isEnabledFor(level):
            if args:
                if isinstance(args[0], dict):
                    args = args[0]
                msg = msg % args
            self.record.set(self.name, level, msg)
            handlers = self.handlers
            if not handlers:
                handlers = getLogger().handlers
            for h in handlers:
                h.emit(self.record)

    def debug(self, msg, *args):
        self.log(DEBUG, msg, *args)

    def info(self, msg, *args):
        self.log(INFO, msg, *args)

    def warning(self, msg, *args):
        self.log(WARNING, msg, *args)

    def error(self, msg, *args):
        self.log(ERROR, msg, *args)

    def critical(self, msg, *args):
        self.log(CRITICAL, msg, *args)

    def exception(self, msg, *args, exc_info=True):
        self.log(ERROR, msg, *args)
        tb = None
        if isinstance(exc_info, BaseException):
            tb = exc_info
        elif hasattr(sys, "exc_info"):
            tb = sys.exc_info()[1]
        if tb:
            buf = io.StringIO()
            sys.print_exception(tb, buf)
            self.log(ERROR, buf.getvalue())

    def addHandler(self, handler):
        self.handlers.append(handler)

    def hasHandlers(self):
        return len(self.handlers) > 0


def getLogger(name=None):
    if name is None:
        name = "root"
    if name not in _loggers:
        _loggers[name] = Logger(name)
        if name == "root":
            basicConfig()
    return _loggers[name]


def log(level, msg, *args): getLogger().log(level, msg, *args)
def debug(msg, *args): getLogger().debug(msg, *args)
def info(msg, *args): getLogger().info(msg, *args)
def warning(msg, *args): getLogger().warning(msg, *args)
def error(msg, *args): getLogger().error(msg, *args)
def critical(msg, *args): getLogger().critical(msg, *args)
def exception(msg, *args): getLogger().exception(msg, *args)

def shutdown():
    """Исправленный shutdown: безопасно закрывает файлы и очищает словарь"""
    for logger in _loggers.values():
        for h in logger.handlers:
            h.close()
    _loggers.clear()

def addLevelName(level, name):
    _level_dict[level] = name

def basicConfig(
    filename=None,
    filemode="a",
    format=None,
    level=WARNING,
    stream=None,
    encoding="UTF-8",
    force=False,
):
    if "root" not in _loggers:
        _loggers["root"] = Logger("root")

    logger = _loggers["root"]

    if force or not logger.handlers:
        for h in logger.handlers:
            h.close()
        logger.handlers = []

        if filename is None:
            handler = StreamHandler(stream)
        else:
            handler = FileHandler(filename, filemode, encoding)

        handler.setLevel(level)
        handler.setFormatter(Formatter(format))

        logger.setLevel(level)
        logger.addHandler(handler)


if hasattr(sys, "atexit"):
    sys.atexit(shutdown)
    
if __name__ == "__main__":
    # Тестируем новую красоту
    basicConfig(level=DEBUG)
    logger = getLogger("MAIN")

    logger.debug("This is a debug message - Usually grey/blue")
    logger.info("Everything is OK - Usually green")
    logger.warning("Low memory warning - Usually yellow")
    logger.error("Failed to transmit data! - Red")
    logger.critical("SYSTEM HALTED - Bold Red")