#!/usr/bin/env python3
"""Automatic, crash-tolerant CSV run logger for every sideboard Python tool."""
import atexit
import csv
import os
import signal
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
TOOLS_DIR = MODULE_DIR.parent
REPO_DIR = TOOLS_DIR.parent
RUN_LOG_DIR = TOOLS_DIR / "logs" / "run"


class _Tee:
    def __init__(self, original, logger, stream_name):
        self.original = original
        self.logger = logger
        self.stream_name = stream_name
        self.buf = ""

    def write(self, text):
        if not isinstance(text, str):
            text = str(text)
        try:
            self.original.write(text)
            self.original.flush()
        except Exception:
            pass
        self.buf += text
        while "\n" in self.buf:
            line, self.buf = self.buf.split("\n", 1)
            self.logger.row(self.stream_name, line.rstrip("\r"))
        return len(text)

    def flush(self):
        try:
            self.original.flush()
        except Exception:
            pass
        if self.buf:
            self.logger.row(self.stream_name, self.buf.rstrip("\r"))
            self.buf = ""

    def isatty(self):
        try:
            return self.original.isatty()
        except Exception:
            return False

    def fileno(self):
        return self.original.fileno()

    @property
    def encoding(self):
        return getattr(self.original, "encoding", "utf-8")


class RunCsv:
    def __init__(self, script):
        RUN_LOG_DIR.mkdir(parents=True, exist_ok=True)
        stem = Path(script).stem
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = RUN_LOG_DIR / f"{stem}_{stamp}_{os.getpid()}.csv"
        self.started = time.monotonic()
        self.file = open(self.path, "w", newline="", encoding="utf-8", buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(["iso_time", "elapsed_s", "stream", "message"])
        self.file.flush()
        self.closed = False

    def row(self, stream, message):
        if self.closed:
            return
        msg = str(message).replace("\x00", "")
        try:
            self.writer.writerow([
                datetime.now().astimezone().isoformat(timespec="milliseconds"),
                f"{time.monotonic()-self.started:.6f}",
                stream,
                msg,
            ])
            self.file.flush()
        except Exception:
            pass

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass


def _display(path):
    try:
        return Path(path).resolve().relative_to(REPO_DIR).as_posix()
    except Exception:
        return str(path)


def _exit_code(value):
    if value is None:
        return 0
    if isinstance(value, bool):
        return int(value)
    if isinstance(value, int):
        return value
    return 1


def run_logged(main_func, script):
    """Run main(), mirror stdout/stderr into CSV, and always close on Ctrl+C/error."""
    log = RunCsv(script)
    old_out, old_err = sys.stdout, sys.stderr
    tee_out = _Tee(old_out, log, "stdout")
    tee_err = _Tee(old_err, log, "stderr")
    sys.stdout, sys.stderr = tee_out, tee_err
    atexit.register(log.close)
    old_handlers = {}
    def _graceful_signal(signum, frame):
        log.row("event", f"SIGNAL {signum}")
        raise KeyboardInterrupt
    for sig_name in ("SIGTERM", "SIGBREAK"):
        sig = getattr(signal, sig_name, None)
        if sig is not None:
            try:
                old_handlers[sig] = signal.getsignal(sig)
                signal.signal(sig, _graceful_signal)
            except (ValueError, OSError):
                pass
    rc = 0
    try:
        log.row("event", "START argv=" + repr(sys.argv[1:]))
        print(f"AUTO CSV run-log: {_display(log.path)}")
        try:
            rc = _exit_code(main_func())
        except KeyboardInterrupt:
            rc = 130
            log.row("event", "CTRL_C")
            print("\nCtrl+C diterima; CSV sudah di-flush dan disimpan.", file=sys.stderr)
        except SystemExit as exc:
            rc = _exit_code(exc.code)
            log.row("event", f"SYSTEM_EXIT code={rc}")
        except BaseException as exc:
            rc = 1
            log.row("exception", f"{type(exc).__name__}: {exc}")
            for line in traceback.format_exc().splitlines():
                log.row("traceback", line)
            raise
        finally:
            log.row("event", f"END exit_code={rc}")
            tee_out.flush()
            tee_err.flush()
    finally:
        sys.stdout, sys.stderr = old_out, old_err
        for sig, handler in old_handlers.items():
            try: signal.signal(sig, handler)
            except (ValueError, OSError): pass
        try: atexit.unregister(log.close)
        except Exception: pass
        log.close()
    return rc


def main():
    print("run_csv logger self-test OK")
    print(f"run-log directory: {_display(RUN_LOG_DIR)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run_logged(main, __file__))
