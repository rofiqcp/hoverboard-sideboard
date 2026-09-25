#!/usr/bin/env python3
"""Portable paths for sideboard tools.

All generated CSV files live under tools/logs. User-supplied output names must
be relative so the same command works on Windows and Linux.
"""
from datetime import datetime
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent
TOOLS_DIR = MODULE_DIR.parent
REPO_DIR = TOOLS_DIR.parent
LOG_DIR = TOOLS_DIR / "logs"


def tool_file(name: str) -> Path:
    """Path modul executable di tools/imu/."""
    return MODULE_DIR / name


def log_output(value=None, prefix="imu", suffix=".csv") -> Path:
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    if not value or str(value).lower() == "auto":
        name = f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S')}{suffix}"
        return LOG_DIR / name

    p = Path(str(value))
    if p.is_absolute():
        raise ValueError("output harus path relatif; file CSV disimpan di tools/logs/")

    parts = list(p.parts)
    if parts and parts[0].lower() == "tools":
        parts = parts[1:]
    if parts and parts[0].lower() == "logs":
        parts = parts[1:]
    if not parts:
        raise ValueError("nama output CSV kosong")

    rel = Path(*parts)
    if rel.suffix == "":
        rel = rel.with_suffix(suffix)
    elif rel.suffix.lower() != suffix.lower():
        raise ValueError(f"output harus berakhiran {suffix}")

    target = (LOG_DIR / rel).resolve()
    root = LOG_DIR.resolve()
    if not target.is_relative_to(root):
        raise ValueError("output harus tetap berada di tools/logs/")
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def display_path(path) -> str:
    p = Path(path).resolve()
    try:
        return p.relative_to(REPO_DIR).as_posix()
    except ValueError:
        return str(p)


def _relative_log_name(value) -> Path:
    p = Path(str(value))
    if p.is_absolute():
        raise ValueError("CSV harus memakai path relatif di tools/logs/")
    parts = list(p.parts)
    if parts and parts[0].lower() == "tools":
        parts = parts[1:]
    if parts and parts[0].lower() == "logs":
        parts = parts[1:]
    if not parts:
        raise ValueError("nama/path CSV kosong")
    rel = Path(*parts)
    if ".." in rel.parts:
        raise ValueError("CSV tidak boleh keluar dari tools/logs/")
    return rel


def log_input(value) -> Path:
    rel = _relative_log_name(value)
    target = (LOG_DIR / rel).resolve()
    if not target.is_relative_to(LOG_DIR.resolve()):
        raise ValueError("CSV harus berada di tools/logs/")
    if not target.exists():
        raise FileNotFoundError(f"CSV tidak ditemukan: {display_path(target)}")
    return target


def expand_log_inputs(values):
    """Expand relative CSV names/globs under tools/logs on Windows and Linux."""
    out = []
    for value in values:
        rel = _relative_log_name(value)
        pattern = rel.as_posix()
        wildcard = any(ch in pattern for ch in "*?[")
        matches = sorted(LOG_DIR.glob(pattern)) if wildcard else [LOG_DIR / rel]
        if not matches or any(not p.exists() for p in matches):
            raise FileNotFoundError(f"CSV tidak ditemukan: tools/logs/{pattern}")
        out.extend(p.resolve() for p in matches)
    return out


def main():
    LOG_DIR.mkdir(parents=True,exist_ok=True)
    print("repo:",REPO_DIR)
    print("tools:",TOOLS_DIR)
    print("logs:",LOG_DIR)
    return 0

if __name__=="__main__":
    from run_csv import run_logged
    raise SystemExit(run_logged(main,__file__))
