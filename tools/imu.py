#!/usr/bin/env python3
"""Simple entry point for sideboard IMU tools.

User flow:
  RAW -> STILL -> 6-SIDE -> optional PRECISION/THERMAL -> READY

Without arguments: beginner-friendly interactive menu.
All implementation modules remain directly runnable in tools/imu/.
"""
import argparse
import importlib
import subprocess
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
IMU_DIR = TOOLS_DIR / "imu"
REPO_DIR = TOOLS_DIR.parent
if str(IMU_DIR) not in sys.path:
    sys.path.insert(0, str(IMU_DIR))

from run_csv import run_logged

COMMANDS = {
    # Main user workflow
    "raw": ("read_imu.py", ["--mode", "raw"], "Baca sensor mentah"),
    "ready": ("read_imu.py", ["--mode", "ready"], "Baca data final siap pakai"),
    "read": ("read_imu.py", ["--mode", "all"], "Diagnostik lengkap (legacy)"),
    "log": ("imu_log.py", [], "Rekam data lengkap ke CSV"),
    "still": ("calibrate_imu.py", ["still"], "Kalibrasi STILL"),
    "six-side": ("calibrate_imu.py", ["rotate-start"], "Kalibrasi accelerometer 6 sisi"),
    "status": ("calibrate_imu.py", ["status"], "Status kalibrasi"),

    # Advanced / maintenance
    "monitor": ("calibrate_imu.py", ["monitor"], "Monitor kalibrasi"),
    "config": ("configure_imu.py", [], "Konfigurasi IMU"),
    "vesc": ("vesc_imu_query.py", [], "Query kompatibel VESC"),
    "aiding": ("aiding_imu.py", [], "Kirim external aiding"),
    "ellipsoid": ("accel_ellipsoid_calibration.py", [], "Fit accelerometer 18-24 pose"),
    "thermal": ("thermal_calibration.py", [], "Fit thermal"),
    "allan": ("allan_analysis.py", [], "Analisis Allan/noise"),
    "serial": ("serial_common.py", [], "Diagnostik serial"),
    "paths": ("tool_paths.py", [], "Tampilkan path"),
    "flash": ("flash_uart.py", [], "Flash UART"),
    "gui": ("imu_gui.py", [], "Qt6 interactive dashboard"),
}


def _tool(name):
    p = IMU_DIR / name
    if not p.exists():
        raise FileNotFoundError(f"Tool tidak ditemukan: {p}")
    return p


_IN_PROCESS = {"raw","ready","read","log","still","six-side","status"}

def _run_in_process(script, argv):
    """Jalankan tool utama pada interpreter yang sama agar ownership COM tunggal."""
    module_name=Path(script).stem
    module=importlib.import_module(module_name)
    old_argv=sys.argv[:]
    sys.argv=[script,*argv]
    try:
        rc=module.main()
        return int(rc or 0)
    finally:
        sys.argv=old_argv

def run_tool(command, extra=None):
    script, fixed, _ = COMMANDS[command]
    args=[*fixed]
    if extra:
        args += list(extra)
    print("\n>", sys.executable, str(_tool(script)), *args, flush=True)
    try:
        if command in _IN_PROCESS:
            return _run_in_process(script,args)
        cmd=[sys.executable,str(_tool(script)),*args]
        return subprocess.call(cmd,cwd=str(REPO_DIR))
    except KeyboardInterrupt:
        # main() tool wajib menutup serial/file lewat finally/context manager.
        print("\nDihentikan. Port serial sudah ditutup; CSV yang sudah diterima tetap tersimpan.")
        return 130
    except (TimeoutError,OSError,FileNotFoundError) as exc:
        print(f"\nKomunikasi belum siap: {exc}")
        print("Kembali ke menu tanpa meninggalkan handle COM.")
        return 2


def _ask(prompt, default=""):
    suffix = f" [{default}]" if default else ""
    v = input(f"{prompt}{suffix}: ").strip()
    return v or default


def _pause(rc=0):
    print(f"\nSelesai (exit code {rc}).")
    input("Tekan Enter untuk kembali ke menu...")


def raw_read():
    print("\nRAW = accel/gyro/temperature asli sensor, sebelum dipakai estimator.")
    print("Gunakan ini untuk memastikan sensor hidup, stabil, dan stream tidak error.")
    return run_tool("raw", ["--port", "auto", "--every", "5"])


def ready_read():
    print("\nREADY = data yang sudah melewati kalibrasi + filter/ESKF.")
    print("Ditampilkan: RPY, linear acceleration, velocity, position, uncertainty, health.")
    return run_tool("ready", ["--port", "auto", "--every", "5"])


def record_data():
    duration = _ask("Durasi rekam (detik)", "60")
    name = _ask("Nama CSV", "auto")
    return run_tool("log", ["--port", "auto", "--duration", duration, "--output", name])


def do_still():
    print("\nSTILL calibration")
    print("- Letakkan board pada posisi stabil.")
    print("- Jangan disentuh/digerakkan sampai selesai.")
    print("- Mengkalibrasi gyro bias dan reference temperature.")
    return run_tool("still", ["--port", "auto"])


def do_six_side():
    print("\n6-SIDE accelerometer calibration")
    print("Tahan diam satu per satu: +X, -X, +Y, -Y, +Z, -Z.")
    print("Tunggu coverage sisi diterima sebelum pindah sisi.")
    return run_tool("six-side", ["--port", "auto"])


def precision_menu():
    while True:
        print("\n" + "-" * 58)
        print(" KALIBRASI LANJUTAN (opsional, untuk hasil lebih presisi)")
        print("-" * 58)
        print(" 1. Rekam satu pose accelerometer")
        print(" 2. Fit 18-24 pose accelerometer")
        print(" 3. Rekam satu plateau thermal")
        print(" 4. Fit thermal")
        print(" 0. Kembali")
        c = input("Pilih: ").strip()
        if c == "0":
            return 0
        if c == "1":
            idx = _ask("Nomor pose", "01")
            sec = _ask("Durasi diam (detik)", "6")
            rc = run_tool("log", ["--port", "auto", "--duration", sec,
                                  "--output", f"accel_pose_{idx}.csv"])
            _pause(rc)
        elif c == "2":
            print("Memakai file tools/logs/accel_pose_*.csv.")
            apply = input("Apply ke firmware jika quality gate lolos? [y/N]: ").strip().lower()
            args = ["accel_pose_*.csv"]
            if apply in ("y", "yes"):
                args.append("--apply")
            rc = run_tool("ellipsoid", args)
            _pause(rc)
        elif c == "3":
            name = _ask("Nama plateau", "thermal_mid")
            sec = _ask("Durasi diam (detik)", "60")
            rc = run_tool("log", ["--port", "auto", "--duration", sec,
                                  "--output", f"{name}.csv"])
            _pause(rc)
        elif c == "4":
            print("Memakai file tools/logs/thermal_*.csv.")
            apply = input("Apply ke firmware jika fit valid? [y/N]: ").strip().lower()
            args = ["thermal_*.csv"]
            if apply in ("y", "yes"):
                args.append("--apply")
            rc = run_tool("thermal", args)
            _pause(rc)
        else:
            print("Pilihan tidak valid.")



def axis_output_menu():
    from configure_imu import request, packet, COMM as CFG_COMM, SUB as CFG_SUB
    import configure_imu

    def read_map():
        d=request(bytes((CFG_COMM,CFG_SUB["get"])),CFG_SUB["get"],"auto",921600,5.0)
        return int(d.get("output_map",0)), d

    def write_map(mask):
        return request(bytes((CFG_COMM,CFG_SUB["output-map"],mask&0x0F)),
                       CFG_SUB["output-map"],"auto",921600,5.0)

    while True:
        try:
            mask,_=read_map()
        except Exception as exc:
            print("Gagal baca setting:",exc)
            return 2
        print("\n"+"-"*58)
        print(" AXIS / OUTPUT MAPPING")
        print("-"*58)
        print(f" Invert X       : {'ON' if mask&1 else 'OFF'}")
        print(f" Invert Y       : {'ON' if mask&2 else 'OFF'}")
        print(f" Invert Z       : {'ON' if mask&4 else 'OFF'}")
        print(f" Swap Roll/Pitch: {'ON' if mask&8 else 'OFF'}")
        print()
        print(" 1. Toggle Invert X")
        print(" 2. Toggle Invert Y")
        print(" 3. Toggle Invert Z")
        print(" 4. Toggle Swap Roll/Pitch")
        print(" 5. Reset mapping saja")
        print(" 6. RESET ALL firmware/calibration")
        print(" 0. Kembali")
        c=input("Pilih: ").strip()
        if c=="0": return 0
        bits={"1":1,"2":2,"3":4,"4":8}
        if c in bits:
            new=mask^bits[c]
            d=write_map(new)
            print(f"status={d['status']} output_map=0x{d.get('output_map',new):02X}")
        elif c=="5":
            d=write_map(0)
            print(f"status={d['status']} mapping kembali normal")
        elif c=="6":
            token=input("Ini menghapus SEMUA kalibrasi/config. Ketik RESET ALL: ").strip()
            if token!="RESET ALL":
                print("Dibatalkan.")
                continue
            d=request(bytes((CFG_COMM,CFG_SUB["reset-all"])),CFG_SUB["reset-all"],"auto",921600,8.0)
            print(f"status={d['status']} reset all selesai")
        else:
            print("Pilihan tidak valid.")


def advanced_menu():
    while True:
        print("\n" + "-" * 58)
        print(" ADVANCED / MAINTENANCE")
        print("-" * 58)
        print(" 1. Diagnostik lengkap RAW + ESKF")
        print(" 2. Monitor kalibrasi interaktif")
        print(" 3. Konfigurasi IMU")
        print(" 4. Analisis Allan / noise")
        print(" 5. External aiding ESKF")
        print(" 6. Query format VESC")
        print(" 7. Diagnostik serial / hotplug")
        print(" 8. Flash firmware UART")
        print(" 9. Tampilkan path")
        print(" 0. Kembali")
        c = input("Pilih: ").strip()
        if c == "0":
            return 0
        if c == "1":
            rc = run_tool("read", ["--port", "auto"])
        elif c == "2":
            rc = run_tool("monitor", ["--port", "auto"])
        elif c == "3":
            print("Gunakan command langsung untuk konfigurasi spesifik.")
            print("Contoh: py tools/imu.py config get --port auto")
            rc = run_tool("config", ["get", "--port", "auto"])
        elif c == "4":
            name = _ask("CSV stationary", "04_allan_20min.csv")
            rc = run_tool("allan", [name])
        elif c == "5":
            print("Contoh: wheel 0 --sigma 0.05 --port auto")
            args = _ask("Argumen", "wheel 0 --sigma 0.05 --port auto").split()
            rc = run_tool("aiding", args)
        elif c == "6":
            rc = run_tool("vesc", ["--port", "auto"])
        elif c == "7":
            rc = run_tool("serial")
        elif c == "8":
            image = _ask("Firmware .bin", ".pio/build/APP_USART/firmware.bin")
            if input("Ketik FLASH untuk lanjut: ").strip() != "FLASH":
                print("Dibatalkan.")
                continue
            rc = run_tool("flash", [image, "--port", "auto", "--baud", "921600"])
        elif c == "9":
            rc = run_tool("paths")
        else:
            print("Pilihan tidak valid.")
            continue
        _pause(rc)


def main_menu():
    while True:
        print("\n" + "=" * 62)
        print(" HOVERBOARD SIDEBOARD - IMU")
        print("=" * 62)
        print(" Urutan disarankan: 1 RAW -> 2 STILL -> 3 6-SISI -> 5 READY")
        print("                    4 opsional untuk kalibrasi presisi/thermal")
        print()
        print(" 1. Baca IMU RAW")
        print(" 2. Kalibrasi STILL (gyro bias)")
        print(" 3. Kalibrasi accelerometer 6 sisi")
        print(" 4. Kalibrasi lanjutan (presisi / thermal)")
        print(" 5. Baca DATA FINAL siap pakai")
        print(" 6. Rekam data lengkap ke CSV")
        print(" 7. Status kalibrasi")
        print(" 8. Advanced / maintenance")
        print(" 9. Axis / Output Mapping + Reset All")
        print(" G. GUI Dashboard Qt6")
        print(" 0. Keluar")
        c = input("\nPilih: ").strip().lower()
        if c in ("0", "q", "quit", "exit"):
            return 0
        if c == "1":
            _pause(raw_read())
        elif c == "2":
            _pause(do_still())
        elif c == "3":
            _pause(do_six_side())
        elif c == "4":
            precision_menu()
        elif c == "5":
            _pause(ready_read())
        elif c == "6":
            _pause(record_data())
        elif c == "7":
            _pause(run_tool("status", ["--port", "auto"]))
        elif c == "8":
            advanced_menu()
        elif c == "9":
            axis_output_menu()
        elif c in ("g", "gui"):
            _pause(run_tool("gui"))
        else:
            print("Pilihan tidak valid.")


def main():
    ap = argparse.ArgumentParser(
        description="IMU launcher. Tanpa command membuka menu sederhana."
    )
    ap.add_argument("command", nargs="?", choices=sorted(COMMANDS))
    ap.add_argument("args", nargs=argparse.REMAINDER,
                    help="argumen diteruskan ke tool terkait")
    ns = ap.parse_args()
    if ns.command is None:
        return main_menu()
    return run_tool(ns.command, ns.args)


if __name__ == "__main__":
    raise SystemExit(run_logged(main, __file__))
