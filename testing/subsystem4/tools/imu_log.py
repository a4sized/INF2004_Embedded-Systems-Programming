"""
imu_log.py -- capture and plot the CSV stream from subsystem 4.

1. In app_program/subsystem4/s4_config.h set   S4_PRINT_CSV  to 1
   (and S4_PRINT_STATUS to 0 to keep the log clean), rebuild, flash.
2. pip install pyserial matplotlib
3. Capture (stop with Ctrl+C):
       python imu_log.py capture COM5 run1.csv        (Windows)
       python imu_log.py capture /dev/ttyACM0 run1.csv   (Linux/WSL)
4. Plot:
       python imu_log.py plot run1.csv

Lines that are not CSV (the "[S4] ..." event messages) are kept in the file
as comments and drawn as markers on the plot.  Use the plots to tune the
thresholds in s4_config.h and for the terrain analysis report.
"""
import sys

COLUMNS = ["t_ms", "ax_mg", "ay_mg", "az_mg", "pitch_cdeg", "prel_cdeg",
           "height_mm", "hump"]
STATE = {0: "FLAT", 1: "UP", 2: "DOWN"}


def capture(port, out_path):
    import serial  # pyserial
    with serial.Serial(port, 115200, timeout=1) as ser, open(out_path, "w") as out:
        print(f"Logging {port} -> {out_path}  (Ctrl+C to stop)")
        try:
            while True:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                out.write(line + "\n")
                if "[S4]" in line:
                    print(line)
        except KeyboardInterrupt:
            print("\nStopped.")


def load(path):
    rows, events = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            parts = line.split(",")
            if len(parts) == len(COLUMNS) and parts[0].lstrip("-").isdigit():
                try:
                    rows.append([int(p) for p in parts])
                except ValueError:
                    pass
            elif "[S4]" in line:
                events.append((rows[-1][0] / 1000.0 if rows else 0.0, line.split("[S4]")[1].strip()))
    return rows, events


def plot(path):
    import matplotlib.pyplot as plt
    rows, events = load(path)
    if not rows:
        print("No CSV rows found. Is S4_STREAM_CSV set to 1?")
        return
    col = {n: [r[i] for r in rows] for i, n in enumerate(COLUMNS)}
    t = [v / 1000.0 for v in col["t_ms"]]

    fig, ax = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
    ax[0].plot(t, [v / 1000 for v in col["ax_mg"]], label="forward")
    ax[0].plot(t, [v / 1000 for v in col["ay_mg"]], label="left")
    ax[0].plot(t, [v / 1000 for v in col["az_mg"]], label="up")
    ax[0].set_ylabel("acceleration (g)"); ax[0].legend(loc="upper right")
    ax[1].plot(t, [v / 100 for v in col["prel_cdeg"]], color="tab:purple")
    ax[1].set_ylabel("pitch vs level (deg)")
    ax[2].plot(t, [v / 10 for v in col["height_mm"]], color="tab:green")
    ax[2].set_ylabel("hump height (cm)")
    ax[3].plot(t, col["hump"], drawstyle="steps-post", color="tab:red")
    ax[3].set_yticks(list(STATE)); ax[3].set_yticklabels(list(STATE.values()))
    ax[3].set_ylabel("hump state"); ax[3].set_xlabel("time (s)")
    for a in ax:
        a.grid(True, alpha=0.3)
        for et, text in events:
            if text.startswith(("hump", "COLLISION")):
                a.axvline(et, color="k", alpha=0.25, lw=0.8)
    print(f"{len(rows)} samples over {t[-1] - t[0]:.1f} s; {len(events)} event lines:")
    for et, text in events:
        print(f"  {et:7.2f} s  {text}")
    plt.suptitle(path); plt.tight_layout(); plt.show()


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "capture":
        capture(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3 and sys.argv[1] == "plot":
        plot(sys.argv[2])
    else:
        print(__doc__)
