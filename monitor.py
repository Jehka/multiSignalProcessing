"""
monitor.py — Live Air Quality Monitor
Reads 9-byte frames from FPGA UART at 115200 baud.

Frame format:
  [0xAA] [status] [ch0_hi] [ch0_lo] [ch1_hi] [ch1_lo] [ch2_hi] [ch2_lo] [0x55]
  status byte: bits[3:2]=alarm_ch, bits[1:0]=alarm_level

Usage:
  pip install pyserial matplotlib
  python monitor.py --port COM3        # Windows
  python monitor.py --port /dev/ttyUSB0  # Linux/WSL

Output:
  - Live 3-panel plot (ch0/ch1/ch2 FIR results + alarm level)
  - CSV log: data/log_YYYYMMDD_HHMMSS.csv
  - Console alarm events with timestamp
"""

import serial
import argparse
import threading
import queue
import csv
import os
import time
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# =============================================================================
# Config
# =============================================================================
BAUD_RATE    = 115200
FRAME_LEN    = 9
START_MARKER = 0xAA
END_MARKER   = 0x55
PLOT_WINDOW  = 500       # samples to show in rolling plot
LOG_DIR      = "data"

ALARM_LABELS = {0: "CLEAR", 1: "WARNING", 2: "DANGER", 3: "CRITICAL"}
ALARM_COLORS = {0: "green", 1: "yellow", 2: "orange", 3: "red"}
CH_LABELS    = {0: "MQ-3 (Alcohol/VOC)", 1: "MQ-135 (Air Quality)", 2: "LDR (Light)"}

# =============================================================================
# Frame Parser
# =============================================================================
def parse_frame(buf):
    """Parse a 9-byte frame. Returns dict or None on error."""
    if len(buf) != FRAME_LEN:
        return None
    if buf[0] != START_MARKER or buf[8] != END_MARKER:
        return None
    status     = buf[1]
    alarm_level = status & 0x03
    alarm_ch    = (status >> 2) & 0x03
    ch0 = (buf[2] << 8) | buf[3]
    ch1 = (buf[4] << 8) | buf[5]
    ch2 = (buf[6] << 8) | buf[7]
    return {
        "ts":          time.time(),
        "alarm_level": alarm_level,
        "alarm_ch":    alarm_ch,
        "ch0":         ch0,
        "ch1":         ch1,
        "ch2":         ch2,
    }

# =============================================================================
# Serial Reader Thread
# =============================================================================
def serial_reader(port, baud, data_queue, stop_event):
    """Reads serial port, finds frame boundaries, pushes parsed frames."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[INFO] Opened {port} at {baud} baud")
    except serial.SerialException as e:
        print(f"[ERROR] Could not open port: {e}")
        stop_event.set()
        return

    buf = []
    synced = False

    while not stop_event.is_set():
        byte = ser.read(1)
        if not byte:
            continue
        val = byte[0]

        if not synced:
            if val == START_MARKER:
                buf = [val]
                synced = True
        else:
            buf.append(val)
            if len(buf) == FRAME_LEN:
                frame = parse_frame(buf)
                if frame:
                    data_queue.put(frame)
                else:
                    # Sync lost — search for next start marker
                    synced = False
                buf = []

    ser.close()

# =============================================================================
# CSV Logger
# =============================================================================
def setup_csv():
    os.makedirs(LOG_DIR, exist_ok=True)
    fname = os.path.join(LOG_DIR, datetime.now().strftime("log_%Y%m%d_%H%M%S.csv"))
    f = open(fname, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["timestamp", "alarm_level", "alarm_label",
                     "alarm_ch", "ch0_fir", "ch1_fir", "ch2_fir"])
    print(f"[INFO] Logging to {fname}")
    return f, writer

# =============================================================================
# Live Plot
# =============================================================================
def run_dashboard(port, baud):
    data_queue = queue.Queue()
    stop_event = threading.Event()

    # Start serial reader thread
    t = threading.Thread(target=serial_reader,
                         args=(port, baud, data_queue, stop_event),
                         daemon=True)
    t.start()

    # CSV setup
    csv_file, csv_writer = setup_csv()

    # Rolling buffers
    ts_buf   = deque(maxlen=PLOT_WINDOW)
    ch0_buf  = deque(maxlen=PLOT_WINDOW)
    ch1_buf  = deque(maxlen=PLOT_WINDOW)
    ch2_buf  = deque(maxlen=PLOT_WINDOW)
    alm_buf  = deque(maxlen=PLOT_WINDOW)
    t_start  = time.time()

    # Plot setup
    fig, axes = plt.subplots(4, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("Air Quality Monitor — Live", fontsize=13)

    lines = [ax.plot([], [], lw=1.2)[0] for ax in axes[:3]]
    axes[0].set_ylabel("MQ-3\n(Alcohol/VOC)")
    axes[1].set_ylabel("MQ-135\n(Air Quality)")
    axes[2].set_ylabel("LDR\n(Light)")
    axes[3].set_ylabel("Alarm Level")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylim(-0.2, 3.2)
    axes[3].set_yticks([0, 1, 2, 3])
    axes[3].set_yticklabels(["CLEAR", "WARN", "DANGER", "CRIT"])

    alarm_scatter = axes[3].scatter([], [], c=[], cmap="RdYlGn_r",
                                    vmin=0, vmax=3, s=8)

    for ax in axes[:3]:
        ax.set_ylim(0, 65535)
        ax.set_xlim(0, 50)
        ax.grid(True, alpha=0.3)
    axes[3].set_xlim(0, 50)
    axes[3].grid(True, alpha=0.3)

    alarm_text = fig.text(0.75, 0.95, "CLEAR", fontsize=16,
                          color="green", ha="center", va="top",
                          bbox=dict(boxstyle="round", facecolor="wheat"))

    def update(frame):
        # Drain queue
        while not data_queue.empty():
            d = data_queue.get_nowait()
            elapsed = d["ts"] - t_start
            ts_buf.append(elapsed)
            ch0_buf.append(d["ch0"])
            ch1_buf.append(d["ch1"])
            ch2_buf.append(d["ch2"])
            alm_buf.append(d["alarm_level"])

            # Log to CSV
            csv_writer.writerow([
                f"{elapsed:.3f}",
                d["alarm_level"],
                ALARM_LABELS[d["alarm_level"]],
                d["alarm_ch"],
                d["ch0"], d["ch1"], d["ch2"]
            ])
            csv_file.flush()

            # Console alarm events
            if d["alarm_level"] > 0:
                print(f"[{elapsed:8.2f}s] {ALARM_LABELS[d['alarm_level']]:8s} "
                      f"| CH{d['alarm_ch']} | "
                      f"MQ3={d['ch0']:5d} MQ135={d['ch1']:5d} LDR={d['ch2']:5d}")

        if not ts_buf:
            return lines + [alarm_scatter, alarm_text]

        ts = list(ts_buf)
        xmin = max(0, ts[-1] - 50)
        xmax = ts[-1] + 1

        for ax in axes:
            ax.set_xlim(xmin, xmax)

        lines[0].set_data(ts, list(ch0_buf))
        lines[1].set_data(ts, list(ch1_buf))
        lines[2].set_data(ts, list(ch2_buf))

        # Alarm scatter
        alm = list(alm_buf)
        alarm_scatter.set_offsets(list(zip(ts, alm)))
        alarm_scatter.set_array([float(a) for a in alm])

        # Alarm label
        latest_alarm = alm[-1] if alm else 0
        alarm_text.set_text(ALARM_LABELS[latest_alarm])
        alarm_text.set_color(ALARM_COLORS[latest_alarm])

        return lines + [alarm_scatter, alarm_text]

    ani = animation.FuncAnimation(fig, update, interval=100,
                                  blit=False, cache_frame_data=False)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        stop_event.set()
        csv_file.close()
        print("[INFO] Stopped.")

# =============================================================================
# Entry Point
# =============================================================================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Air Quality Live Monitor")
    parser.add_argument("--port", default="COM3",
                        help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    args = parser.parse_args()
    run_dashboard(args.port, args.baud)