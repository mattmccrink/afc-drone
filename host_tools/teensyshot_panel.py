#!/usr/bin/env python3
"""
teensyshot spin-up control panel (tkinter).

Commands a closed-loop RPM setpoint to the ESCPID firmware (jacqu/teensyshot)
over USB serial and displays the returned telemetry. Intended for a simple
bench spin-up test of a single motor.

============================ SAFETY MODEL ================================
teensyshot is a DEAD-MAN'S-SWITCH design:
  * The motor spins ONLY while this panel is streaming packets, and it stops
    on its own ~80 ms after streaming ceases.
  * STREAMING == MOTOR LIVE. There is no "connected but idle" state: even a
    zero setpoint idles the motor at minimum throttle.
  * STOP ceases streaming -- that IS the intended, safe way to stop.
  * Closing the window ceases streaming too.
  * Telemetry only updates while streaming (the Teensy replies only to the
    packets you send), so the readout freezes when the motor is stopped.

======================= BENCH NOTES (no rotor) ==========================
  * Start LOW. With no load the motor accelerates fast, and the stock PID
    gains (tuned for a loaded system) may overshoot or hunt. Raise gradually.
  * Minimum-throttle idle may exceed very low setpoints, so the motor can
    spin faster than a low commanded RPM. Pick a setpoint above idle speed.
  * RPM assumes the firmware's ESCCMD_TLM_NB_POLES matches your motor. If it
    doesn't, displayed/commanded RPM is scaled by (real_poles / 14).

Requires: pyserial, tkinter (tkinter ships with Python on Windows/macOS;
on Debian/RPi: sudo apt install python3-tk).
"""

import struct
import time
import threading
import tkinter as tk
from tkinter import ttk
import serial

# ---- MUST MATCH FIRMWARE (ESCPID.h) -------------------------------------
DEFAULT_PORT = "COM11"          # Teensy port ("/dev/ttyACM0" on Linux)
MAX_ESC = 6                    # ESCPID_MAX_ESC  -> 64-byte USB-aligned packet
NB_ESC  = 1                    # ESCPID_NB_ESC   -> single channel (slot 0)

# ---- SAFETY LIMITS (tune to your motor) ---------------------------------
MAX_RPM     = 30000             # UI clamp on commanded mechanical RPM
DEFAULT_RPM = 1000             # startup setpoint shown in the box
RAMP_RPM_S  = 2000             # soft-start / setpoint ramp rate (rpm per second)
SEND_HZ     = 60               # stream rate (must stay >25 Hz for the 40 ms watchdog)

# ---- PID gains (firmware defaults, raw ints) ----------------------------
DEF_P, DEF_I, DEF_D, DEF_F = 400, 1050, 1000, 9900

MAGIC       = 0x43305735
MAGIC_BYTES = struct.pack("<I", MAGIC)
BAUD        = 115200

N = MAX_ESC
OUT_FMT = "<I" + f"{N}h" + f"{N}H" * 4
IN_FMT  = "<I" + f"{N}b" + f"{N}B" + f"{N}H" * 3 + f"{N}h"
OUT_SIZE = struct.calcsize(OUT_FMT)
IN_SIZE  = struct.calcsize(IN_FMT)


# ------------------------- protocol helpers ------------------------------
def build_packet(rpm_ref, kp=DEF_P, ki=DEF_I, kd=DEF_D, kf=DEF_F):
    """64-byte Host_comm with everything applied to ESC slot 0; rest zeroed."""
    def slot(v):
        a = [0] * N
        a[0] = int(v)
        return a
    return struct.pack(OUT_FMT, MAGIC,
                       *slot(rpm_ref), *slot(kp), *slot(ki), *slot(kd), *slot(kf))


def decode(frame):
    """Parse a 64-byte ESCPID_comm frame -> dict for ESC slot 0."""
    f = struct.unpack(IN_FMT, frame[:IN_SIZE])
    o = 1
    def take():
        nonlocal o
        s = f[o:o + N]; o += N
        return s
    err, deg, cmd, volt, amp, rpm = take(), take(), take(), take(), take(), take()
    return dict(err=err[0], temp_C=deg[0], cmd=cmd[0],
                volt_V=volt[0] * 0.01, amp_A=amp[0] * 0.01, rpm=rpm[0] * 10)


def next_frame(ser, buf, timeout=0.01):
    """Return the next magic-aligned IN_SIZE frame from the stream, or None.
    `buf` persists across calls so it absorbs USB latency without desyncing."""
    end = time.time() + timeout
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
        i = buf.find(MAGIC_BYTES)
        if i > 0:
            del buf[:i]
        elif i < 0 and len(buf) > 3:
            del buf[:-3]
        if len(buf) >= IN_SIZE and buf[:4] == MAGIC_BYTES:
            fr = bytes(buf[:IN_SIZE]); del buf[:IN_SIZE]
            return fr
    return None


# ------------------------------- panel -----------------------------------
class Panel:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.alive = False           # comm thread runs while True
        self.streaming = False       # True == commanding the motor (LIVE)
        self.lock = threading.Lock()
        self.target_rpm = float(DEFAULT_RPM)
        self.cmd_rpm = 0.0           # ramped setpoint actually being sent
        self.tlm = None
        self.tlm_time = 0.0
        self.err_msg = ""
        self._frame_times = []
        self._build_ui()
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.refresh)

    def _build_ui(self):
        root = self.root
        root.title("teensyshot spin-up panel")
        pad = dict(padx=6, pady=4)

        conn = ttk.LabelFrame(root, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)
        ttk.Label(conn, text="Port:").grid(row=0, column=0, **pad)
        self.port_var = tk.StringVar(value=DEFAULT_PORT)
        ttk.Entry(conn, textvariable=self.port_var, width=12).grid(row=0, column=1, **pad)
        self.conn_btn = ttk.Button(conn, text="Connect", command=self.toggle_connect)
        self.conn_btn.grid(row=0, column=2, **pad)
        self.conn_lbl = ttk.Label(conn, text="\u25cf disconnected", foreground="gray")
        self.conn_lbl.grid(row=0, column=3, **pad)

        cmd = ttk.LabelFrame(root, text="Command")
        cmd.grid(row=1, column=0, sticky="ew", **pad)
        ttk.Label(cmd, text="Commanded RPM:").grid(row=0, column=0, **pad)
        self.rpm_var = tk.DoubleVar(value=DEFAULT_RPM)
        self.rpm_scale = ttk.Scale(cmd, from_=0, to=MAX_RPM, variable=self.rpm_var,
                                   orient="horizontal", length=260,
                                   command=lambda _e: self._set_target())
        self.rpm_scale.grid(row=0, column=1, **pad)
        self.rpm_entry = ttk.Entry(cmd, textvariable=self.rpm_var, width=8)
        self.rpm_entry.grid(row=0, column=2, **pad)
        self.rpm_entry.bind("<Return>", lambda _e: self._set_target())
        ttk.Label(cmd, text=f"(max {MAX_RPM})").grid(row=0, column=3, **pad)

        btns = ttk.Frame(root)
        btns.grid(row=2, column=0, sticky="ew", **pad)
        self.spin_btn = tk.Button(btns, text="ARM & SPIN", bg="#2e7d32", fg="white",
                                  width=14, height=2, command=self.arm_spin)
        self.spin_btn.grid(row=0, column=0, **pad)
        self.stop_btn = tk.Button(btns, text="STOP", bg="#c62828", fg="white",
                                  width=14, height=2, command=self.stop)
        self.stop_btn.grid(row=0, column=1, **pad)
        self.motor_lbl = tk.Label(btns, text="MOTOR OFF", fg="white", bg="#2e7d32",
                                  width=14, font=("TkDefaultFont", 10, "bold"))
        self.motor_lbl.grid(row=0, column=2, **pad)

        tlm = ttk.LabelFrame(root, text="Telemetry")
        tlm.grid(row=3, column=0, sticky="ew", **pad)
        self.vars = {k: tk.StringVar(value="--") for k in
                     ("rpm", "volt", "amp", "temp", "cmd", "err", "link", "age")}
        rows = [("Measured RPM", "rpm"), ("Voltage (V)", "volt"),
                ("Current (A)", "amp"), ("Temp (\u00b0C)", "temp"),
                ("ESC cmd", "cmd"), ("Err", "err"),
                ("Link (fps)", "link"), ("Frame age (ms)", "age")]
        for idx, (label, key) in enumerate(rows):
            r, c = divmod(idx, 2)
            ttk.Label(tlm, text=label + ":").grid(row=r, column=c * 2, sticky="e", **pad)
            ttk.Label(tlm, textvariable=self.vars[key], width=10,
                      font=("TkFixedFont", 11, "bold")).grid(
                      row=r, column=c * 2 + 1, sticky="w", **pad)

        self.status = ttk.Label(root, text="Ready. Connect to begin.", foreground="gray")
        self.status.grid(row=4, column=0, sticky="w", **pad)
        self._set_controls(False)

    def _set_controls(self, connected):
        state = "normal" if connected else "disabled"
        self.spin_btn.config(state=state)
        self.stop_btn.config(state=state)

    def _set_target(self):
        try:
            v = float(self.rpm_var.get())
        except (tk.TclError, ValueError):
            return
        self.target_rpm = max(0.0, min(v, MAX_RPM))

    # ---------- connection ----------
    def toggle_connect(self):
        if self.ser:                       # currently connected -> disconnect
            self.stop()
            self.alive = False
            time.sleep(0.1)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.conn_btn.config(text="Connect")
            self.conn_lbl.config(text="\u25cf disconnected", foreground="gray")
            self._set_controls(False)
            self.status.config(text="Disconnected.", foreground="gray")
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), BAUD, timeout=0.05)
            time.sleep(0.2)
            self.ser.reset_input_buffer()
        except Exception as e:
            self.status.config(text=f"Open failed: {e}", foreground="red")
            self.ser = None
            return
        self.alive = True
        self.streaming = False
        self.cmd_rpm = 0.0
        threading.Thread(target=self.comm_loop, daemon=True).start()
        self.conn_btn.config(text="Disconnect")
        self.conn_lbl.config(text="\u25cf connected", foreground="green")
        self._set_controls(True)
        self.status.config(text="Connected. Motor OFF. Press ARM & SPIN when ready.",
                           foreground="black")

    # ---------- motor control ----------
    def arm_spin(self):
        if not self.ser:
            return
        self._set_target()
        self.streaming = True
        self.status.config(text=f"SPINNING  ->  ramping to {self.target_rpm:.0f} rpm",
                           foreground="#b00020")

    def stop(self):
        self.streaming = False             # cease streaming -> motor stops (~80 ms)
        self.cmd_rpm = 0.0
        self.status.config(text="STOP: streaming ceased, motor stopping.",
                           foreground="black")

    # ---------- comm thread ----------
    def comm_loop(self):
        rxbuf = bytearray()
        period = 1.0 / SEND_HZ
        last = time.perf_counter()
        while self.alive:
            now = time.perf_counter()
            dt = now - last
            last = now
            if self.streaming and self.ser:
                tgt = self.target_rpm                      # soft ramp toward target
                if self.cmd_rpm < tgt:
                    self.cmd_rpm = min(self.cmd_rpm + RAMP_RPM_S * dt, tgt)
                elif self.cmd_rpm > tgt:
                    self.cmd_rpm = max(self.cmd_rpm - RAMP_RPM_S * dt, tgt)
                rpm_r = int(round(self.cmd_rpm / 10.0))    # firmware units: 10 rpm
                try:
                    self.ser.write(build_packet(rpm_r))    # this write feeds the watchdog
                    fr = next_frame(self.ser, rxbuf, timeout=0.01)
                    if fr:
                        d = decode(fr)
                        with self.lock:
                            self.tlm = d
                            self.tlm_time = now
                            self._frame_times.append(now)
                except Exception as e:
                    with self.lock:
                        self.err_msg = str(e)
                    self.streaming = False
            else:
                self.cmd_rpm = 0.0                          # not sending -> motor stops
            slack = period - (time.perf_counter() - now)
            if slack > 0:
                time.sleep(slack)

    # ---------- periodic GUI refresh (main thread) ----------
    def refresh(self):
        with self.lock:
            d = self.tlm
            t = self.tlm_time
            err = self.err_msg
            cutoff = time.perf_counter() - 1.0
            self._frame_times = [x for x in self._frame_times if x > cutoff]
            fps = len(self._frame_times)

        if self.streaming and d:
            self.vars["rpm"].set(f"{d['rpm']}")
            self.vars["volt"].set(f"{d['volt_V']:.2f}")
            self.vars["amp"].set(f"{d['amp_A']:.2f}")
            self.vars["temp"].set(f"{d['temp_C']}")
            self.vars["cmd"].set(f"{d['cmd']}")
            self.vars["err"].set(f"{d['err']}")
            self.vars["link"].set(f"{fps}")
            self.vars["age"].set(f"{(time.perf_counter() - t) * 1000:.0f}" if t else "--")

        if self.streaming:
            self.motor_lbl.config(text="\u25cf MOTOR LIVE", bg="#c62828")
        else:
            self.motor_lbl.config(text="MOTOR OFF", bg="#2e7d32")
            self.vars["link"].set("--")
            self.vars["age"].set("--")

        if err:
            self.status.config(text=f"Comm error: {err}", foreground="red")
            with self.lock:
                self.err_msg = ""

        self.root.after(100, self.refresh)

    def on_close(self):
        self.streaming = False
        self.alive = False
        time.sleep(0.15)
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.root.destroy()


def main():
    root = tk.Tk()
    Panel(root)
    root.mainloop()


if __name__ == "__main__":
    main()
