#!/usr/bin/env python3
"""TW3 tuning GUI, v1 -- the parameter console.

Connects to the MAIN MCU (USB serial or the radio bridge's COM port, both
115200) and drives the parameter registry that shipped 2026-08-27:

  * on connect it sends `param.dump` and builds the table from the
    PARAMDEF,name,value,min,max lines the mouse answers with -- the GUI has
    NO parameter list of its own, so a parameter appended to the firmware
    registry appears here with no change to this file;
  * edit a value and press Enter (or Set) to send `name=value` -- RAM only,
    exactly like typing it at the console, and the mouse's reply (including
    a clamp complaint) lands in the log pane;
  * Save = `w` (persist to FRAM), Refresh = re-`param.dump`, Diff =
    `param.diff` into the log pane;
  * the command line at the bottom sends anything raw ('p', 'M', 'y',
    'route.arm', ...), so this fully replaces a terminal for a tuning
    session. Everything the mouse prints is timestamped into the log and
    appended to tune_session_YYYYMMDD_HHMMSS.log next to this script.

v1 is deliberately just this: discovery, set, save, log. The lap-report
display and the auto-step tuner (TUNING_PLAN.md) build on top of this file
once the firmware grows the lap report.

Usage:
  pip install pyserial          (once)
  python tools/tune_gui.py            # pick the COM port in the dropdown
  python tools/tune_gui.py COM5       # or name it up front
"""

import sys
import time
import queue
import threading
import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

import tkinter as tk
from tkinter import ttk

BAUD = 115200


class TuneGui:
    def __init__(self, root, port=None):
        self.root = root
        self.ser = None
        self.rx = queue.Queue()
        self.rows = {}          # name -> (value_var, entry, minmax)
        self.dump_open = False  # inside a PARAMDEF burst
        logname = datetime.datetime.now().strftime(
            "tune_session_%Y%m%d_%H%M%S.log")
        self.logfile = open(logname, "a", encoding="utf-8")

        root.title("TW3 tune")
        top = ttk.Frame(root); top.pack(fill="x", padx=6, pady=4)
        self.port_var = tk.StringVar(value=port or "")
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if not port and ports:
            self.port_var.set(ports[0])
        ttk.Combobox(top, textvariable=self.port_var, values=ports,
                     width=12).pack(side="left")
        self.btn_conn = ttk.Button(top, text="Connect", command=self.connect)
        self.btn_conn.pack(side="left", padx=4)
        ttk.Button(top, text="Refresh", command=lambda: self.send("param.dump")
                   ).pack(side="left", padx=4)
        ttk.Button(top, text="Diff", command=lambda: self.send("param.diff")
                   ).pack(side="left", padx=4)
        ttk.Button(top, text="Save to FRAM ('w')",
                   command=lambda: self.send("w")).pack(side="left", padx=4)
        self.status = tk.StringVar(value="not connected")
        ttk.Label(top, textvariable=self.status).pack(side="right")

        body = ttk.Frame(root); body.pack(fill="both", expand=True,
                                          padx=6, pady=2)
        # Parameter table (left)
        self.table = ttk.Frame(body)
        self.table.pack(side="left", fill="y", padx=(0, 6))
        hdr = ttk.Frame(self.table); hdr.pack(fill="x")
        for text, w in (("param", 10), ("value", 12), ("range", 18)):
            ttk.Label(hdr, text=text, width=w,
                      font=("TkDefaultFont", 9, "bold")).pack(side="left")
        self.rowbox = ttk.Frame(self.table); self.rowbox.pack(fill="y")

        # Log pane (right)
        right = ttk.Frame(body); right.pack(side="left", fill="both",
                                            expand=True)
        self.log = tk.Text(right, width=72, height=32, state="disabled",
                           font=("Consolas", 9))
        sb = ttk.Scrollbar(right, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        self.log.pack(side="left", fill="both", expand=True)
        sb.pack(side="left", fill="y")

        bottom = ttk.Frame(root); bottom.pack(fill="x", padx=6, pady=4)
        ttk.Label(bottom, text="cmd:").pack(side="left")
        self.cmd = ttk.Entry(bottom)
        self.cmd.pack(side="left", fill="x", expand=True, padx=4)
        self.cmd.bind("<Return>", self.send_cmd)

        root.after(50, self.poll)

    # -- serial ---------------------------------------------------------------
    def connect(self):
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self.btn_conn.configure(text="Connect")
            self.status.set("not connected")
            return
        try:
            self.ser = serial.Serial(self.port_var.get(), BAUD, timeout=0.1)
        except Exception as e:
            self.status.set(f"open failed: {e}")
            return
        self.btn_conn.configure(text="Disconnect")
        self.status.set(f"{self.port_var.get()} @ {BAUD}")
        threading.Thread(target=self.reader, daemon=True).start()
        # Give the port a moment, then discover the registry.
        self.root.after(300, lambda: self.send("param.dump"))

    def reader(self):
        buf = b""
        while self.ser:
            try:
                data = self.ser.read(256)
            except Exception:
                break
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.rx.put(line.decode("utf-8", "replace").rstrip())

    def send(self, text):
        if not self.ser:
            self.logline(f"[not connected] {text}")
            return
        try:
            self.ser.write((text + "\n").encode())
        except Exception as e:
            self.logline(f"[tx failed: {e}]")
            return
        self.logline(f"> {text}")

    def send_cmd(self, _ev):
        t = self.cmd.get().strip()
        if t:
            self.send(t)
            self.cmd.delete(0, "end")

    # -- incoming -------------------------------------------------------------
    def poll(self):
        while not self.rx.empty():
            line = self.rx.get()
            if line.startswith("PARAMDEF,"):
                self.handle_paramdef(line)
            else:
                self.logline(line)
        self.root.after(50, self.poll)

    def handle_paramdef(self, line):
        parts = line.split(",")
        if len(parts) == 2 and parts[1] == "end":
            self.dump_open = False
            self.logline(f"[registry: {len(self.rows)} params]")
            return
        if len(parts) != 5:
            self.logline(line)
            return
        _, name, val, lo, hi = parts
        if not self.dump_open:
            self.dump_open = True
        if name not in self.rows:
            row = ttk.Frame(self.rowbox); row.pack(fill="x", pady=1)
            ttk.Label(row, text=name, width=10).pack(side="left")
            var = tk.StringVar()
            ent = ttk.Entry(row, textvariable=var, width=12)
            ent.pack(side="left")
            ent.bind("<Return>",
                     lambda _e, n=name, v=var: self.send(f"{n}={v.get()}"))
            ttk.Label(row, text=f"[{lo} .. {hi}]", width=18,
                      foreground="#666").pack(side="left")
            ttk.Button(row, text="Set", width=4,
                       command=lambda n=name, v=var:
                       self.send(f"{n}={v.get()}")).pack(side="left")
            self.rows[name] = var
        self.rows[name].set(val)

    def logline(self, s):
        stamp = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"{stamp} {s}\n")
        self.log.see("end")
        self.log.configure(state="disabled")
        self.logfile.write(f"{stamp} {s}\n")
        self.logfile.flush()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None
    root = tk.Tk()
    TuneGui(root, port)
    root.mainloop()


if __name__ == "__main__":
    main()
