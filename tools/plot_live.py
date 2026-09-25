#!/usr/bin/env python3
"""Live USB plots for the existing pid_demo and attitude_test firmware."""
import argparse
from collections import deque
import math
from pathlib import Path
import re
import time

NUMBER = r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)'
PID = re.compile(rf'^(ROLL|PITCH)\s+angle=({NUMBER}) deg rate=({NUMBER}) deg/s error=({NUMBER}) P=({NUMBER}) I=({NUMBER}) D=({NUMBER}) correction=({NUMBER})$')
ATTITUDE = re.compile(rf'^roll=({NUMBER}) deg \| pitch=({NUMBER}) deg \| yaw_relative=({NUMBER}) deg \| gyro_corrected\[deg/s\] X=({NUMBER}) Y=({NUMBER}) Z=({NUMBER})$')


def parse_line(line):
    """Return (axis, angle, rate, correction) tuples; ignore startup/raw IMU text."""
    m = PID.fullmatch(line.strip())
    if m:
        values = tuple(float(v) for v in m.groups()[1:])
        if all(math.isfinite(v) for v in values):
            return [(m[1].lower(), values[0], values[1], values[-1])]
    m = ATTITUDE.fullmatch(line.strip())
    if m:
        values = tuple(float(v) for v in m.groups())
        if all(math.isfinite(v) for v in values):
            return [('roll', values[0], values[3], None), ('pitch', values[1], values[4], None)]
    return []


class Lines:
    """Handle serial chunks that split lines without blocking the graph."""
    def __init__(self):
        self.pending = b''

    def feed(self, chunk):
        parts = (self.pending + chunk).split(b'\n')
        self.pending = parts.pop()
        if len(self.pending) > 8192:
            self.pending = b''
        return [p.decode('utf-8', errors='replace').strip() for p in parts]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', nargs='?', help='USB serial port; auto-select if exactly one USB device is found')
    parser.add_argument('--list', action='store_true', help='List available serial ports')
    parser.add_argument('--demo', action='store_true', help='Use clearly labeled synthetic data without hardware')
    parser.add_argument('--window', type=float, default=20, help='Visible history in seconds (default: 20)')
    parser.add_argument('--snapshot', type=Path, help='Save a headless synthetic preview; requires --demo')
    args = parser.parse_args()
    if args.window <= 0 or not math.isfinite(args.window): parser.error('--window must be positive and finite')
    if args.snapshot and not args.demo: parser.error('--snapshot requires --demo')
    device = None
    if not args.demo:
        try:
            import serial
            from serial.tools import list_ports
        except ImportError:
            parser.error('Install dependencies first: python3 -m pip install -r requirements.txt')
        ports = list(list_ports.comports())
        if args.list:
            for p in ports: print(f'{p.device}: {p.description}')
            if not ports: print('No serial ports found. Connect the ESP32 with a USB data cable.')
            return
        port = args.port
        if not port:
            usb = [p for p in ports if p.vid is not None]
            if len(usb) != 1:
                parser.error('Specify a port. Run with --list to see available ports.')
            port = usb[0].device
        try:
            device = serial.Serial(port, 115200, timeout=0)
        except (serial.SerialException, OSError) as e:
            parser.error(f'Cannot open {port}: {e}. Close PlatformIO Monitor and other serial apps.')
    try:
        import matplotlib
        if args.snapshot: matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        if not args.snapshot and matplotlib.get_backend().lower() == 'agg':
            raise RuntimeError('No interactive Matplotlib backend. Run from a local desktop Python with Tk or Qt support.')
        history = {axis: deque(maxlen=10000) for axis in ('roll','pitch')}
        fig, axes = plt.subplots(3,1,figsize=(10,8),sharex=True)
        title = 'SYNTHETIC DEMO — not ESP32 data' if args.demo else f'Live ESP32 data — {device.port}'
        fig.suptitle(title, fontsize=14)
        status = fig.text(0.08,0.025,'Waiting for data; keep IMU still during calibration.',fontsize=9)
        artists = {}
        for index,(ax,label) in enumerate(zip(axes,['Angle (degrees)','Rotation rate (deg/s)','PID correction (unitless)'])):
            for axis,color in [('roll','#2563eb'),('pitch','#d97706')]:
                artists[axis,index], = ax.plot([],[],label=axis.capitalize(),color=color,linewidth=1.6)
            ax.axhline(0,color='#64748b',linestyle='--',linewidth=0.8)
            ax.set_ylabel(label); ax.grid(alpha=0.2); ax.legend(loc='upper right')
        axes[2].set_ylim(-1.05,1.05)
        axes[-1].set_xlabel('Time since plot opened (seconds; computer receive time)')
        fig.tight_layout(rect=(0,0.055,1,0.95))
        frames = Lines(); start = time.monotonic(); last_data = None; disconnected = False

        def ingest(line, elapsed):
            nonlocal last_data
            if 'ICM42688P SENSOR TEST' in line:
                for h in history.values(): h.clear()
                last_data = None
                status.set_text('Board restarted. Waiting for calibration and fresh samples.')
            samples = parse_line(line)
            for axis,angle,rate,correction in samples:
                history[axis].append((elapsed,angle,rate,math.nan if correction is None else correction))
            if samples:
                last_data = time.monotonic()
                status.set_text('Live PID data — serial display rate ~5 Hz; controller ~100 Hz.' if samples[0][3] is not None
                                else 'Live attitude data — upload pid_demo to see corrections.')
            elif line:
                status.set_text(line[:140])

        def draw(elapsed):
            for axis,rows in history.items():
                while rows and rows[0][0] < elapsed-args.window: rows.popleft()
                for index in range(3):
                    artists[axis,index].set_data([r[0] for r in rows],[r[index+1] for r in rows])
            for ax in axes[:2]:
                ax.relim(); ax.autoscale_view(scalex=False,scaley=True)
            axes[-1].set_xlim(max(0,elapsed-args.window),max(args.window,elapsed))

        def update(_):
            nonlocal disconnected
            elapsed = time.monotonic()-start
            if args.demo:
                for axis,phase in [('roll',0),('pitch',1)]:
                    angle = 15*math.sin(elapsed+phase); rate = 15*math.cos(elapsed+phase)
                    history[axis].append((elapsed,angle,rate,-0.02*angle-0.003*rate))
                status.set_text('SYNTHETIC DEMO — no hardware readings. Close window to exit.')
            elif not disconnected:
                try:
                    for line in frames.feed(device.read(min(device.in_waiting,32768))): ingest(line,elapsed)
                except (serial.SerialException, OSError) as e:
                    disconnected = True
                    status.set_text(f'Disconnected: {e}. Reconnect and restart this plot.')
                if not disconnected and last_data is not None and time.monotonic()-last_data > 2:
                    status.set_text('No fresh telemetry for >2 seconds. Check board/USB and firmware messages.')
            draw(elapsed)

        if args.snapshot:
            for i in range(201):
                t=i/10
                for axis,phase in [('roll',0),('pitch',1)]:
                    a=15*math.sin(t+phase); rate=15*math.cos(t+phase)
                    history[axis].append((t,a,rate,-0.02*a-0.003*rate))
            status.set_text('SYNTHETIC PREVIEW — no hardware readings.')
            draw(20); args.snapshot.parent.mkdir(parents=True,exist_ok=True)
            fig.savefig(args.snapshot,dpi=150); plt.close(fig)
        else:
            animation = FuncAnimation(fig,update,interval=50,cache_frame_data=False)
            print('Close the plot window or press Ctrl+C to stop. Keep the IMU still during calibration.')
            plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if device is not None: device.close()


if __name__ == '__main__':
    main()
