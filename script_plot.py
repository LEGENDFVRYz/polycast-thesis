# imu_uwb_visualizer.py
import serial
import time
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import MeshData
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from scipy.spatial.transform import Rotation as R

# -----------------------------
# User configuration
# -----------------------------
SERIAL_PORT = 'COM3'   # default to COM3 (change if needed)
BAUD_RATE = 115200

DISPLAY_SCALE = 5.0
LP_ALPHA = 0.5          # not used for fusion (kept if you want local LP)
VELOCITY_DECAY = 0.90
DRAW_VEL_THRESH = 0.005   # threshold for drawing strokes (m/s)

# Fusion and motion detection config (from your reference)
STATIONARY_ALPHA = 0.025
MOVING_ALPHA = 0.3
MOTION_THRESHOLD = 0.85
FUSION_FACTOR = 0.02
GRAVITY_VECTOR = np.array([0.0, 0.0, 9.81])

# Safety/time traps
TRAP_DT_MAX = 0.5   # ignore >0.5s gaps
# -----------------------------
# Serial setup
# -----------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print("Warning: could not open serial port:", e)
    ser = None

_serial_buf = ""

# -----------------------------
# Fusion helper classes (adapted)
# -----------------------------
class JitterFilter:
    """EMA low-pass filter with dynamic alpha."""
    def __init__(self):
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.is_initialized = False

    def filter(self, raw_x, raw_y, alpha):
        if not self.is_initialized:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.is_initialized = True
        else:
            self.filtered_x = (alpha * raw_x) + (1.0 - alpha) * self.filtered_x
            self.filtered_y = (alpha * raw_y) + (1.0 - alpha) * self.filtered_y
        return self.filtered_x, self.filtered_y

class IMUProcessor:
    """
    Processes IMU (accel + quaternion) to predict relative XY position,
    and supports fuse_with_uwb(...) to correct drift.
    """
    def __init__(self):
        self.position = [0.0, 0.0]   # x, y (wall plane)
        self.velocity = [0.0, 0.0]
        self.last_timestamp_ms = None
        self.linear_accel_magnitude = 0.0

    def process(self, timestamp_ms, accel_body, quat_body):
        if self.last_timestamp_ms is None:
            self.last_timestamp_ms = timestamp_ms
            return tuple(self.position)

        dt_ms = timestamp_ms - self.last_timestamp_ms
        if dt_ms < 0:
            dt_ms += 2**32
        self.last_timestamp_ms = timestamp_ms
        dt_s = dt_ms / 1000.0

        if dt_s <= 0 or dt_s > TRAP_DT_MAX:
            # ignore unreasonable dt
            if dt_s > TRAP_DT_MAX:
                self.last_timestamp_ms = None
            return tuple(self.position)

        # Remove gravity (rotate accel into world frame)
        try:
            r = R.from_quat(quat_body)
        except Exception:
            # quat might have wrong order; assume [qx,qy,qz,qw]
            r = R.from_quat(quat_body)

        accel_world = r.apply(accel_body)
        linear_accel_world = accel_world - GRAVITY_VECTOR

        self.linear_accel_magnitude = np.linalg.norm(linear_accel_world)

        # Integrate only X and Y (wall plane) — matches your reference
        ax = linear_accel_world[0]
        ay = linear_accel_world[1]

        self.velocity[0] += ax * dt_s
        self.velocity[1] += ay * dt_s

        # optional velocity decay to limit runaway
        self.velocity[0] *= VELOCITY_DECAY
        self.velocity[1] *= VELOCITY_DECAY

        self.position[0] += self.velocity[0] * dt_s
        self.position[1] += self.velocity[1] * dt_s

        return self.position[0], self.position[1]

    def fuse_with_uwb(self, uwb_x, uwb_y, fusion_factor):
        error_x = uwb_x - self.position[0]
        error_y = uwb_y - self.position[1]

        self.position[0] += error_x * fusion_factor
        self.position[1] += error_y * fusion_factor

        # dampen velocity to avoid windup
        self.velocity[0] *= (1.0 - fusion_factor)
        self.velocity[1] *= (1.0 - fusion_factor)

        return self.position[0], self.position[1]

    def get_motion_state(self):
        return self.linear_accel_magnitude

    def reset(self):
        self.position = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.last_timestamp_ms = None
        self.linear_accel_magnitude = 0.0

# -----------------------------
# Parsing helper (12 CSV fields expected)
# Format expected (same as your reference):
# raw_x, raw_y, <unused?>, <unused?>, qx, qy, qz, qw, ax, ay, az, timestamp
# (Some variants have slightly different columns — adapt if needed)
# -----------------------------
def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 12:
        return None
    try:
        # first 11 are floats, last is int timestamp
        floats = [float(p) for p in parts[:11]]
        timestamp = int(float(parts[11]))
        # Map to expected names:
        raw_x = floats[0]
        raw_y = floats[1]
        # quaternions in positions 4..7 (index 4..7)
        qx, qy, qz, qw = floats[4], floats[5], floats[6], floats[7]
        ax, ay, az = floats[8], floats[9], floats[10]
        return {
            "raw_x": raw_x,
            "raw_y": raw_y,
            "quat": [qx, qy, qz, qw],
            "accel": [ax, ay, az],
            "timestamp": timestamp
        }
    except ValueError:
        return None

# -----------------------------
# Qt App setup (copied/adapted from your UI)
# -----------------------------
app = QtWidgets.QApplication([])

main_window = QtWidgets.QWidget()
main_window.setWindowTitle("IMU+UWB Fusion Visualizer")
main_layout = QtWidgets.QVBoxLayout(main_window)
main_layout.setContentsMargins(5, 5, 5, 5)

w = gl.GLViewWidget()
w.opts['distance'] = 2
w.setCameraPosition(distance=3)
w.setMinimumHeight(500)
main_layout.addWidget(w, stretch=5)

info_container = QtWidgets.QWidget()
info_layout = QtWidgets.QHBoxLayout(info_container)
info_layout.setContentsMargins(5, 5, 5, 5)
info_layout.setSpacing(0)

info_panel = QtWidgets.QGroupBox()
info_panel.setTitle("IMU+UWB DATA")
info_panel.setAlignment(QtCore.Qt.AlignCenter)
info_panel.setStyleSheet("""
    QGroupBox {
        background-color: #1a1a1a;
        border: 1px solid #444;
        border-radius: 6px;
        color: #00c8ff;
        font-weight: bold;
        font-size: 11pt;
        padding: 6px;
        text-align: center;
    }
    QLabel {
        color: white;
        font-size: 10pt;
    }
""")

line1 = QtWidgets.QLabel("Quat: 0.000,0.000,0.000,1.000 | Euler: R=0.00,P=0.00,Y=0.00 | Acc (lin mag)=0.000")
line2 = QtWidgets.QLabel("UWB raw=(0.000,0.000) filt=(0.000,0.000) fused=(0.000,0.000) | FusionFactor=0.02")

fixed_font = QtGui.QFont("Consolas")
fixed_font.setStyleHint(QtGui.QFont.Monospace)
fixed_font.setPointSize(10)
line1.setFont(fixed_font)
line2.setFont(fixed_font)
line1.setAlignment(QtCore.Qt.AlignCenter)
line1.setWordWrap(False)
line2.setAlignment(QtCore.Qt.AlignCenter)
line2.setWordWrap(False)
line1.setMinimumWidth(1000)
line2.setMinimumWidth(1000)

info_layout_inner = QtWidgets.QVBoxLayout(info_panel)
info_layout_inner.setAlignment(QtCore.Qt.AlignCenter)
info_layout_inner.addWidget(line1)
info_layout_inner.addWidget(line2)

info_layout.addWidget(info_panel)
main_layout.addWidget(info_container, stretch=1)

main_window.resize(1200, 800)
main_window.show()

# Grid axes
for gx, rot, trans in [
    (gl.GLGridItem(), (90, 0, 1, 0), (-1, 0, 0)),
    (gl.GLGridItem(), (90, 1, 0, 0), (0, -1, 0)),
    (gl.GLGridItem(), (0, 0, 0, 0), (0, 0, -1))
]:
    gx.rotate(*rot)
    gx.translate(*trans)
    w.addItem(gx)

# Cube for orientation
verts = np.array([
    [-0.5, -0.5, -0.5],
    [ 0.5, -0.5, -0.5],
    [ 0.5,  0.5, -0.5],
    [-0.5,  0.5, -0.5],
    [-0.5, -0.5,  0.5],
    [ 0.5, -0.5,  0.5],
    [ 0.5,  0.5,  0.5],
    [-0.5,  0.5,  0.5]
])
faces = np.array([
    [0,1,2],[0,2,3],
    [4,5,6],[4,6,7],
    [0,1,5],[0,5,4],
    [2,3,7],[2,7,6],
    [1,2,6],[1,6,5],
    [0,3,7],[0,7,4]
])
md = MeshData(vertexes=verts, faces=faces)
cube = gl.GLMeshItem(meshdata=md, smooth=False, color=(0.2,0.4,0.1,0.9), shader='balloon')
cube.scale(1.0, 0.3, 0.1)
w.addItem(cube)

# Axis arrows
arrow_x = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0.3,0,0]]), width=3)
arrow_y = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0.3,0]]), width=3)
arrow_z = gl.GLLinePlotItem(pos=np.array([[0,0,0],[0,0,0.3]]), width=3)
for arr in (arrow_x, arrow_y, arrow_z):
    w.addItem(arr)

# YZ plane (we'll map fused y->Y axis and fused x->Z for visualization if you like,
# but we'll plot fused Y vs Z=0 for simplicity). We'll plot fused Y vs fused X on this 2D view.
yz_win = pg.plot(title="Fused XY Plane (top-down)")
yz_win.setLabel('bottom', 'X (m)')
yz_win.setLabel('left', 'Y (m)')
yz_curve = yz_win.plot(pen=pg.mkPen('c', width=2))
yz_dot = yz_win.plot(symbol='o', symbolSize=8, symbolBrush='r')
yz_win.show()

# Reset button
def reset_position():
    global imu_processor, uwb_filter, yz_strokes, stroke_items
    imu_processor.reset()
    uwb_filter = JitterFilter()
    # clear strokes
    for item in stroke_items:
        try:
            yz_win.removeItem(item)
        except Exception:
            pass
    yz_strokes.clear()
    stroke_items.clear()

reset_btn = QtWidgets.QPushButton("Zero Position", parent=yz_win)
reset_btn.setStyleSheet("""
    QPushButton {
        background-color: #007acc;
        color: white;
        font-weight: bold;
        border-radius: 5px;
        padding: 6px 10px;
    }
    QPushButton:hover { background-color: #005f99; }
""")
reset_btn.clicked.connect(reset_position)
reset_btn.resize(120,30)
reset_btn.move(int(yz_win.width() - 130), int(yz_win.height() - 40))
reset_btn.show()
def reposition_button():
    reset_btn.move(int(yz_win.width() - 130), int(yz_win.height() - 40))
yz_win.resizeEvent = lambda e: (reposition_button(), pg.PlotWidget.resizeEvent(yz_win, e))

# -----------------------------
# Fusion objects and drawing globals
# -----------------------------
uwb_filter = JitterFilter()
imu_processor = IMUProcessor()

yz_strokes = []
current_stroke = []
is_drawing = False
stroke_items = []

# -----------------------------
# Time helper (handles 32-bit micros wrap)
# -----------------------------
def micros_dt(last, now):
    if last is None:
        return None
    diff = now - last
    if diff < 0:
        diff += 2**32
    return diff / 1e6

last_timestamp_us = None

# -----------------------------
# Update loop (reads serial, updates fusion + visualization)
# -----------------------------
frame_counter = 0

def update():
    global _serial_buf, last_timestamp_us, frame_counter
    global current_stroke, is_drawing, yz_strokes, stroke_items

    try:
        # Serial read
        if ser is not None:
            n = ser.in_waiting
            if n:
                data = ser.read(n).decode('utf-8', errors='ignore')
                _serial_buf += data

            if '\n' not in _serial_buf:
                return

            parts = _serial_buf.split('\n')
            _serial_buf = parts[-1]
            raw = parts[-2].strip()
        else:
            # no serial: nothing to do
            return

        if not raw:
            return

        parsed = parse_line(raw)
        if parsed is None:
            return

        raw_ux, raw_uy = parsed['raw_x'], parsed['raw_y']
        quat = parsed['quat']  # [qx,qy,qz,qw]
        accel_body = parsed['accel']  # [ax,ay,az]
        t_us = parsed['timestamp']

        # --- IMU prediction ---
        imu_x, imu_y = imu_processor.process(t_us, accel_body, quat)
        accel_mag = imu_processor.get_motion_state()

        # --- UWB adaptive filtering ---
        if accel_mag < MOTION_THRESHOLD:
            current_alpha = STATIONARY_ALPHA
        else:
            current_alpha = MOVING_ALPHA

        filt_x, filt_y = uwb_filter.filter(raw_ux, raw_uy, current_alpha)

        # --- Fuse: nudge IMU toward UWB ---
        fused_x, fused_y = imu_processor.fuse_with_uwb(filt_x, filt_y, FUSION_FACTOR)

        # Prepare a small 3D display position: map fused X->X, fused Y->Y, keep Z=0
        pos3 = np.array([fused_x, fused_y, 0.0], dtype=float)
        disp = pos3 * DISPLAY_SCALE

        # --- Orientation for cube ---
        try:
            r = R.from_quat(quat)
            euler = r.as_euler('xyz', degrees=True)
        except Exception:
            euler = np.array([0.0, 0.0, 0.0])

        # Update labels occasionally
        frame_counter += 1
        if frame_counter % 2 == 0:
            line1.setText(
                f"Quat: {quat[0]:.5f},{quat[1]:.5f},{quat[2]:.5f},{quat[3]:.5f} | "
                f"Euler: R={euler[0]:.2f},P={euler[1]:.2f},Y={euler[2]:.2f} | AccMag={accel_mag:.3f}"
            )
            line2.setText(
                f"UWB raw=({raw_ux:.3f},{raw_uy:.3f}) filt=({filt_x:.3f},{filt_y:.3f}) "
                f"fused=({fused_x:.3f},{fused_y:.3f}) | fusion={FUSION_FACTOR:.3f} alpha={current_alpha:.3f}"
            )

        # --- Update 3D cube and arrows ---
        for obj in (cube, arrow_x, arrow_y, arrow_z):
            obj.resetTransform()

        cube.scale(1.0, 0.3, 0.1)
        cube.rotate(euler[0], 1, 0, 0)
        cube.rotate(euler[1], 0, 1, 0)
        cube.rotate(euler[2], 0, 0, 1)
        cube.translate(disp[0], disp[1], disp[2])

        for arr in (arrow_x, arrow_y, arrow_z):
            arr.rotate(euler[0], 1, 0, 0)
            arr.rotate(euler[1], 0, 1, 0)
            arr.rotate(euler[2], 0, 0, 1)
            arr.translate(disp[0], disp[1], disp[2])

        # --- Drawing strokes in XY plane ---
        vel_x, vel_y = imu_processor.velocity[0], imu_processor.velocity[1]
        vel_mag = (vel_x**2 + vel_y**2)**0.5

        if vel_mag > DRAW_VEL_THRESH:
            if not is_drawing:
                current_stroke = []
                yz_strokes.append(current_stroke)
                new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
                yz_win.addItem(new_curve)
                stroke_items.append(new_curve)
                is_drawing = True

            current_stroke.append((fused_x, fused_y))
            stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
        else:
            is_drawing = False

        yz_dot.setData([fused_x], [fused_y])

        # Limit stroke memory
        if len(yz_strokes) > 200:
            yz_strokes.pop(0)
            old_item = stroke_items.pop(0)
            try:
                yz_win.removeItem(old_item)
            except Exception:
                pass

        # Debug print
        # print(f"Vel mag: {vel_mag:.5f}, fused=({fused_x:.3f},{fused_y:.3f})")

    except Exception as ex:
        print("Update error:", ex)

# -----------------------------
# Timer setup
# -----------------------------
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)

if __name__ == "__main__":
    pg.exec()
