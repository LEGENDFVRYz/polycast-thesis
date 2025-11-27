import serial
import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import sys

# -----------------------------
# User configuration
# -----------------------------
SERIAL_PORT = 'COM5'
BAUD_RATE = 115200

# --- ANCHOR POSITIONS (Meters) ---
# MEASURE THESE! (Example: 1.75m width)
ANCHORS = np.array([
    [1.75, 0.00, 0.00],  # Anchor 0
    [1.75, 1.61, 0.00],  # Anchor 1
    [0.00, 0.00, 0.00]   # Anchor 2 (Origin)
])

# --- SENSITIVITY TUNING (Fixed for Overlap) ---
LP_ALPHA = 0.2
BIAS_ALPHA = 0.01
VELOCITY_DECAY = 0.98   # Increased from 0.92 to 0.96 (Allows gliding between letters)
STATIC_THRESH = 0.06    # Lowered from 0.2! Catches slow "hover" moves.
STATIC_TIME = 0.5
TRAP_DT_MAX = 0.1
DRAW_VEL_THRESH = 0.02  # Speed needed to produce "Ink"

IMU_SAMPLES_PER_PACKET = 10 

# --- FUSION SETTINGS ---
UWB_ALPHA_MOVING = 0.02
UWB_ALPHA_STATIC = 0.05 # Low trust when static to prevent "Snapping" back

# Calibration State
is_calibrating = True
calib_count = 0
calib_accum = np.zeros(3)
gravity_ref = np.array([0.0, 0.0, 9.81])

# -----------------------------
# Serial setup
# -----------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except serial.SerialException as e:
    print(f"Serial Error: {e}")
    sys.exit(1)
    
_serial_buf = ""

# -----------------------------
# State variables
# -----------------------------
pos = np.zeros(3)
vel = np.zeros(3)
accel_filtered = np.zeros(3)
accel_bias = np.zeros(3)
prev_accel = None
prev_vel = None
last_timestamp_us = None
static_since = None
imu_heading_offset = 0.0

uwb_pos = np.zeros(3)
uwb_center_offset = np.zeros(3)
fused_pos = np.zeros(3)

# Drawing state
yz_strokes = []      
current_stroke = []  
is_drawing = False
stroke_items = []    

# -----------------------------
# Helpers
# -----------------------------
def micros_dt(last, now):
    if last is None: return None
    diff = now - last
    if diff < 0: diff += 4294967296
    return diff / 1e6

def rotate_vec2d(x, y, theta):
    cs = np.cos(theta)
    sn = np.sin(theta)
    rx = x * cs - y * sn
    ry = x * sn + y * cs
    return rx, ry

def trilaterate(distances, anchors):
    def residuals(x, anchors, dists):
        return np.linalg.norm(anchors - x, axis=1) - dists
    x0 = np.mean(anchors, axis=0)
    res = least_squares(residuals, x0, args=(anchors, distances), 
                        bounds=([-10, -10, -1], [10, 10, 3]))
    return res.x

def apply_uwb_fusion(is_moving_state):
    global pos, fused_pos, uwb_pos, uwb_center_offset
    
    imu_only_pos = pos
    target_offset = uwb_pos - imu_only_pos
    
    if is_moving_state:
        alpha = UWB_ALPHA_MOVING
    else:
        alpha = UWB_ALPHA_STATIC
        
    uwb_center_offset = (1 - alpha) * uwb_center_offset + alpha * target_offset
    fused_pos[:] = imu_only_pos + uwb_center_offset
    return True

def calibrate_alignment():
    global imu_heading_offset, vel
    imu_angle = np.arctan2(vel[1], vel[0])
    target_angle = 0.0 
    new_offset = target_angle - imu_angle
    imu_heading_offset += new_offset
    print(f"--- Heading Calibrated: {np.degrees(imu_heading_offset):.2f}° ---")

# -----------------------------
# Qt App setup
# -----------------------------
app = QtWidgets.QApplication([])

yz_win = pg.plot(title="PolyCast - Sensitive Writing")
# Note: Using Y and Z as your axes based on your description
yz_win.setLabel('bottom', 'Y (m)')
yz_win.setLabel('left', 'Z (m)')
yz_win.setAspectLocked(True)
yz_win.showGrid(x=True, y=True)
yz_dot = yz_win.plot(symbol='o', symbolSize=8, symbolBrush='r')
uwb_ghost = yz_win.plot(symbol='+', symbolSize=10, symbolBrush='g') 
yz_win.show()

info_win = QtWidgets.QGroupBox("System State")
info_layout = QtWidgets.QVBoxLayout(info_win)
info_label = QtWidgets.QLabel("Status: Waiting...")
info_layout.addWidget(info_label)
info_win.show()

def reset_position():
    global pos, vel, yz_strokes, current_stroke, is_drawing, stroke_items, last_timestamp_us
    global uwb_pos, fused_pos, uwb_center_offset
    global is_calibrating, calib_count, calib_accum
    
    pos[:] = 0
    vel[:] = 0
    uwb_pos[:] = 0
    fused_pos[:] = 0
    uwb_center_offset[:] = 0 
    last_timestamp_us = None 
    
    is_calibrating = True
    calib_count = 0
    calib_accum[:] = 0
    
    yz_strokes.clear()
    current_stroke.clear()
    is_drawing = False
    for item in stroke_items:
        try: yz_win.removeItem(item)
        except: pass
    stroke_items.clear()
    
    yz_dot.setData([0], [0])
    print("--- Reset ---")

reset_btn = QtWidgets.QPushButton("Zero Position", parent=yz_win)
reset_btn.clicked.connect(reset_position)
reset_btn.show()

# =============================
# IMU Processing
# =============================
def process_imu_sample(qx, qy, qz, qw, ax_s, ay_s, az_s, t_us):
    global pos, vel, accel_filtered, accel_bias, prev_accel, prev_vel, last_timestamp_us, static_since
    global yz_strokes, current_stroke, is_drawing, stroke_items, fused_pos, imu_heading_offset
    global is_calibrating, calib_count, calib_accum, gravity_ref

    accel_local = np.array([ax_s, ay_s, az_s], dtype=float)
    dt = micros_dt(last_timestamp_us, t_us)
    last_timestamp_us = t_us

    if dt is None or dt <= 0 or dt > TRAP_DT_MAX: return False

    # 1. Rotate
    r = R.from_quat([qx, qy, qz, qw])
    accel_world = r.apply(accel_local)
    accel_world[1] *= -1 

    # 2. Align
    ax_aligned, ay_aligned = rotate_vec2d(accel_world[0], accel_world[1], imu_heading_offset)
    accel_world[0] = ax_aligned
    accel_world[1] = ay_aligned
    
    # Calibration
    if is_calibrating:
        calib_accum += accel_world
        calib_count += 1
        if calib_count >= 50:
            gravity_ref = calib_accum / 50.0
            is_calibrating = False
            print(f"Calibrated Gravity: {gravity_ref}")
        return True

    # 3. Subtract Gravity
    accel_world -= gravity_ref

    # --- REMOVED AGGRESSIVE DEADZONES HERE ---
    # We allow small movements on all axes now.
    # Only kept very tiny noise floor
    if abs(accel_world[1]) < 0.01: accel_world[1] = 0
    if abs(accel_world[2]) < 0.01: accel_world[2] = 0

    # 4. Low-pass filter
    accel_filtered[:] = LP_ALPHA * accel_world + (1 - LP_ALPHA) * accel_filtered
    mag = np.linalg.norm(accel_filtered)
    
    # 5. Static Detection
    if mag < STATIC_THRESH:
        if static_since is None: static_since = time.time()
        static_time = time.time() - static_since
    else:
        static_since = None
        static_time = 0.0
    is_static = (static_time >= STATIC_TIME)

    if is_static:
        # Slower bias update
        accel_bias[:] = (1 - BIAS_ALPHA) * accel_bias + BIAS_ALPHA * accel_filtered
        vel[:] = 0.0 
        is_drawing = False 

    accel_corrected = accel_filtered - accel_bias
    
    # 6. Integrate
    if prev_accel is None:
        prev_accel = accel_corrected.copy()
        prev_vel = vel.copy()
    
    vel += 0.5 * (prev_accel + accel_corrected) * dt
    vel *= VELOCITY_DECAY 
    pos += 0.5 * (prev_vel + vel) * dt

    prev_accel = accel_corrected.copy()
    prev_vel = vel.copy()
    
    # 7. Update Fused Pos
    fused_pos[:] = pos[:] + uwb_center_offset

    # --- Drawing Logic ---
    vel_mag = np.linalg.norm(vel)
    
    # Using Y and Z for drawing plane
    if vel_mag > DRAW_VEL_THRESH and not is_static:
        if not is_drawing:
            current_stroke = []
            yz_strokes.append(current_stroke)
            new_curve = pg.PlotDataItem(pen=pg.mkPen('c', width=2))
            yz_win.addItem(new_curve)
            stroke_items.append(new_curve)
            is_drawing = True

        current_stroke.append((fused_pos[0], fused_pos[2]))
        stroke_items[-1].setData([p[0] for p in current_stroke], [p[1] for p in current_stroke])
    else:
        is_drawing = False

    yz_dot.setData([fused_pos[0]], [fused_pos[2]]) 
    uwb_ghost.setData([uwb_pos[0]], [uwb_pos[1]])

    info_label.setText(
        f"Status: {'CALIBRATING...' if is_calibrating else 'READY'}\n"
        f"Pos Y: {fused_pos[1]:.2f} | Z: {fused_pos[2]:.2f}\n"
    )
    
    if len(yz_strokes) > 100:
        yz_strokes.pop(0)
        old_item = stroke_items.pop(0)
        yz_win.removeItem(old_item)
        
    return True

# =============================
# Update Loop
# =============================
def update():
    global _serial_buf, uwb_pos
    try:
        n = ser.in_waiting
        if n: _serial_buf += ser.read(n).decode('utf-8', errors='ignore')
        if '\n' not in _serial_buf: return

        parts = _serial_buf.split('\n')
        lines_to_process = parts[:-1]
        _serial_buf = parts[-1]
        
        for raw in lines_to_process:
            raw = raw.strip()
            if not raw: continue
            toks = raw.split(',')
            if len(toks) < 86: continue

            try:
                d0 = float(toks[2])
                d1 = float(toks[3])
                d2 = float(toks[4])
                
                if d0 > 0 and d1 > 0 and d2 > 0:
                    raw_uwb_3d = trilaterate([d0, d1, d2], ANCHORS)
                    uwb_pos[:] = raw_uwb_3d[:]
                    apply_uwb_fusion(is_moving_state=is_drawing)
                
                for i in range(IMU_SAMPLES_PER_PACKET):
                    s = 6 + i * 8
                    qx, qy, qz, qw = map(float, toks[s : s+4])
                    ax, ay, az = map(float, toks[s+4 : s+7])
                    t_us = int(toks[s+7])
                    process_imu_sample(qx, qy, qz, qw, ax, ay, az, t_us)
            except ValueError: continue
    except Exception as ex: print(ex)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10) 

if __name__ == "__main__":
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+R"), yz_win, reset_position)
    QtGui.QShortcut(QtGui.QKeySequence("Ctrl+C"), yz_win, calibrate_alignment)
    sys.exit(pg.exec())