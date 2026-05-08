"""
imu_native_viz.py  —  PolyCast Native Window 3D IMU Visualizer
==============================================================
True 3D visualization inside a native OS window (no browser).
Requires: pip install pyvista
"""

import sys
import time
import numpy as np
import pyvista as pv

from data_parser import AsyncDataParser
from imu_integrator import quat_to_rotmat
from config import SERIAL_PORT, BAUD_RATE

# Set this to a path or leave blank for live stream
DATASET_FILENAME = ''

def main():
    dataset = sys.argv[1] if len(sys.argv) > 1 else DATASET_FILENAME
    parser = AsyncDataParser(port=SERIAL_PORT, baud=BAUD_RATE, csv_path=dataset)
    
    if not parser.connect():
        sys.exit(1)

    # ── Scene Setup ────────────────────────────────────────────────────────
    # Create the native window Plotter
    pl = pv.Plotter(title="PolyCast — True 3D Marker Visualization")
    pl.set_background('#1a1a1a')
    
    # World axes (X=Red, Y=Green, Z=Blue) in the bottom corner
    pl.add_axes(line_width=4, labels_off=False)

    # ── Marker Geometry (in Body Frame) ────────────────────────────────────
    # The IMU is the origin (0,0,0).
    # Body X maps to the board normal (away from board).
    # Tip is 13cm toward the board (-0.13), Rear is 8cm away (+0.08)
    
    # Center of cylinder = midpoint between tip and rear = (-0.13 + 0.08) / 2 = -0.025
    # direction=(1,0,0) lays the cylinder along the Body X axis.
    mesh_body = pv.Cylinder(center=(-0.025, 0.0, 0.0), direction=(1, 0, 0), radius=0.012, height=0.21)
    mesh_tip  = pv.Sphere(radius=0.015, center=(-0.13, 0.0, 0.0))
    mesh_rear = pv.Cube(center=(0.08, 0.0, 0.0), x_length=0.03, y_length=0.03, z_length=0.01)
    mesh_imu  = pv.Cube(center=(0.0, 0.0, 0.0),  x_length=0.02, y_length=0.02, z_length=0.005)

    # Local Body Axes (small arrows pointing along the body axes)
    mesh_arr_x = pv.Arrow(start=(0,0,0), direction=(1,0,0), scale=0.06)
    mesh_arr_y = pv.Arrow(start=(0,0,0), direction=(0,1,0), scale=0.06)
    mesh_arr_z = pv.Arrow(start=(0,0,0), direction=(0,0,1), scale=0.06)

    # Add meshes to the plotter (these return Actor objects we can directly transform)
    actor_body = pl.add_mesh(mesh_body, color='white', opacity=0.7)
    actor_tip  = pl.add_mesh(mesh_tip, color='red')
    actor_rear = pl.add_mesh(mesh_rear, color='cyan')
    actor_imu  = pl.add_mesh(mesh_imu, color='yellow')
    
    actor_arr_x = pl.add_mesh(mesh_arr_x, color='orange')
    actor_arr_y = pl.add_mesh(mesh_arr_y, color='magenta')
    actor_arr_z = pl.add_mesh(mesh_arr_z, color='cyan')

    # Group all actors we want to rotate together
    actors = [actor_body, actor_tip, actor_rear, actor_imu, actor_arr_x, actor_arr_y, actor_arr_z]

    print("\n🎮 Native Window Visualization started.")
    print("Controls: Left-click & drag to rotate, scroll to zoom.\n")

    # Open the window in interactive, non-blocking mode
    pl.show(interactive_update=True)

    try:
        while True:
            # Throttle CSV playback so it doesn't finish in half a second
            if parser.mode == 'csv':
                time.sleep(0.01)

            # Drain buffer to get the latest packet
            latest_imu = None
            if parser.mode == 'live':
                while parser.data_available():
                    pkt = parser.get_packet()
                    if pkt and pkt.get('type') == 'imu':
                        latest_imu = pkt
            else:
                pkt = parser.get_packet()
                if pkt == 'EOF':
                    print("\n📁 Reached end of data stream.")
                    break
                if pkt and pkt.get('type') == 'imu':
                    latest_imu = pkt

            if latest_imu:
                q = latest_imu['quat']
                force = latest_imu['force']
                
                # Get 3x3 rotation matrix
                R = quat_to_rotmat(*q)
                
                # PyVista transforms require a 4x4 homogeneous matrix
                T = np.eye(4)
                T[:3, :3] = R
                
                # Apply the rotation to all geometry pieces simultaneously
                for actor in actors:
                    actor.user_matrix = T

                # FSR Contact logic visualizer
                if force > 10.0:
                    actor_tip.prop.color = 'green'
                else:
                    actor_tip.prop.color = 'red'

            # Render the updated scene
            pl.update()
            
            # Break the loop if the user clicks the "X" on the window
            if pl.render_window is None:
                break

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user.")
    finally:
        parser.close()
        parser.summary()

if __name__ == '__main__':
    main()