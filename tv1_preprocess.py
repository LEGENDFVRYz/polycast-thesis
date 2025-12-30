import time
import os
import numpy as np
from collections import deque
from pyquaternion import Quaternion
from scipy.optimize import least_squares
from tv1_config import Config

# ==============================================================================
# MODULE A: IMU CLEANER (Phase 1)
# Responsibility: Physics, Rotation, & Drift Killing
# ==============================================================================
class IMUCleaner:
    def __init__(self):
        # Public State for Debugging
        self.state = {
            'is_static': False,
            'raw_mag': 0.0,
            'clean_mag': 0.0,
            'last_clean_acc': np.zeros(3)
        }

    def process(self, raw_sample):
        """
        Input: Raw Dictionary from SerialStreamer
        Output: Clean World Acceleration Vector (np.array) [x, y, z]
        """
        # 1. ROTATION (Standard Mode)
        q = Quaternion(
            raw_sample['quat'][3], 
            raw_sample['quat'][0], 
            raw_sample['quat'][1], 
            raw_sample['quat'][2]
        )
        
        acc_raw_vec = np.array(raw_sample['acc'])
        acc_world = q.rotate(acc_raw_vec)
        
        # 2. ZUPT (Zero-Velocity Update)
        mag = np.linalg.norm(acc_world)
        self.state['raw_mag'] = mag
        
        if mag < Config.IMU_ZUPT_THRESH:
            clean_acc = np.array([0.0, 0.0, 0.0])
            self.state['is_static'] = True
        else:
            clean_acc = acc_world
            self.state['is_static'] = False
            
        self.state['clean_mag'] = np.linalg.norm(clean_acc)
        self.state['last_clean_acc'] = clean_acc
        
        return clean_acc


# ==============================================================================
# MODULE B: TIME SYNCER (Phase 3)
# Responsibility: Buffering & Interpolation
# ==============================================================================
class TimeSyncer:
    def __init__(self):
        self.buffer = deque(maxlen=Config.SYNC_BUFFER_SIZE)
        self.state = {'last_method': 'NONE', 'lag': 0.0}
        
    def push(self, timestamp, clean_acc):
        self.buffer.append({'ts': timestamp, 'acc': clean_acc})
        
    def get_acceleration_at(self, query_time):
        """
        Finds acceleration at 'query_time' via interpolation.
        Returns: np.array([x,y,z])
        """
        frame_after = None
        frame_before = None
        
        for i in range(len(self.buffer)-1, -1, -1):
            curr = self.buffer[i]
            if curr['ts'] >= query_time:
                frame_after = curr
            else:
                frame_before = curr
                break 
        
        # CASE 1: Interpolate
        if frame_after and frame_before:
            dt = frame_after['ts'] - frame_before['ts']
            if dt > 0.2: 
                self.state['last_method'] = 'GAP_DROP'
                return frame_before['acc']
            
            ratio = (query_time - frame_before['ts']) / dt
            acc_interp = (1 - ratio) * frame_before['acc'] + ratio * frame_after['acc']
            
            self.state['last_method'] = 'INTERP'
            self.state['lag'] = 0.0
            return acc_interp
            
        # CASE 2: Hold (Latest data is slightly old)
        elif frame_before:
            lag = query_time - frame_before['ts']
            self.state['lag'] = lag
            if lag < 0.1: 
                self.state['last_method'] = 'HOLD'
                return frame_before['acc']
                
        # CASE 3: No Data
        self.state['last_method'] = 'EMPTY'
        return np.array([0.0, 0.0, 0.0])


# ==============================================================================
# MODULE C: UWB CLEANER (Phase 2)
# Responsibility: Geometry, Trilateration & Velocity Gating
# ==============================================================================
class UWBCleaner:
    def __init__(self):
        self.last_valid_pos = None
        self.last_valid_time = 0
        
        self.state = {
            'status': 'WAITING',
            'raw_pos': np.zeros(2),
            'final_pos': np.zeros(2),
            'speed': 0.0,
            'reject_reason': ''
        }

    def _solve_geometry(self, dists):
        def residuals(guess, anchors, measured_dists):
            return np.linalg.norm(anchors - guess, axis=1) - measured_dists

        x0 = np.mean(Config.ANCHORS, axis=0)
        res = least_squares(
            residuals, 
            x0, 
            bounds=(Config.GEO_BOUNDS_MIN, Config.GEO_BOUNDS_MAX), 
            args=(Config.ANCHORS, dists), 
            loss='soft_l1'
        )
        return res.x

    def process(self, raw_dists, timestamp, syncer_ref):
        """
        Input: Raw Distances, Timestamp, TimeSyncer Ref
        Output: Valid Position Vector, Status String
        """
        # 1. GEOMETRY
        dists_vec = np.array(raw_dists[:3])
        raw_pos = self._solve_geometry(dists_vec)
        self.state['raw_pos'] = raw_pos[:2]
        
        status = "VALID"
        reason = ""
        speed = 0.0
        final_pos = raw_pos[:2]
        
        # 2. PHYSICS GATE
        if self.last_valid_pos is not None:
            dt = timestamp - self.last_valid_time
            
            # A. Speed Check
            dist_moved = np.linalg.norm(raw_pos[:2] - self.last_valid_pos[:2])
            speed = dist_moved / dt if dt > 0 else 0
            
            # B. IMU Truth Check
            imu_acc = syncer_ref.get_acceleration_at(timestamp)
            is_moving = np.linalg.norm(imu_acc) > 0.0
            
            # C. Rules
            if speed > Config.MAX_HUMAN_SPEED:
                status = "REJECTED"
                reason = "SPEED_LIMIT"
                final_pos = self.last_valid_pos 
                
            elif (not is_moving) and (speed > Config.GHOST_SPEED_LIMIT):
                status = "REJECTED"
                reason = "GHOST_STATIC"
                final_pos = self.last_valid_pos 
            else:
                final_pos = raw_pos[:2]
        
        # 3. UPDATE STATE
        if status == "VALID":
            self.last_valid_pos = final_pos
            self.last_valid_time = timestamp
        
        self.state['status'] = status
        self.state['reject_reason'] = reason
        self.state['speed'] = speed
        self.state['final_pos'] = final_pos
            
        return final_pos, status


# ==============================================================================
# LIVE DEBUGGER (With Clock Synchronization)
# ==============================================================================
if __name__ == "__main__":
    from tv1_unpacker import SerialStreamer
    
    # --- HELPER: Clock Synchronizer ---
    # Converts Hardware Microseconds -> System Seconds
    class ClockAligner:
        def __init__(self):
            self.offset = None
        
        def get_synced_time(self, hw_micros):
            hw_sec = hw_micros / 1_000_000.0
            if self.offset is None:
                # Calculate difference between PC time and Sensor time once
                self.offset = time.time() - hw_sec
            return hw_sec + self.offset

    # SETUP
    streamer = SerialStreamer(port='COM3', baud=115200)
    imu_cleaner = IMUCleaner()
    time_syncer = TimeSyncer()
    uwb_cleaner = UWBCleaner()
    clock = ClockAligner()
    
    # Display Configurations
    DISPLAY_RATE = 0.1
    last_draw_time = 0
    
    def clear_screen():
        os.system('cls' if os.name == 'nt' else 'clear')

    print("[DEBUG] PIPELINE LIVE MONITOR")
    print("Connecting to hardware...")
    time.sleep(1)
    
    try:
        while True:
            # 1. READ SERIAL
            packets = streamer.read_new_packets()
            
            # 2. PROCESS PACKETS
            for pkt in packets:
                
                # --- PIPELINE STEP A: IMU CLEAN ---
                if pkt['type'] == 'IMU':
                    # Extract hardware timestamp (from the specific sample)
                    raw_sample = pkt['samples'][-1]
                    sensor_time = clock.get_synced_time(raw_sample['ts'])
                    
                    clean_acc = imu_cleaner.process(raw_sample)
                    
                    # Store with PRECISE sensor time
                    time_syncer.push(sensor_time, clean_acc)
                    
                # --- PIPELINE STEP B: UWB CLEAN ---
                elif pkt['type'] == 'UWB':
                    if 'dists' in pkt:
                        # Extract hardware timestamp (UWB usually has its own TS)
                        # Fallback to current time only if packet lacks TS
                        if 'ts' in pkt:
                            sensor_time = clock.get_synced_time(pkt['ts'])
                        else:
                            sensor_time = time.time() 
                            
                        uwb_cleaner.process(pkt['dists'], sensor_time, time_syncer)
            
            # 3. VISUALIZATION
            if (time.time() - last_draw_time > DISPLAY_RATE):
                clear_screen()
                print(f"=========== PIPELINE MONITOR ({DISPLAY_RATE}s) ==========")
                
                # --- IMU SECTION ---
                print(f"[IMU CLEANER]")
                s = imu_cleaner.state
                status = "STATIONARY (ZUPT)" if s['is_static'] else "MOVING"
                print(f"  State:     {status}")
                print(f"  Raw Mag:   {s['raw_mag']:.3f} m/s^2")
                print(f"  Clean Acc: X={s['last_clean_acc'][0]:.2f} Y={s['last_clean_acc'][1]:.2f} Z={s['last_clean_acc'][2]:.2f}")
                
                print("-" * 50)
                
                # --- UWB SECTION ---
                print(f"[UWB CLEANER]")
                u = uwb_cleaner.state
                
                print(f"  Input Pos: X={u['raw_pos'][0]:.2f} Y={u['raw_pos'][1]:.2f}")
                print(f"  Speed:     {u['speed']:.2f} m/s")
                
                if u['status'] == 'VALID':
                    print(f"  Status:    [ VALID ]")
                else:
                    print(f"  Status:    [ {u['status']} ] -> {u['reject_reason']}")
                
                print(f"  FINAL POS: X={u['final_pos'][0]:.3f} Y={u['final_pos'][1]:.3f}")

                print("-" * 50)
                
                # --- SYNC SECTION ---
                syn = time_syncer.state
                print(f"[SYNC ENGINE]")
                print(f"  Method:    {syn['last_method']}")
                print(f"  Lag:       {syn['lag']:.3f}s")
                
                print("=====================================================")
                last_draw_time = time.time()
                
    except KeyboardInterrupt:
        print("\nStopping...")
        streamer.close()