# xp_pen_api.py
import os
import ctypes
import time

# -------------------------------------------------------------------------
# CONFIG
# -------------------------------------------------------------------------
SDK_DIR = r"C:\Users\cruzs\Documents\Software_Installer\mor\SDK_V1.1.1.25\Libs\x64"
DLL_NAME = "libSign.dll"
dll_path = os.path.join(SDK_DIR, DLL_NAME)

# -------------------------------------------------------------------------
# SDK Structures
# -------------------------------------------------------------------------
class DATAPACKET(ctypes.Structure):
    _fields_ = [
        ("eventtype", ctypes.c_int),
        ("physical_key", ctypes.c_ushort),
        ("virtual_key", ctypes.c_ushort),
        ("keystatus", ctypes.c_int),
        ("penstatus", ctypes.c_int),
        ("x", ctypes.c_ushort),
        ("y", ctypes.c_ushort),
        ("pressure", ctypes.c_ushort),
        ("wheel_direction", ctypes.c_short),
        ("button", ctypes.c_ushort),
        ("tiltX", ctypes.c_byte),
        ("tiltY", ctypes.c_byte),
    ]

# -------------------------------------------------------------------------
# Load DLL and set prototypes
# -------------------------------------------------------------------------
sign_lib = ctypes.WinDLL(dll_path)

sign_lib.signInitialize.restype = ctypes.c_int
sign_lib.signOpenDevice.restype = ctypes.c_int
sign_lib.signCloseDevice.restype = ctypes.c_int
sign_lib.signClean.restype = ctypes.c_int

CALLBACK_TYPE = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.POINTER(DATAPACKET))
sign_lib.signRegisterDataCallBack.argtypes = [CALLBACK_TYPE]
sign_lib.signRegisterDataCallBack.restype = ctypes.c_long  # returns a handle
sign_lib.signUnregisterDataCallBack.argtypes = [ctypes.c_long]
sign_lib.signUnregisterDataCallBack.restype = None

# -------------------------------------------------------------------------
# XP-Pen API Wrapper
# -------------------------------------------------------------------------
class XPPenAPI:
    def __init__(self):
        self.cb_handle = None
        self._callback_ref = None

    def start(self, callback):
        """Start the SDK and register callback"""
        rc = sign_lib.signInitialize()
        if rc != 0:
            raise RuntimeError(f"signInitialize failed ({rc})")

        rc = sign_lib.signOpenDevice()
        if rc != 0:
            sign_lib.signClean()
            raise RuntimeError(f"signOpenDevice failed ({rc})")

        # Keep a reference to avoid GC
        self._callback_ref = CALLBACK_TYPE(callback)
        self.cb_handle = sign_lib.signRegisterDataCallBack(self._callback_ref)
        if self.cb_handle == 0:
            sign_lib.signCloseDevice()
            sign_lib.signClean()
            raise RuntimeError("signRegisterDataCallBack failed")

        print("✅ XP-Pen API listening...")

    def stop(self):
        """Stop and cleanup"""
        if self.cb_handle:
            sign_lib.signUnregisterDataCallBack(self.cb_handle)
        sign_lib.signCloseDevice()
        sign_lib.signClean()
        print("🛑 XP-Pen API stopped.")

# -------------------------------------------------------------------------
# Example: run standalone
# -------------------------------------------------------------------------
if __name__ == "__main__":
    api = XPPenAPI()

    @CALLBACK_TYPE
    def data_callback(pkt_ptr):
        pkt = pkt_ptr.contents
        x = pkt.x
        y = pkt.pressure      # remap: Y is in pressure field
        pressure = pkt.button # remap: Pressure is in button field
        print(f"X={x}, Y={y}, Pressure={pressure}")
        return 0

    try:
        api.start(data_callback)
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        api.stop()
