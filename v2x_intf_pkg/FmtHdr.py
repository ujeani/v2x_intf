import ctypes

# Define the v2x_intf_hdr_type structure
class v2x_intf_hdr_type(ctypes.Structure):
    _fields_ = [
        ("hdr_flag", ctypes.c_uint),  # unsigned int
        ("msgID", ctypes.c_uint),     # unsigned int
        ("msgLen", ctypes.c_uint)     # unsigned int
    ]
