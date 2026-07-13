"""Private ctypes declarations for the recognition wire format."""

import ctypes


class DateTime(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("year", ctypes.c_ushort), ("month", ctypes.c_ubyte),
        ("day", ctypes.c_ubyte), ("hour", ctypes.c_ubyte),
        ("minute", ctypes.c_ubyte), ("second", ctypes.c_ushort),
        ("offset", ctypes.c_short),
    ]


class Position3D(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("latitude", ctypes.c_int32), ("longitude", ctypes.c_int32)]


class PositionalAccuracy(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("semiMajor", ctypes.c_ubyte), ("semiMinor", ctypes.c_ubyte),
        ("orientation", ctypes.c_ushort),
    ]


class PositionOffset(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("offsetX", ctypes.c_short), ("offsetY", ctypes.c_short)]


class RecognitionFixed(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("msgSeq", ctypes.c_ubyte), ("equipmentType", ctypes.c_ubyte),
        ("sDSMTimeStamp", DateTime), ("refPos", Position3D),
        ("refPosXYConf", PositionalAccuracy),
        ("numDetectedObjects", ctypes.c_ubyte),
    ]


class DetectedObject(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("objType", ctypes.c_ubyte), ("objTypeCfd", ctypes.c_ubyte),
        ("objectID", ctypes.c_ushort), ("measurementTime", ctypes.c_short),
        ("timeConfidence", ctypes.c_ubyte), ("pos", PositionOffset),
        ("posConfidence", ctypes.c_ubyte), ("speed", ctypes.c_ushort),
        ("speedConfidence", ctypes.c_ubyte), ("heading", ctypes.c_ushort),
        ("headingConf", ctypes.c_ubyte),
    ]
