"""Tests for the J3224 SDSM UPER codec."""

from v2x_core.protocol.sdsm import (
    SDSM_MESSAGE_ID,
    decode_message_frame,
    encode_message_frame,
)


def test_sdsm_message_frame_round_trip():
    sdsm = {
        "msgCnt": 7,
        "sourceID": b"\x00\x00\x00\x2a",
        "equipmentType": "obu",
        "sDSMTimeStamp": {
            "year": 2026,
            "month": 8,
            "day": 4,
            "hour": 12,
            "minute": 30,
            "second": 12345,
            "offset": 540,
        },
        "refPos": {"lat": 375000000, "long": 1270000000},
        "refPosXYConf": {
            "semiMajor": 255,
            "semiMinor": 255,
            "orientation": 65535,
        },
        "objects": [{
            "detObjCommon": {
                "objType": "vehicle",
                "objTypeCfd": 90,
                "objectID": 0,
                "measurementTime": -10,
                "timeConfidence": "unavailable",
                "pos": {"offsetX": 100, "offsetY": -50},
                "posConfidence": {
                    "pos": "unavailable",
                    "elevation": "unavailable",
                },
                "speed": 500,
                "speedConfidence": "unavailable",
                "heading": 1000,
                "headingConf": "unavailable",
            }
        }],
    }

    message_id, decoded = decode_message_frame(encode_message_frame(sdsm))

    assert message_id == SDSM_MESSAGE_ID
    assert decoded == sdsm
