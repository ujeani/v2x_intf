"""Tests for WAVE IFM envelope generation."""

from v2x_core.protocol.sdsm import encode_message_frame
from v2x_core.protocol.wave import WaveMessage


def _sdsm():
    return {
        "msgCnt": 1,
        "sourceID": b"\x01\x02\x03\x04",
        "equipmentType": "obu",
        "sDSMTimeStamp": {"year": 2026, "second": 0},
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
                "objectID": 1,
                "measurementTime": 0,
                "timeConfidence": "unavailable",
                "pos": {"offsetX": 10, "offsetY": 20},
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


def test_pack_sdsm_ifm_envelope():
    message_frame = encode_message_frame(_sdsm())
    packed = WaveMessage().pack_ifm_message(message_frame)

    assert b"Type=SensorDataSharingMessage\n" in packed
    assert b"PSID=8010\n" in packed
    assert packed.endswith(f"Payload={message_frame.hex()}\n".encode())
