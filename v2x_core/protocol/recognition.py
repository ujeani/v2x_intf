"""ROS-independent recognition data and J3224 SDSM conversion."""

import datetime
from dataclasses import dataclass
from typing import List, Tuple

from v2x_core.protocol.sdsm import SaeJ2735Codec

_EQUIPMENT_TYPES = ("unknown", "rsu", "obu", "vru")
_OBJECT_TYPES = ("unknown", "vehicle", "vru", "animal")


@dataclass
class DetectedObjectData:
    """Transport-neutral description of one detected object."""

    detection_time: datetime.datetime
    position: Tuple[float, float]
    velocity: float
    heading: float
    object_class: int
    confidence: int


@dataclass
class RecognitionData:
    """Transport-neutral recognition message used by the SDSM codec."""

    source_id: int
    timestamp: datetime.datetime
    position: Tuple[float, float]
    objects: List[DetectedObjectData]


class RecognitionCodec:
    """Translate ordinary Python recognition data to and from J3224 SDSM."""

    def __init__(
        self,
        equipment_type: int = 2,
        timezone_offset: int = 540,
        asn1_directory=None,
    ):
        if not 0 <= equipment_type < len(_EQUIPMENT_TYPES):
            raise ValueError("equipment_type must be between 0 and 3")
        if not -840 <= timezone_offset <= 840:
            raise ValueError("timezone_offset must be between -840 and 840")
        self._equipment_type = equipment_type
        self._timezone_offset = timezone_offset
        self._asn1 = SaeJ2735Codec(asn1_directory)
        self._send_sequence = 0

    def encode(self, message: RecognitionData) -> bytes:
        """Return a UPER J2735 MessageFrame containing a J3224 SDSM."""
        if not isinstance(message, RecognitionData):
            raise TypeError("message must be RecognitionData")
        if not 1 <= len(message.objects) <= 256:
            raise ValueError("J3224 SDSM requires between 1 and 256 objects")
        if not 0 <= int(message.source_id) <= 0xFFFFFFFF:
            raise ValueError("source_id must fit in the four-byte SDSM sourceID")

        latitude = round(float(message.position[0]) * 10_000_000)
        longitude = round(float(message.position[1]) * 10_000_000)
        if not -900_000_000 <= latitude <= 900_000_001:
            raise ValueError(f"latitude is out of range: {latitude}")
        if not -1_799_999_999 <= longitude <= 1_800_000_001:
            raise ValueError(f"longitude is out of range: {longitude}")

        sdsm = {
            "msgCnt": self._send_sequence,
            "sourceID": int(message.source_id).to_bytes(4, "big"),
            "equipmentType": _EQUIPMENT_TYPES[self._equipment_type],
            "sDSMTimeStamp": self._encode_timestamp(message.timestamp),
            "refPos": {"lat": latitude, "long": longitude},
            "refPosXYConf": {
                "semiMajor": 255,
                "semiMinor": 255,
                "orientation": 65535,
            },
            "objects": [
                self._encode_object(item, index, message.timestamp)
                for index, item in enumerate(message.objects)
            ],
        }
        self._send_sequence = (self._send_sequence + 1) % 128
        return self._asn1.encode_sdsm(sdsm)

    def _encode_timestamp(self, timestamp: datetime.datetime) -> dict:
        return {
            "year": timestamp.year,
            "month": timestamp.month,
            "day": timestamp.day,
            "hour": timestamp.hour,
            "minute": timestamp.minute,
            "second": timestamp.second * 1000 + timestamp.microsecond // 1000,
            "offset": self._timezone_offset,
        }

    def _encode_object(self, source, index, base_time):
        offset_ms = round(
            (source.detection_time - base_time).total_seconds() * 1000
        )
        object_class = self._clamp(int(source.object_class), 0, 3)
        speed = round(float(source.velocity) / 0.02)
        if not 0 <= speed <= 8190:
            speed = 8191
        heading = min(round((float(source.heading) % 360.0) / 0.0125), 28799)
        return {
            "detObjCommon": {
                "objType": _OBJECT_TYPES[object_class],
                "objTypeCfd": self._clamp(int(source.confidence), 0, 101),
                "objectID": index,
                "measurementTime": self._clamp(offset_ms, -1500, 1500),
                "timeConfidence": "unavailable",
                "pos": {
                    "offsetX": self._clamp(
                        round(float(source.position[0]) * 10), -32767, 32767
                    ),
                    "offsetY": self._clamp(
                        round(float(source.position[1]) * 10), -32767, 32767
                    ),
                },
                "posConfidence": {
                    "pos": "unavailable",
                    "elevation": "unavailable",
                },
                "speed": speed,
                "speedConfidence": "unavailable",
                "heading": heading,
                "headingConf": "unavailable",
            }
        }

    def decode(self, data) -> RecognitionData:
        """Convert a decoded SDSM dictionary or UPER MessageFrame to data."""
        if isinstance(data, bytes):
            sdsm = self._asn1.decode_sdsm(data)
        elif isinstance(data, dict):
            sdsm = data
        else:
            raise TypeError("SDSM must be encoded bytes or a decoded dictionary")

        stamp = sdsm["sDSMTimeStamp"]
        timestamp = datetime.datetime(
            stamp.get("year", 0),
            stamp.get("month", 0),
            stamp.get("day", 0),
            stamp.get("hour", 0),
            stamp.get("minute", 0),
            stamp.get("second", 0) // 1000,
            (stamp.get("second", 0) % 1000) * 1000,
        )
        objects = []
        for entry in sdsm["objects"]:
            item = entry["detObjCommon"]
            objects.append(DetectedObjectData(
                detection_time=timestamp + datetime.timedelta(
                    milliseconds=item["measurementTime"]
                ),
                position=(
                    item["pos"]["offsetX"] / 10.0,
                    item["pos"]["offsetY"] / 10.0,
                ),
                velocity=item["speed"] * 0.02,
                heading=item["heading"] * 0.0125,
                object_class=_OBJECT_TYPES.index(item["objType"]),
                confidence=item["objTypeCfd"],
            ))

        return RecognitionData(
            source_id=int.from_bytes(sdsm["sourceID"], "big"),
            timestamp=timestamp,
            position=(
                sdsm["refPos"]["lat"] / 10_000_000.0,
                sdsm["refPos"]["long"] / 10_000_000.0,
            ),
            objects=objects,
        )

    @staticmethod
    def _clamp(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))
