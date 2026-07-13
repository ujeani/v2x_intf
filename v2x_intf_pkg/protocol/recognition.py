"""Conversion between ROS Recognition messages and the binary V2X payload."""

import ctypes
import datetime

from v2x_intf_msg.msg import Object, Recognition

from v2x_intf_pkg.config import EQUIPMENT_TYPE, MSG_RECOGNITION
from v2x_intf_pkg.protocol.packet import PacketCodec, PacketError
from v2x_intf_pkg.protocol.structures import DetectedObject, RecognitionFixed


class RecognitionCodec:
    """Encode and decode Recognition messages without network or ROS node logic."""

    def __init__(self):
        self._send_sequence = 0
        self._expected_sequence = None

    def encode(self, message: Recognition) -> bytes:
        if not isinstance(message, Recognition):
            raise TypeError("message must be Recognition")
        if len(message.vehicle_time) != 7:
            raise ValueError("vehicle_time must contain seven values")
        if len(message.vehicle_position) != 2:
            raise ValueError("vehicle_position must contain latitude and longitude")

        vehicle_time = datetime.datetime(*message.vehicle_time)
        fixed = RecognitionFixed()
        fixed.msgSeq = self._send_sequence
        self._send_sequence = (self._send_sequence + 1) % 256
        fixed.equipmentType = EQUIPMENT_TYPE
        fixed.sDSMTimeStamp.year = vehicle_time.year
        fixed.sDSMTimeStamp.month = vehicle_time.month
        fixed.sDSMTimeStamp.day = vehicle_time.day
        fixed.sDSMTimeStamp.hour = vehicle_time.hour
        fixed.sDSMTimeStamp.minute = vehicle_time.minute
        fixed.sDSMTimeStamp.second = (
            vehicle_time.second * 1000 + vehicle_time.microsecond // 1000
        )
        fixed.sDSMTimeStamp.offset = 9 * 60

        latitude = round(float(message.vehicle_position[0]) * 10_000_000)
        longitude = round(float(message.vehicle_position[1]) * 10_000_000)
        if not -900_000_000 <= latitude <= 900_000_000:
            raise ValueError(f"latitude is out of range: {latitude}")
        if not -1_800_000_000 <= longitude <= 1_800_000_000:
            raise ValueError(f"longitude is out of range: {longitude}")
        fixed.refPos.latitude = latitude
        fixed.refPos.longitude = longitude
        fixed.refPosXYConf.semiMajor = 255
        fixed.refPosXYConf.semiMinor = 255
        fixed.refPosXYConf.orientation = 65535

        objects = list(message.object_data[:255])
        fixed.numDetectedObjects = len(objects)
        encoded_objects = bytearray()
        for index, source in enumerate(objects):
            detected = DetectedObject()
            detection_time = datetime.datetime(*source.detection_time)
            offset_ms = round((detection_time - vehicle_time).total_seconds() * 1000)
            detected.measurementTime = max(-1500, min(1500, offset_ms))
            detected.objType = source.object_class
            detected.objTypeCfd = source.recognition_accuracy
            detected.objectID = ((message.vehicle_id & 0xFF) << 8) | index
            detected.pos.offsetX = self._clamp(
                round(float(source.object_position[0]) * 10), -32767, 32767
            )
            detected.pos.offsetY = self._clamp(
                round(float(source.object_position[1]) * 10), -32767, 32767
            )
            speed = round(float(source.object_velocity) / 0.02)
            detected.speed = speed if 0 <= speed <= 8191 else 8192
            heading = float(source.object_heading) % 360.0
            detected.heading = min(round(heading / 0.0125), 28800)
            encoded_objects.extend(bytes(detected))

        payload = bytes(fixed) + bytes(encoded_objects)
        return PacketCodec.encode(MSG_RECOGNITION, payload)

    def decode(self, data: bytes) -> Recognition:
        packet = PacketCodec.decode(data)
        if packet.message_id != MSG_RECOGNITION:
            raise PacketError(f"unsupported message ID: {packet.message_id:#x}")
        fixed_size = ctypes.sizeof(RecognitionFixed)
        object_size = ctypes.sizeof(DetectedObject)
        if len(packet.payload) < fixed_size:
            raise PacketError("recognition payload is shorter than its fixed fields")
        fixed = RecognitionFixed.from_buffer_copy(packet.payload[:fixed_size])
        expected_size = fixed_size + fixed.numDetectedObjects * object_size
        if len(packet.payload) != expected_size:
            raise PacketError(
                f"invalid recognition length: {len(packet.payload)} != {expected_size}"
            )

        vehicle_time = [
            fixed.sDSMTimeStamp.year, fixed.sDSMTimeStamp.month,
            fixed.sDSMTimeStamp.day, fixed.sDSMTimeStamp.hour,
            fixed.sDSMTimeStamp.minute, fixed.sDSMTimeStamp.second // 1000,
            (fixed.sDSMTimeStamp.second % 1000) * 1000,
        ]
        base_time = datetime.datetime(*vehicle_time)
        decoded_objects = []
        vehicle_id = 0
        for index in range(fixed.numDetectedObjects):
            start = fixed_size + index * object_size
            item = DetectedObject.from_buffer_copy(
                packet.payload[start:start + object_size]
            )
            vehicle_id = item.objectID >> 8
            detected_at = base_time + datetime.timedelta(milliseconds=item.measurementTime)
            decoded_objects.append(Object(
                detection_time=[
                    detected_at.year, detected_at.month, detected_at.day,
                    detected_at.hour, detected_at.minute, detected_at.second,
                    detected_at.microsecond,
                ],
                object_position=[item.pos.offsetX / 10.0, item.pos.offsetY / 10.0],
                object_velocity=item.speed * 0.02,
                object_heading=item.heading * 0.0125,
                object_class=item.objType,
                recognition_accuracy=item.objTypeCfd,
            ))

        self._expected_sequence = (fixed.msgSeq + 1) % 256
        return Recognition(
            vehicle_id=vehicle_id,
            vehicle_time=vehicle_time,
            vehicle_position=[
                fixed.refPos.latitude / 10_000_000.0,
                fixed.refPos.longitude / 10_000_000.0,
            ],
            object_data=decoded_objects,
        )

    @staticmethod
    def _clamp(value: int, minimum: int, maximum: int) -> int:
        return max(minimum, min(maximum, value))
