"""Tests for ROS-independent recognition conversion."""

import datetime

from v2x_core.protocol import (
    DetectedObjectData,
    RecognitionCodec,
    RecognitionData,
)


def test_recognition_data_sdsm_round_trip():
    timestamp = datetime.datetime(2026, 8, 4, 12, 30, 10, 123000)
    message = RecognitionData(
        source_id=42,
        timestamp=timestamp,
        position=(37.5, 127.0),
        objects=[DetectedObjectData(
            detection_time=timestamp + datetime.timedelta(milliseconds=100),
            position=(1.2, -3.4),
            velocity=10.0,
            heading=90.0,
            object_class=1,
            confidence=90,
        )],
    )
    codec = RecognitionCodec()

    decoded = codec.decode(codec.encode(message))

    assert decoded == message
