"""Adapters between ROS messages and the ROS-independent core model."""

import datetime

from v2x_intf_msg.msg import Object, Recognition

from v2x_core.protocol import DetectedObjectData, RecognitionData


def recognition_from_ros(message: Recognition) -> RecognitionData:
    """Convert a ROS Recognition message to the core dataclass."""
    if not isinstance(message, Recognition):
        raise TypeError("message must be Recognition")
    if len(message.vehicle_time) != 7:
        raise ValueError("vehicle_time must contain seven values")
    if len(message.vehicle_position) != 2:
        raise ValueError("vehicle_position must contain latitude and longitude")

    objects = []
    for item in message.object_data:
        if len(item.detection_time) != 7:
            raise ValueError("object detection_time must contain seven values")
        if len(item.object_position) != 2:
            raise ValueError("object_position must contain two values")
        objects.append(DetectedObjectData(
            detection_time=datetime.datetime(*item.detection_time),
            position=(item.object_position[0], item.object_position[1]),
            velocity=item.object_velocity,
            heading=item.object_heading,
            object_class=item.object_class,
            confidence=item.recognition_accuracy,
        ))

    return RecognitionData(
        source_id=message.vehicle_id,
        timestamp=datetime.datetime(*message.vehicle_time),
        position=(message.vehicle_position[0], message.vehicle_position[1]),
        objects=objects,
    )


def recognition_to_ros(message: RecognitionData) -> Recognition:
    """Convert the core dataclass to a ROS Recognition message."""
    objects = [
        Object(
            detection_time=_datetime_values(item.detection_time),
            object_position=list(item.position),
            object_velocity=item.velocity,
            object_heading=item.heading,
            object_class=item.object_class,
            recognition_accuracy=item.confidence,
        )
        for item in message.objects
    ]
    return Recognition(
        vehicle_id=message.source_id,
        vehicle_time=_datetime_values(message.timestamp),
        vehicle_position=list(message.position),
        object_data=objects,
    )


def _datetime_values(value: datetime.datetime):
    return [
        value.year,
        value.month,
        value.day,
        value.hour,
        value.minute,
        value.second,
        value.microsecond,
    ]
