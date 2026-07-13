"""IFM 0.7 text-envelope formatting for WAVE messages."""

WAVE_MESSAGES = {
    18: ("MAP", "0082", "183", "3"),
    19: ("SPAT", "0082", "183", "3"),
    20: ("BSM", "0020", "183", "6"),
    31: ("TIM", "0083", "183", "3"),
    32: ("PSM", "0027", "183", "6"),
    41: ("SensorDataSharingMessage", "8010", "183", "6"),
    240: ("MobilityRequest", "BFEE", "183", "6"),
    241: ("MobilityResponse", "BFEE", "183", "6"),
    242: ("MobilityPath", "BFEE", "183", "6"),
    243: ("MobilityOperation", "BFEE", "183", "6"),
    244: ("TrafficControlRequest", "8003", "183", "6"),
    245: ("TrafficControlMessage", "8003", "183", "6"),
    246: ("EmergencyVehicleResponse", "8005", "183", "6"),
    247: ("EmergencyVehicleAck", "8005", "183", "6"),
}


def pack_ifm_message(data: bytes, message_type: str = "Unknown") -> bytes:
    """Wrap a DSRC payload in the text envelope expected by the IFM service."""
    if len(data) < 3:
        raise ValueError("DSRC payload must contain at least three bytes")
    message_id = int.from_bytes(data[:2], "big")
    name, psid, channel, priority = WAVE_MESSAGES.get(
        message_id, (message_type, str(message_id), "CCH", "1")
    )
    fields = [
        "Version=0.7", f"Type={name}", f"PSID={psid}",
        f"Priority={priority}", "TxMode=ALT", f"TxChannel={channel}",
        "TxInterval=0", "DeliveryStart=", "DeliveryStop=",
        "Signature=False", "Encryption=False", f"Payload={data.hex()}",
    ]
    return ("\n".join(fields) + "\n").encode()
