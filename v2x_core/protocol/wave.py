"""J2735 WAVE message identification and IFM envelope formatting."""

from v2x_core.protocol.sdsm import SaeJ2735Codec

WAVE_MESSAGES = {
    18: ("MapData", "0082", "183", "3"),
    19: ("SPAT", "0082", "183", "3"),
    20: ("BasicSafetyMessage", "0020", "183", "6"),
    31: ("TravelerInformation", "0083", "183", "3"),
    32: ("PersonalSafetyMessage", "0027", "183", "6"),
    41: ("SensorDataSharingMessage", "8010", "183", "6"),
}


class WaveMessage:
    """Inspect official UPER J2735 MessageFrames."""

    def __init__(self, asn1_directory=None):
        self._asn1 = SaeJ2735Codec(asn1_directory)

    @staticmethod
    def get_msg_name(msg_id: int):
        info = WAVE_MESSAGES.get(msg_id)
        return info[0] if info else None

    def on_message(self, data: bytes):
        """Return message metadata and its table-resolved ASN.1 value."""
        if not data:
            return None, None, None, None
        try:
            msg_id, open_value = self._asn1.decode_frame(data)
            type_name, value = open_value
            msg_name = self.get_msg_name(msg_id) or type_name
            return msg_id, msg_name, value, None
        except Exception as exc:
            return None, None, None, f"Invalid J2735 MessageFrame: {exc}"

    def pack_ifm_message(self, data: bytes, message_type="Unknown") -> bytes:
        """Wrap an official UPER MessageFrame in an IFM text envelope."""
        msg_id, _ = self._asn1.decode_frame(data)
        name, psid, channel, priority = WAVE_MESSAGES.get(
            msg_id, (message_type, str(msg_id), "CCH", "1")
        )
        fields = [
            "Version=0.7", f"Type={name}", f"PSID={psid}",
            f"Priority={priority}", "TxMode=ALT", f"TxChannel={channel}",
            "TxInterval=0", "DeliveryStart=", "DeliveryStop=",
            "Signature=False", "Encryption=False", f"Payload={data.hex()}",
        ]
        return ("\n".join(fields) + "\n").encode()
