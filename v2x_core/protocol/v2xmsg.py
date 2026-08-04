"""J2735 WAVE message identification and IFM envelope formatting."""

from typing import Any, Callable
from v2x_core.protocol.saej2735 import SaeJ2735Codec

WAVE_MSG_IDS = [
    {
        "name": "BSM",
        "psid": "0020",
        "dsrc_msg_id": "20",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "MAP",
        "psid": "0082",
        "dsrc_msg_id": "18",
        "channel": "183",
        "priority": "3",
    },
    {
        "name": "SPAT",
        "psid": "0082",
        "dsrc_msg_id": "19",
        "channel": "183",
        "priority": "3",
    },
    {
        "name": "TIM",
        "psid": "0083",
        "dsrc_msg_id": "31",
        "channel": "183",
        "priority": "3",
    },
    {
        "name": "PSM",
        "psid": "0027",
        "dsrc_msg_id": "32",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "SensorDataSharingMessage",
        "psid": "8010",
        "dsrc_msg_id": "41",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "MobilityRequest",
        "psid": "BFEE",
        "dsrc_msg_id": "240",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "MobilityResponse",
        "psid": "BFEE",
        "dsrc_msg_id": "241",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "MobilityOperation",
        "psid": "BFEE",
        "dsrc_msg_id": "243",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "MobilityPath",
        "psid": "BFEE",
        "dsrc_msg_id": "242",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "TrafficControlRequest",
        "psid": "8003",
        "dsrc_msg_id": "244",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "TrafficControlMessage",
        "psid": "8003",
        "dsrc_msg_id": "245",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "EmergencyVehicleResponse",
        "psid": "8005",
        "dsrc_msg_id": "246",
        "channel": "183",
        "priority": "6",
    },
    {
        "name": "EmergencyVehicleAck",
        "psid": "8005",
        "dsrc_msg_id": "247",
        "channel": "183",
        "priority": "6",
    },
]


class V2XMessage:
    """Inspect official UPER J2735 MessageFrames."""

    def __init__(self, check_validity: bool = True):
        self.check_validity = check_validity

        # WAVE Service Advertisement (WSA) frame size = 1 byte for J2735 payloads < 128 bytes. IEEE 1609.3 (2020) - 8.1.3.
        # Add 2 bytes for DSRCmsgID.
        self.short_frame_ = 3

        # WAVE Service Advertisement (WSA) frame size = 2 bytes for J2735 payloads > 127 bytes. IEEE 1609.3 (2020) - 8.1.3.
        # Add 2 bytes for DSRCmsgID.
        self.long_frame_ =4

    @staticmethod
    def get_msg_info(msg_id: int):
        for entry in WAVE_MSG_IDS:
            if int(entry["dsrc_msg_id"]) == msg_id:
                return entry
        return None

    @staticmethod
    def get_msg_name(msg_id: int):
        info = V2XMessage.get_msg_info(msg_id)
        return info["name"] if info else None


    @staticmethod
    def find_msg_info_by_name(name: str):
        for entry in WAVE_MSG_IDS:
            if entry["name"] == name:
                return entry
        return None

    @staticmethod
    def is_valid_msg_id(msg_id: int):
        for entry in WAVE_MSG_IDS:
            if int(entry["dsrc_msg_id"]) == msg_id:
                return entry
        return None


    @staticmethod
    def is_possible_psid(msg_id: str) -> bool:
        for entry in WAVE_MSG_IDS:
            psid_value = int(entry["psid"],16)
            psid_int = str(psid_value)
            if msg_id == psid_int :
                return True            
        return False

    @staticmethod
    def is_valid_msg_size(msg_vec: bytes, start_index: int, entry: bytes):
        if len(msg_vec) > 127 :
            tmp_start_index = start_index + V2XMessage.long_frame_
            long_vec = entry[tmp_start_index:]
            msg_size = (msg_vec[2] & 0x7F) << 8 | msg_vec[3]
            if msg_size == len(long_vec) :
                return True
            else :
                return False
        elif len(msg_vec) < 128 and len(msg_vec) > 3 :
            tmp_start_index = start_index + V2XMessage.short_frame_
            short_vec = entry[tmp_start_index:]
            msg_size = msg_vec[2]
            if msg_size == len(short_vec) :
                return True
            else :
                return False
        else :
            return False

    @staticmethod
    def is_valid_msg_assuming_bsm_psid(start_index, entry: bytes):
        if start_index < 0 or start_index >= len(entry)-1 :
            return False
    
        # Valid element id will exist, at max, 5 bytes after a PSID
        for i in range(start_index, min(start_index + 6, len(entry)-1)) :
            # Generate a 16-bit element id from two bytes, e.g. [03 128 ...] = 0x0380
            element_id = (int(entry[i]) << 8) | int(entry[i + 1])
            # Check if valid element id 896 (0x0380) exists after PSID and before DSRCmsgID
            if element_id == 896 :
                element_id_index = i
                # Valid DSRCmsgID will exist, at max, 5 bytes after the element id
                for j in range(element_id_index, min(element_id_index + 6, len(entry) - 1)):
                    # Generate a 16-bit message id from two bytes, e.g. [0x00, 0x14] = 0x0014
                    possible_msg_id = (int(entry[j]) << 8) | int(entry[j + 1])

                    # Check if BSM DSRCmsgID 20 (0x0014)
                    if possible_msg_id == 20:
                        return True
        return False

    def on_message(self, data: bytes, callback: Callable[[int, str, bytes], None] = None):
        if not callback:
            raise ValueError("callback must be provided")
            return 
        
        if not data or len(data) < 3:
            return

        valid_msg_done = False

        if self.check_validity :
            for i in range(len(data)-3) :
                msg_id = (data[i] << 8) | data[i+1]
                msg_info = self.is_valid_msg_id(msg_id)
                if msg_info is None:
                    continue
                if ((i + self.short_frame_) >= len(data)) :
                    break; # Break if not enough data remaining

                start_index = i
                msg_vec = data[start_index:]
                if len(msg_vec) > 16383 :
                    break; # Break if message length exceeds max allowed

                if self.is_valid_msg_size(msg_vec, start_index, data) == False:
                    continue
                
                if self.callback:
                    self.callback(msg_id, msg_info["name"], msg_vec)
                    valid_msg_done = True
                    break


        if not valid_msg_done :
            # msg_id = (data[0] << 8) | data[1]
            if self.callback:
                self.callback(None, None, data)


    def pack_ifm_message(self, data: bytes, message_type="Unknown") -> bytes:
        """Wrap an official UPER MessageFrame in an IFM text envelope."""
        msg_id, _ = self._asn1.decode_frame(data)
        msg_info = self.get_msg_info(msg_id)
        if msg_info:
            name = msg_info["name"]
            psid = msg_info["psid"]
            channel = msg_info["channel"]
            priority = msg_info["priority"]
        else:
            name, psid, channel, priority = message_type, str(msg_id), "CCH", "1"
        fields = [
            "Version=0.7", f"Type={name}", f"PSID={psid}",
            f"Priority={priority}", "TxMode=ALT", f"TxChannel={channel}",
            "TxInterval=0", "DeliveryStart=", "DeliveryStop=",
            "Signature=False", "Encryption=False", f"Payload={data.hex()}",
        ]
        return ("\n".join(fields) + "\n").encode()

