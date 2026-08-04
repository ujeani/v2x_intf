from unittest import result

import asn1tools
import os
from pathlib import Path
from v2x_intf_pkg.config import ASN_1_DIR


NAME_IDX = 0
PSID_IDX = 1
CHANNEL_IDX = 2
PRIORITY_IDX = 3

WAVE_MESSAGES = {
    18:  ("MapData", "0082", "183", "3"),
    19:  ("SPAT", "0082", "183", "3"),
    20:  ("BasicSafetyMessage", "0020", "183", "6"),
    31:  ("TravelerInformation", "0083", "183", "3"),
    32:  ("PersonalSafetyMessage", "0027", "183", "6"),
    41:  ("SensorDataSharingMessage", "8010", "183", "6"),
    240: ("MobilityRequest", "BFEE", "183", "6"),
    241: ("MobilityResponse", "BFEE", "183", "6"),
    242: ("MobilityPath", "BFEE", "183", "6"),
    243: ("MobilityOperation", "BFEE", "183", "6"),
    244: ("TrafficControlRequest", "8003", "183", "6"),
    245: ("TrafficControlMessage", "8003", "183", "6"),
    246: ("EmergencyVehicleResponse", "8005", "183", "6"),
    247: ("EmergencyVehicleAck", "8005", "183", "6"),
}


class WaveMessage:
    def __init__(self, check_validity: bool = True):

        self.check_validity = check_validity

        # WAVE Service Advertisement (WSA) frame size = 1 byte for J2735 payloads < 128 bytes. IEEE 1609.3 (2020) - 8.1.3.
        # Add 2 bytes for DSRCmsgID.
        self.short_frame_ = 3

        # WAVE Service Advertisement (WSA) frame size = 2 bytes for J2735 payloads > 127 bytes. IEEE 1609.3 (2020) - 8.1.3.
        # Add 2 bytes for DSRCmsgID.
        self.long_frame_ = 4

        package_dir = Path(__file__).resolve().parent.parent
        j2735_asn_dir = package_dir / ASN_1_DIR

        asn_files = sorted(j2735_asn_dir.glob("*.asn"))
        self.j2735_spec = asn1tools.compile_files(
            [str(path) for path in asn_files],
            codec="uper",
        )

    def get_msg_name(self, msg_id: int):
        """!
        @brief Retrieves message information for a given DSRC message ID.
        
        @param msg_id The DSRC message ID as an integer.
        @return Tuple containing (name, psid, channel, priority) if valid, otherwise None.
        """
        msg_info = WAVE_MESSAGES.get(msg_id)
        if msg_info is not None:
            
            return msg_info[NAME_IDX]
        return None
    
    def is_valid_msg_id(self, msg_id: int):
        """!
        @brief Checks if a given message ID exists in the known WAVE_MSG_IDS list.
        
        @param msg_id The DSRC message ID as an integer.
        @return Dictionary containing message info if valid, otherwise None.
        """
        return WAVE_MESSAGES.get(msg_id, None)

    def is_possible_psid(self, msg_id: str) -> bool:
        """!
        @brief Checks if a given Provider Service Identifier (PSID) is known.
        
        @param msg_id The string representation of the parsed message ID.
        @return True if the PSID matches an entry in WAVE_MSG_IDS, False otherwise.
        """
        for entry in WAVE_MESSAGES.values():
            psid_value = int(entry[PSID_IDX],16)
            psid_int = str(psid_value)
            if msg_id == psid_int :
                return True
            
        return False


    def is_valid_msg_assuming_bsm_psid(self, start_index, entry: bytes):
        """!
        @brief Validates the message structure assuming it starts with a BSM PSID.
        
        @param start_index The starting index in the byte array.
        @param entry The original raw byte array.
        @return True if valid BSM identifiers are found, False otherwise.
        """
        if start_index < 0 or start_index >= len(entry)-1 :
            # print(f"Error: invalid start index {start_index} for data length {len(entry)}")
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

    def parse(self, msg_id, msg_name, data: bytes) -> dict:
        if data is None or len(data) < 3 :
            return None, "Invalid message data."

        if msg_id is None or  msg_name is None :
            return None, "Invalid message ID or name."

        mf = self.j2735_spec.decode("MessageFrame", data    )
        try :
            msg = self.j2735_spec.decode(msg_name, mf.get('value', ''))
            return msg, None
        except Exception as e :
            return None, f"Failed to decode message {msg_name} with ID {msg_id}: {e}"


    def on_message(self, data: bytes) -> tuple[int, str, dict, str]: 
        """!
        @brief Process a received WAVE message.
        
        @param data The raw bytes of the received message.
        @return Tuple containing (msg_id, msg_name, parsed_wsm, error_message) if valid, otherwise None.
        """

        if not data or len(data) < 3 :
            return None, None, None, "Invalid message data."

        if self.check_validity:
            for i in range(len(data)-3) :
                msg_id = (data[i] << 8) | data[i+1]
                msg_info = self.is_valid_msg_id(msg_id)
                if msg_info is None:
                    return msg_id, None, None, None # message name None means unknown, but we still return the ID for further processing
                
                if ((i + self.short_frame_) >= len(data)) :
                    return None, None, None, "discarding received message with insufficient data for short frame header."

                start_index = i
                msg_vec = data[start_index:]
                if len(msg_vec) > 16383 :
                    return None, None, None, "discarding received message with length field longer than 16383."

                if self.is_valid_msg_size(msg_vec, start_index, data) == False:
                    return None, None, None, "discarding received message with invalid size field."
                
                is_wave_msg = (not self.is_possible_psid(str(msg_id))) or (not self.is_valid_msg_assuming_bsm_psid(start_index, data))
                if is_wave_msg :
                    msg_name = self.get_msg_name(msg_id)
                    return msg_id, msg_name, self.parse(msg_id, msg_name, data), None
                else :
                    return msg_id, None, None, None # message name None means unknown, but we still return the ID for further processing
        else :
            msg_id = (data[0] << 8) | data[1]
            return msg_id, None, None, None # message name None means unknown, but we still return the ID for further processing

    # def pack_ifm_message(self, data: bytes, message_type: str = "Unknown") -> bytes:
    #     """Wrap a DSRC payload in the text envelope expected by the IFM service."""
    #     if len(data) < 3:
    #         raise ValueError("DSRC payload must contain at least three bytes")
    #     message_id = int.from_bytes(data[:2], "big")
    #     name, psid, channel, priority = WAVE_MESSAGES.get(
    #         message_id, (message_type, str(message_id), "CCH", "1")
    #     )
    #     fields = [
    #         "Version=0.7", f"Type={name}", f"PSID={psid}",
    #         f"Priority={priority}", "TxMode=ALT", f"TxChannel={channel}",
    #         "TxInterval=0", "DeliveryStart=", "DeliveryStop=",
    #         "Signature=False", "Encryption=False", f"Payload={data.hex()}",
    #     ]
    #     return ("\n".join(fields) + "\n").encode()
