"""Common V2X packet header encoding and validation."""

import struct
from dataclasses import dataclass

from v2x_intf_pkg.config import HEADER_FLAG

HEADER = struct.Struct("=III")


class PacketError(ValueError):
    """Raised when a V2X packet is malformed."""


@dataclass(frozen=True)
class Packet:
    message_id: int
    payload: bytes


class PacketCodec:
    """Encode and decode the package's fixed 12-byte packet header."""

    @staticmethod
    def encode(message_id: int, payload: bytes) -> bytes:
        # The legacy wire format stores only the length in network byte order.
        header = HEADER.pack(HEADER_FLAG, message_id, socket_htonl(len(payload)))
        return header + payload

    @staticmethod
    def decode(data: bytes) -> Packet:
        if not isinstance(data, bytes):
            raise PacketError("packet must be bytes")
        if len(data) < HEADER.size:
            raise PacketError("packet is shorter than its header")
        header_flag, message_id, encoded_length = HEADER.unpack_from(data)
        if header_flag != HEADER_FLAG:
            raise PacketError(f"invalid header flag: {header_flag:#x}")
        payload = data[HEADER.size:]
        expected_length = socket_ntohl(encoded_length)
        if len(payload) != expected_length:
            raise PacketError(
                f"invalid payload length: {len(payload)} != {expected_length}"
            )
        return Packet(message_id=message_id, payload=payload)


def socket_htonl(value: int) -> int:
    """Keep byte-order conversion isolated and easy to test."""
    return struct.unpack("=I", struct.pack("!I", value))[0]


def socket_ntohl(value: int) -> int:
    return struct.unpack("!I", struct.pack("=I", value))[0]
