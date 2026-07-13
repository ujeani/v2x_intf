"""Tests for the common V2X packet codec."""

import pytest

from v2x_intf_pkg.protocol.packet import PacketCodec, PacketError


def test_packet_round_trip():
    payload = b"recognition-payload"
    encoded = PacketCodec.encode(0x1234, payload)

    decoded = PacketCodec.decode(encoded)

    assert decoded.message_id == 0x1234
    assert decoded.payload == payload


def test_packet_rejects_invalid_header():
    encoded = bytearray(PacketCodec.encode(0x1234, b"payload"))
    encoded[0] ^= 0xFF

    with pytest.raises(PacketError, match="invalid header flag"):
        PacketCodec.decode(bytes(encoded))


def test_packet_rejects_invalid_length():
    encoded = PacketCodec.encode(0x1234, b"payload")

    with pytest.raises(PacketError, match="invalid payload length"):
        PacketCodec.decode(encoded[:-1])
