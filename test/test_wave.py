"""Tests for WAVE IFM envelope generation."""

import pytest

from v2x_intf_pkg.protocol.wave import pack_ifm_message


def test_pack_known_bsm():
    packed = pack_ifm_message(b"\x00\x14\x00")

    assert b"Type=BSM\n" in packed
    assert b"PSID=0020\n" in packed
    assert packed.endswith(b"Payload=001400\n")


def test_pack_rejects_short_payload():
    with pytest.raises(ValueError):
        pack_ifm_message(b"\x00\x14")
