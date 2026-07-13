"""Tests for the protocol-independent UDP transport."""

import socket

from v2x_intf_pkg.transport import UdpTransport


def test_udp_send_and_receive():
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(("127.0.0.1", 0))
    peer.settimeout(1.0)
    transport = UdpTransport("127.0.0.1", peer.getsockname()[1], 0)
    transport.open()
    try:
        assert transport.send(b"outgoing") == len(b"outgoing")
        assert peer.recvfrom(1024)[0] == b"outgoing"

        peer.sendto(b"incoming", ("127.0.0.1", transport.bound_port))
        for _ in range(1000):
            received = transport.receive()
            if received is not None:
                break
        assert received == b"incoming"
    finally:
        transport.close()
        peer.close()
