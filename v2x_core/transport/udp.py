"""Protocol-independent UDP transport for Python applications."""

import socket
from typing import Optional, Tuple


class UdpTransport:
    """Own one nonblocking UDP socket used by the ROS bridge."""

    def __init__(self, remote_host: str, remote_port: int, local_port: int):
        self.remote_address: Tuple[str, int] = (remote_host, remote_port)
        self.local_port = local_port
        self._socket: Optional[socket.socket] = None

    @property
    def is_open(self) -> bool:
        return self._socket is not None

    @property
    def bound_port(self) -> Optional[int]:
        """Return the actual local port, including when port zero was requested."""
        return self._socket.getsockname()[1] if self._socket else None

    def open(self) -> None:
        if self.is_open:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", self.local_port))
            sock.setblocking(False)
        except OSError:
            sock.close()
            raise
        self._socket = sock

    def send(self, data: bytes) -> int:
        if not self._socket:
            raise RuntimeError("UDP transport is not open")
        return self._socket.sendto(data, self.remote_address)

    def receive(self, max_size: int = 65535) -> Optional[bytes]:
        if not self._socket:
            return None
        try:
            data, _ = self._socket.recvfrom(max_size)
            return data
        except BlockingIOError:
            return None

    def close(self) -> None:
        if self._socket:
            self._socket.close()
            self._socket = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()
