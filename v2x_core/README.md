# v2x_core

Reusable, ROS-independent V2X protocol and transport library used by
[`v2x_intf_pkg`](../README.md), the ROS 2 bridge in this repository.

## Architecture

- `protocol/`: official SAE J3224/J2735 compilation and WAVE codecs
- `transport/`: protocol-independent, nonblocking UDP I/O

The protocol layer uses pycrate to compile the official SAE ASN.1 module
collection at application startup. No handwritten ASN.1 schema is used.

## Use from a normal Python application

`v2x_core` has no dependency on `rclpy` or `v2x_intf_msg` and can be used on
its own:

```python
import datetime

from v2x_core.protocol import (
    DetectedObjectData,
    RecognitionCodec,
    RecognitionData,
)
from v2x_core.protocol.v2xmsg import V2XMessage
from v2x_core.transport import UdpTransport

now = datetime.datetime.now()
message = RecognitionData(
    source_id=42,
    timestamp=now,
    position=(37.5, 127.0),
    objects=[DetectedObjectData(
        detection_time=now,
        position=(10.0, 2.0),
        velocity=12.5,
        heading=90.0,
        object_class=1,
        confidence=90,
    )],
)

asn1_directory = "/path/to/official/modules"
codec = RecognitionCodec(asn1_directory=asn1_directory)
frame = codec.encode(message)
datagram = V2XMessage(asn1_directory).pack_ifm_message(frame)

with UdpTransport("127.0.0.1", 1516, 0) as transport:
    transport.send(datagram)
```

Install the repository in a Python environment with:

```bash
python3 -m pip install .
```

## SAE J2735 ASN.1 files

The `asn1/` directory is reserved for the official SAE J2735 ASN.1
definitions used by pycrate. The licensed definitions are not included in
this repository.

### Setup

1. Obtain the official
   [SAE J2735ASN_202409 package](https://www.sae.org/standards/j2735asn_202409-v2x-communications-message-set-dictionary-asn-file).
2. Extract the downloaded archive.
3. Copy the extracted ASN.1 files into `asn1/`.

The resulting layout should resemble:

```text
v2x_core/
|-- asn1/
    |-- J2735ASN_202409/
        |-- *.asn files
```

Use the J2735 version expected by the protocol implementation. SAE files are
subject to SAE International's licensing terms; verify that those terms permit
your intended use and redistribution before committing them to the repository.

The codec finds the module collection in one of two ways, in this order:

1. The `asn1_directory` argument passed to `RecognitionCodec` or `V2XMessage`.
2. `V2X_ASN1_DIR` from `v2x_core/config.py`, resolved relative to this
   package's own directory.
