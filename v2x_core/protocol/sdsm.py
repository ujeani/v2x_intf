"""J3224 SDSM codec compiled from the official SAE ASN.1 modules."""

import copy
import hashlib
import importlib.util
import tempfile
import threading
from functools import lru_cache
from pathlib import Path

from pycrate_asn1c.asnproc import (
    GLOBAL,
    PycrateGenerator,
    compile_text,
    generate_modules,
)
from v2x_core.config import default_asn1_dir

SDSM_MESSAGE_ID = 41
_MODULE_DIRECTORY = "V2X ASN.1 Module Collection 2024"
_compile_lock = threading.Lock()
_runtime_lock = threading.RLock()


class SaeAsn1Error(RuntimeError):
    """Raised when the official SAE ASN.1 modules cannot be loaded."""


class SaeJ2735Codec:
    """Encode and decode official SAE J2735/J3224 UPER messages."""

    def __init__(self, asn1_directory=None):
        self.asn1_directory = resolve_asn1_directory(asn1_directory)
        self._module = _compile_official_modules(str(self.asn1_directory))
        self._message_frame = self._module.MessageFrame.MessageFrame

    def encode_sdsm(self, sdsm: dict) -> bytes:
        """Encode an SDSM inside its official J2735 MessageFrame open type."""
        value = {
            "messageId": SDSM_MESSAGE_ID,
            "value": (
                ("SensorDataSharingMessage", "SensorDataSharingMessage"),
                sdsm,
            ),
        }
        with _runtime_lock:
            try:
                self._message_frame.set_val(value)
                return self._message_frame.to_uper()
            except Exception as exc:
                raise SaeAsn1Error(f"failed to encode SDSM: {exc}") from exc

    def decode_frame(self, data: bytes):
        """Decode a J2735 MessageFrame using the official table constraint."""
        with _runtime_lock:
            try:
                self._message_frame.from_uper(data)
                frame = copy.deepcopy(self._message_frame.get_val())
            except Exception as exc:
                raise SaeAsn1Error(
                    f"failed to decode MessageFrame: {exc}"
                ) from exc
        return frame["messageId"], frame["value"]

    def decode_sdsm(self, data: bytes) -> dict:
        """Decode a MessageFrame and require a J3224 SDSM value."""
        message_id, open_value = self.decode_frame(data)
        if message_id != SDSM_MESSAGE_ID:
            raise ValueError(f"unsupported MessageFrame ID: {message_id}")
        type_name, value = open_value
        if type_name != "SensorDataSharingMessage":
            raise ValueError(f"message ID 41 resolved to unexpected type {type_name}")
        return value


def resolve_asn1_directory(directory=None) -> Path:
    """Find the directory containing the licensed official SAE modules."""
    if not directory:
        directory = default_asn1_dir()
    configured = Path(directory).expanduser()
    candidates = [configured, configured / _MODULE_DIRECTORY]

    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_dir() and any(resolved.glob("*.asn")):
            return resolved
    raise SaeAsn1Error(
        "Official SAE ASN.1 modules were not found. Pass asn1_directory or "
        "place them under the package's default asn1 directory."
    )


@lru_cache(maxsize=4)
def _compile_official_modules(directory: str):
    """Compile the unmodified SAE module collection into pycrate runtime types."""
    paths = sorted(Path(directory).glob("*.asn"))
    if not paths:
        raise SaeAsn1Error(f"no ASN.1 files found in {directory}")
    texts = [_read_sae_source(path) for path in paths]
    digest = hashlib.sha256()
    for path, text in zip(paths, texts):
        digest.update(path.name.encode())
        digest.update(text.encode("utf-8"))
    module_name = f"_v2x_sae_j2735_{digest.hexdigest()[:16]}"

    with _compile_lock:
        GLOBAL.clear()
        try:
            compile_text(texts, filenames=[str(path) for path in paths])
            with tempfile.TemporaryDirectory(prefix="v2x_asn1_") as temp_dir:
                generated = Path(temp_dir) / f"{module_name}.py"
                generate_modules(PycrateGenerator, str(generated))
                module = _import_generated_module(module_name, generated)
        except Exception as exc:
            raise SaeAsn1Error(
                f"failed to compile official SAE ASN.1 modules: {exc}"
            ) from exc
        finally:
            GLOBAL.clear()
    return module


def _read_sae_source(path: Path) -> str:
    data = path.read_bytes()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("windows-1252")


def _import_generated_module(module_name: str, path: Path):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise SaeAsn1Error(f"could not load generated module {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def encode_message_frame(sdsm: dict, asn1_directory=None) -> bytes:
    """Encode an SDSM with the official SAE module collection."""
    return SaeJ2735Codec(asn1_directory).encode_sdsm(sdsm)


def decode_frame(data: bytes, asn1_directory=None):
    """Decode an official J2735 MessageFrame and its open value."""
    return SaeJ2735Codec(asn1_directory).decode_frame(data)


def decode_message_frame(data: bytes, asn1_directory=None):
    """Decode an official MessageFrame containing a J3224 SDSM."""
    codec = SaeJ2735Codec(asn1_directory)
    return SDSM_MESSAGE_ID, codec.decode_sdsm(data)
