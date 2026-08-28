"""Configuration for the standalone V2X core package."""

from pathlib import Path

V2X_ASN1_DIR = "./asn1/J2735ASN_202409"


def default_asn1_dir() -> Path:
    """Resolve V2X_ASN1_DIR relative to this package's own directory."""
    return (Path(__file__).resolve().parent / V2X_ASN1_DIR).resolve()
