"""Parser for the SolidWorks 2015+ chunked container format.

Python port of the modern-format parser in openswx (MIT,
https://github.com/schwitters/openswx — see vendor/openswx/libopenswx/src/
internal/modern_parser.cc for the reference implementation and format notes).

Container layout:
  - ROL key: 1 byte at file offset 7 — rotate-left decodes all stream names.
  - Chunks are located by scanning for the 6-byte marker 14 00 06 00 08 00;
    a chunk starts 4 bytes before the marker (si = marker_pos - 4):

      si+0x00  val_a          u32   file-specific tag
      si+0x04  14 00 06 00 08 00    marker
      si+0x0a  section_type   u8    0xDF=TOC, 0xFD=data, 0x1C=mini
      si+0x0b  suffix         3B    file-specific
      si+0x0e  f1             u32   >= 65536 -> inline chunk with data
      si+0x12  csz            u32   compressed size
      si+0x16  usz            u32   uncompressed size
      si+0x1a  nsz            u32   stream-name length
      si+0x1e  name[nsz]            ROL-encoded UTF-8 stream name
      si+0x1e+nsz  data[csz]        raw-deflate payload (inline chunks only)

Pre-2015 OLE2 files are out of scope.
"""

from __future__ import annotations

import zlib
from dataclasses import dataclass, field

MARKER = b"\x14\x00\x06\x00\x08\x00"
CHUNK_HEADER_SIZE = 0x1E
INLINE_F1_THRESHOLD = 65536
MAX_NAME_SIZE = 512
MAX_COMPRESSED_SIZE = 64 * 1024 * 1024

OLE2_MAGIC = b"\xd0\xcf\x11\xe0"
ZIP_MAGIC = b"PK\x03\x04"


def rol_byte(b: int, shift: int) -> int:
    shift &= 7
    if shift == 0:
        return b
    return ((b << shift) | (b >> (8 - shift))) & 0xFF


def rol_decode(data: bytes, key: int) -> str:
    return bytes(rol_byte(b, key) for b in data).decode("latin-1")


def is_valid_stream_name(name: str) -> bool:
    return bool(name) and all(0x20 <= ord(c) < 0x80 for c in name)


def is_modern_swx(data: bytes) -> bool:
    if len(data) < 22:
        return False
    if data.startswith(OLE2_MAGIC) or data.startswith(ZIP_MAGIC):
        return False
    return data.find(MARKER, 0, 64) != -1


@dataclass
class Chunk:
    """One chunk record, including non-inline (reference) chunks."""

    offset: int
    section_type: int
    f1: int
    csz: int
    usz: int
    name: str
    data_offset: int | None = None  # absolute offset of deflate payload
    data: bytes | None = None  # decompressed payload (inline chunks)

    @property
    def inline(self) -> bool:
        return self.f1 >= INLINE_F1_THRESHOLD


@dataclass
class SwxFile:
    path: str
    rol_key: int
    chunks: list[Chunk] = field(default_factory=list)

    @property
    def streams(self) -> dict[str, bytes]:
        """First-wins map of stream name -> decompressed payload."""
        out: dict[str, bytes] = {}
        for c in self.chunks:
            if c.data is not None and c.name not in out:
                out[c.name] = c.data
        return out


def _u32(data: bytes, off: int) -> int:
    return int.from_bytes(data[off : off + 4], "little")


def parse(data: bytes, path: str = "<memory>") -> SwxFile:
    if not is_modern_swx(data):
        raise ValueError(f"{path}: not a modern (2015+) SolidWorks file")

    key = data[7]
    swx = SwxFile(path=path, rol_key=key)

    pos = 0
    while True:
        marker_pos = data.find(MARKER, pos)
        if marker_pos == -1:
            break
        if marker_pos < 4:
            pos = marker_pos + 1
            continue
        si = marker_pos - 4
        if si + CHUNK_HEADER_SIZE > len(data):
            pos = marker_pos + 1
            continue

        f1 = _u32(data, si + 0x0E)
        csz = _u32(data, si + 0x12)
        usz = _u32(data, si + 0x16)
        nsz = _u32(data, si + 0x1A)
        if nsz > MAX_NAME_SIZE or csz > MAX_COMPRESSED_SIZE:
            pos = marker_pos + 1
            continue

        name_start = si + CHUNK_HEADER_SIZE
        name_end = name_start + nsz
        if name_end > len(data):
            pos = marker_pos + 1
            continue

        name = rol_decode(data[name_start:name_end], key)
        if not is_valid_stream_name(name):
            pos = marker_pos + 1
            continue

        chunk = Chunk(
            offset=si,
            section_type=data[si + 0x0A],
            f1=f1,
            csz=csz,
            usz=usz,
            name=name,
        )

        if chunk.inline and csz > 0:
            data_end = name_end + csz
            if data_end <= len(data):
                chunk.data_offset = name_end
                try:
                    chunk.data = zlib.decompress(data[name_end:data_end], wbits=-15)
                except zlib.error:
                    chunk.data = None
                swx.chunks.append(chunk)
                pos = data_end
                continue
        elif chunk.inline:
            chunk.data = b""

        swx.chunks.append(chunk)
        pos = marker_pos + len(MARKER)

    return swx


def parse_file(path: str) -> SwxFile:
    with open(path, "rb") as f:
        return parse(f.read(), path)
