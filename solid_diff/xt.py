"""Load Parasolid XT node graphs via ps-parser (vendored, see REFERENCES.md)."""

from __future__ import annotations

import io
import sys
from pathlib import Path

_PSPARSER_DIR = Path(__file__).resolve().parent.parent / "vendor" / "ps-parser"
if str(_PSPARSER_DIR) not in sys.path:
    sys.path.insert(0, str(_PSPARSER_DIR))

from psparser import load_schema, parse_ps  # noqa: E402

_SCHEMA_PATH = _PSPARSER_DIR / "assets" / "sch_13006.s_t"
_schema_cache = None


def _schema():
    global _schema_cache
    if _schema_cache is None:
        _schema_cache = load_schema(str(_SCHEMA_PATH))
    return _schema_cache


class Graph:
    """Parasolid node graph with id-based link resolution."""

    def __init__(self, nodes: list[dict]):
        self.nodes = {n["id"]: n for n in nodes}

    @classmethod
    def from_bytes(cls, data: bytes) -> "Graph":
        return cls(parse_ps(io.BytesIO(data), _schema()))

    @classmethod
    def from_file(cls, path: str) -> "Graph":
        return cls.from_bytes(Path(path).read_bytes())

    def deref(self, ref: int | None) -> dict | None:
        return self.nodes.get(ref) if ref is not None else None

    def by_type(self, name: str):
        return [n for n in self.nodes.values() if n["node_name"] == name]

    def chain(self, first: int | None, link: str = "next"):
        """Follow a linked list of nodes starting at id `first`."""
        seen = set()
        node = self.deref(first)
        while node is not None and node["id"] not in seen:
            seen.add(node["id"])
            yield node
            node = self.deref(node.get(link))

    def face_loops(self, face: dict) -> list[dict]:
        return list(self.chain(face.get("loop")))

    def loop_halfedges(self, loop: dict) -> list[dict]:
        return list(self.chain(loop.get("halfedge"), link="forward"))

    def attributes(self, node: dict) -> list[dict]:
        """All ATTRIBUTE nodes owned by node, as (identifier, values) pairs."""
        out = []
        for att in self.chain(node.get("attributes_features")):
            if att["node_name"] != "ATTRIBUTE":
                continue
            adef = self.deref(att.get("definition"))
            ident = self.deref(adef.get("identifier")) if adef else None
            name = ident.get("string") if ident else None
            fields = att.get("fields")
            if not isinstance(fields, list):
                fields = [fields] if fields is not None else []
            values = [
                f.get("values") for f in (self.deref(fid) for fid in fields) if f
            ]
            out.append({"name": name, "values": values})
        return out

    def face_color(self, face: dict) -> tuple[float, float, float] | None:
        for att in self.attributes(face):
            if att["name"] == "SDL/TYSA_COLOUR" and att["values"]:
                rgb = att["values"][0]
                if isinstance(rgb, list) and len(rgb) == 3:
                    return tuple(rgb)
        return None
