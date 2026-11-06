"""Prepare an uploaded document for Celeste.

PDFs are handed to the retrieval path; everything else is treated as text and
goes into the conversation context directly.
"""

from pathlib import Path
from typing import NamedTuple


class DocumentError(ValueError):
    pass


class PreparedDocument(NamedTuple):
    text: str | None
    data: bytes | None
    saved_name: str
    kind: str


def prepare_document(filename, data, max_text_bytes=200 * 1024):
    name = Path(filename).name

    if Path(name).suffix.lower() == ".pdf":
        return PreparedDocument(text=None, data=data, saved_name=name, kind="pdf")

    try:
        text = data.decode("utf-8-sig")
    except UnicodeError as exc:
        raise DocumentError("file is not a PDF or UTF-8 text") from exc
    if len(text.encode("utf-8")) > max_text_bytes:
        raise DocumentError("text is larger than %d kB" % (max_text_bytes // 1024))
    saved_name = name if name.lower().endswith(".txt") else name + ".txt"
    return PreparedDocument(text=text, data=None, saved_name=saved_name, kind="text")
