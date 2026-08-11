"""
Split speech into short pieces before giving to TTS services. 
Cap length, prefer sentence ends, and merge tiny openers into the next sentence.
"""

from __future__ import annotations

import re

# chunk size settings
MAX_CHUNK_CHARS = 100
MIN_CHUNK_CHARS = 28

# Common title abbreviations that end with a period but not sentence ends
_ABBREVIATIONS = {
    'dr', 'mr', 'mrs', 'ms', 'prof', 'sr', 'jr', 'st', 'vs', 'etc',
    'approx', 'dept', 'est', 'fig', 'gen', 'gov', 'inc', 'ltd', 'no',
}

# Sentence end: delimiter + whitespace + (end of string or capital letter)
_SENTENCE_END_RE = re.compile(
    r'([.?!;:])(?:\s+)(?=[A-Z]|$)'
)


def chunk_speech_text(
    text: str,
    max_chars: int = MAX_CHUNK_CHARS,
    min_chars: int = MIN_CHUNK_CHARS,
) -> list[str]:
    """Return TTS chunks by merging whole sentences when possible."""
    if text is None:
        return []
    normalized = ' '.join(str(text).split())
    if not normalized:
        return []

    sentences: list[str] = []
    for piece in _split_sentences(normalized):
        if len(piece) <= max_chars:
            sentences.append(piece)
        else:
            sentences.extend(_split_oversized(piece, max_chars))

    chunks = _greedy_merge(sentences, max_chars)
    return _merge_trailing_short(chunks, max_chars, min_chars)


def _split_sentences(text: str) -> list[str]:
    """Split on sentence delimiters with light abbreviation/decimal guards."""
    parts: list[str] = []
    start = 0
    for match in _SENTENCE_END_RE.finditer(text):
        delim_index = match.start(1)
        delim = match.group(1)
        # Sskip decimals: digit . digit
        if delim == '.' and delim_index > 0 and delim_index + 1 < len(text):
            if text[delim_index - 1].isdigit() and text[delim_index + 1].isdigit():
                continue
        # skip known abbreviations and single-letter initials (like U.S. / A.)
        if delim == '.' and _is_abbreviation_period(text, delim_index):
            continue
        end = match.end(1)  # include the delimiter in the sentence
        piece = text[start:end].strip()
        if piece:
            parts.append(piece)
        start = match.end()  # skip trailing whitespace after delimiter
    tail = text[start:].strip()
    if tail:
        parts.append(tail)
    return parts


def _is_abbreviation_period(text: str, period_index: int) -> bool:
    """True if the period at period_index looks like an abbreviation, not end of sentence."""
    # walk left to the token before the period
    i = period_index - 1
    while i >= 0 and text[i].isalpha():
        i -= 1
    token = text[i + 1:period_index]
    if not token:
        return False
    if token.lower() in _ABBREVIATIONS:
        return True
    # single-letter initials: styles like "U.S." or "A. B. King"
    if len(token) == 1 and token.isalpha():
        return True
    return False


def _split_oversized(sentence: str, max_chars: int) -> list[str]:
    """Split a sentence longer than max_chars on commas, then word boundaries."""
    pieces: list[str] = []
    for comma_piece in _split_on_commas(sentence):
        if len(comma_piece) <= max_chars:
            pieces.append(comma_piece)
        else:
            pieces.extend(_split_on_words(comma_piece, max_chars))
    return pieces


def _split_on_commas(text: str) -> list[str]:
    """Split after commas and keep the comma with the preceding fragment."""
    if ',' not in text:
        return [text]
    raw = re.split(r'(?<=,)\s*', text)
    return [p.strip() for p in raw if p and p.strip()]


def _split_on_words(text: str, max_chars: int) -> list[str]:
    """Pack words into chunks of at most max_chars; hard-split if needed in the end."""
    words = text.split()
    if not words:
        return []
    chunks: list[str] = []
    buf = ''
    for word in words:
        if len(word) > max_chars:
            if buf:
                chunks.append(buf)
                buf = ''
            for i in range(0, len(word), max_chars):
                chunks.append(word[i:i + max_chars])
            continue
        candidate = word if not buf else f'{buf} {word}'
        if len(candidate) <= max_chars:
            buf = candidate
        else:
            chunks.append(buf)
            buf = word
    if buf:
        chunks.append(buf)
    return chunks


def _greedy_merge(sentences: list[str], max_chars: int) -> list[str]:
    chunks: list[str] = []
    buf = ''
    for sentence in sentences:
        if not buf:
            buf = sentence
            continue
        candidate = f'{buf} {sentence}'
        if len(candidate) <= max_chars:
            buf = candidate
        else:
            chunks.append(buf)
            buf = sentence
    if buf:
        chunks.append(buf)
    return chunks


def _merge_trailing_short(
    chunks: list[str],
    max_chars: int,
    min_chars: int,
) -> list[str]:
    if len(chunks) < 2:
        return chunks
    last = chunks[-1]
    prev = chunks[-2]
    if len(last) < min_chars and len(prev) + 1 + len(last) <= max_chars:
        return chunks[:-2] + [f'{prev} {last}']
    return chunks


def force_split_chunk(text: str, max_chars: int | None = None) -> list[str]:
    """Emergency split when still too long.
    Prefer a space near the middle, else cut hard.
    """
    if max_chars is None:
        max_chars = max(1, len(text) // 2)
    text = ' '.join(str(text).split())
    if not text:
        return []
    if len(text) <= max_chars:
        return [text]
    # prefer splitting at a space near the midpoint/max_chars
    cut = min(max_chars, len(text))
    space = text.rfind(' ', 0, cut)
    if space <= 0:
        space = cut
    left = text[:space].strip()
    right = text[space:].strip()
    parts = []
    if left:
        parts.append(left)
    if right:
        parts.append(right)
    return parts or [text[:cut]]
