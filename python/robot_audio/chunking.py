"""Split an utterance into speakable chunks for pipelined synthesis.

Time-to-first-sound is what a listener perceives as latency, and it is set by
the FIRST chunk alone: everything after it is synthesized while earlier audio
is already playing. So the first chunk is cut deliberately short, and later
ones are packed up to a larger budget. Splitting prefers sentence ends, then
clause punctuation, then a word boundary, so a chunk never breaks mid-word.
"""
import re

# Sentence-final punctuation, keeping the mark with the sentence it ends.
_SENTENCE = re.compile(r'(?<=[.!?…])\s+')
# Clause breaks, used only when one sentence alone exceeds a chunk budget.
_CLAUSE = re.compile(r'(?<=[,;:—–])\s+')


def split_for_speech(text, first_max=48, rest_max=160):
    """Yield chunks of `text`, the first no longer than `first_max`."""
    text = ' '.join(text.split())
    if not text:
        return []

    chunks = []
    pending = ''

    def budget():
        # The first chunk out of the door is the only one the listener waits
        # for, so it keeps the tight budget whether or not it is still pending.
        return first_max if not chunks else rest_max

    for sentence in _SENTENCE.split(text):
        if not sentence:
            continue

        joined = f'{pending} {sentence}'.strip()
        if pending and len(joined) <= budget():
            pending = joined
            continue

        if pending:
            chunks.append(pending)
            pending = ''

        if len(sentence) <= budget():
            pending = sentence
            continue

        pieces = _fit(sentence, budget(), rest_max)
        chunks.extend(pieces[:-1])
        pending = pieces[-1]

    if pending:
        chunks.append(pending)
    return _merge_runts(chunks, first_max, rest_max)


def _fit(sentence, budget, rest_budget):
    """Break one over-long sentence into pieces, by clause then by word."""
    pieces, current = [], ''
    for clause in _CLAUSE.split(sentence):
        limit = budget if not pieces else rest_budget
        candidate = f'{current} {clause}'.strip()
        if current and len(candidate) > limit:
            pieces.append(current)
            current = clause
        else:
            current = candidate
    if current:
        pieces.append(current)

    # A single clause can still exceed the budget -- fall back to word
    # boundaries so a chunk never splits a word.
    out = []
    for piece in pieces:
        limit = budget if not out else rest_budget
        out.extend(_by_words(piece, limit) if len(piece) > limit else [piece])
    return out


def _by_words(piece, budget):
    out, current = [], ''
    for word in piece.split():
        candidate = f'{current} {word}'.strip()
        if current and len(candidate) > budget:
            out.append(current)
            current = word
        else:
            current = candidate
    if current:
        out.append(current)
    return out


def _merge_runts(chunks, first_max, rest_max):
    """Fold a trailing scrap into its predecessor.

    A one-word tail synthesized on its own is audible as a hiccup, and it costs
    a whole extra synthesis call to say almost nothing.
    """
    merged = []
    for chunk in chunks:
        budget = first_max if len(merged) <= 1 else rest_max
        if merged and len(chunk) < 12 and len(merged[-1]) + len(chunk) + 1 <= budget * 2:
            merged[-1] = f'{merged[-1]} {chunk}'
        else:
            merged.append(chunk)
    return merged
