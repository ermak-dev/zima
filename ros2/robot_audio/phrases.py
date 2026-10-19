"""Lexical phrase matching and configurable pronunciation metadata.

Text normalization deliberately ignores stress. Positive/negative pronunciations
are metadata for ASR/TTS matching and acoustic evaluation, not an acoustic KWS
implementation or proof that a speaker used the requested stress.
"""
from dataclasses import dataclass
import unicodedata


def _words(text):
    """Yield lexical spans in the original string, including combining marks."""
    start = None
    for index, char in enumerate(text):
        if char.isalnum() or (start is not None and unicodedata.category(char).startswith('M')):
            if start is None:
                start = index
        elif start is not None:
            yield start, index
            start = None
    if start is not None:
        yield start, len(text)


def _canonical(text, *, keep_stress=False):
    text = unicodedata.normalize('NFD', text)
    if not keep_stress:
        text = text.replace('́', '')
    return unicodedata.normalize('NFC', text).casefold().replace('ё', 'е')


def _lexical(text, *, keep_stress=False):
    return ' '.join(_canonical(text[start:end], keep_stress=keep_stress)
                    for start, end in _words(text))


def normalize_phrase(text: str) -> str:
    """Return casefolded words with punctuation, acute stress and ё folded.

    Canonically equivalent NFC/NFD inputs match; other diacritics (e.g. й's
    breve) are preserved. There is no stemming, fuzzy or acoustic matching.
    """
    return _lexical(text)


def _separator(char):
    return char.isspace() or unicodedata.category(char).startswith('P')


def strip_wake_prefix(transcript: str, phrase: str, *, allow_asr_substitution=False) -> str:
    """Remove one complete lexical wake prefix, otherwise return stripped input.

    Punctuation/whitespace surrounding the matched prefix is consumed. The
    remaining command keeps its original case, Unicode spelling and punctuation
    (apart from surrounding whitespace). Empty lexical phrases are a no-op.
    Stress cannot be checked here: both pronunciations have the same words.
    After acoustic activation, allow_asr_substitution may tolerate one replaced
    letter in the last word (at least four letters, at least two phrase words).
    Earlier words must match exactly; this never edits the remaining command.
    """
    expected = normalize_phrase(phrase).split()
    spans = list(_words(transcript))
    if not expected or len(spans) < len(expected):
        return transcript.strip()
    if not all(_separator(char) for char in transcript[:spans[0][0]]):
        return transcript.strip()
    for index, (word, (start, end)) in enumerate(zip(expected, spans)):
        actual = normalize_phrase(transcript[start:end])
        if actual != word:
            substitution = (allow_asr_substitution is True and len(expected) >= 2 and
                            index == len(expected) - 1 and 4 <= len(word) <= 32 and
                            len(actual) == len(word) and word.isalpha() and actual.isalpha() and
                            sum(a != b for a, b in zip(actual, word)) == 1)
            if not substitution:
                return transcript.strip()
    end = spans[len(expected) - 1][1]
    while end < len(transcript) and _separator(transcript[end]):
        end += 1
    return transcript[end:].strip()


@dataclass(frozen=True)
class PhraseSpec:
    """Explicit phrase and contrasting pronunciations; no global wake default.

    Pronunciation strings retain their supplied stress marks and Unicode form.
    All variants must contain the same lexical words, and negative variants must
    differ from the positive pronunciation beyond case/punctuation/ё alone.
    """
    text: str
    pronunciation: str
    negative_pronunciations: tuple[str, ...] = ()
    language: str = 'ru'

    def __post_init__(self):
        if not isinstance(self.text, str) or not normalize_phrase(self.text):
            raise ValueError('text must contain lexical words')
        if not isinstance(self.language, str) or not self.language.strip():
            raise ValueError('language must be nonempty')
        if isinstance(self.negative_pronunciations, str):
            raise ValueError('negative_pronunciations must be a collection of strings')
        try:
            negatives = tuple(self.negative_pronunciations)
        except TypeError as error:
            raise ValueError('negative_pronunciations must be a collection of strings') from error
        for variant in (self.pronunciation, *negatives):
            if not isinstance(variant, str) or normalize_phrase(variant) != normalize_phrase(self.text):
                raise ValueError('pronunciation must match the lexical text')
        positive = _lexical(self.pronunciation, keep_stress=True)
        if any(_lexical(variant, keep_stress=True) == positive for variant in negatives):
            raise ValueError('negative pronunciation must contrast with the positive')
        object.__setattr__(self, 'negative_pronunciations', negatives)

    def to_tts_dict(self) -> dict:
        """Return fresh JSON-ready metadata, not instructions to be spoken.

        The caller/provider adapter must keep metadata separate from spoken text;
        this module neither invokes TTS nor guarantees provider stress support.
        """
        return {'text': self.text, 'pronunciation': self.pronunciation,
                'negative_pronunciations': list(self.negative_pronunciations),
                'language': self.language}
