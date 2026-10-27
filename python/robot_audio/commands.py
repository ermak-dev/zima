"""Pure, read-only battery intent/response layer; no ROS, shell, network or LLM.

Integration: handle_command(text, reader) calls reader() only for a recognized
battery question. The reader returns BatterySnapshot. A caller may declare
freshness='fresh' only with independent evidence of sensor measurement age;
receipt of a Player packet is NOT such evidence. The fixed BatteryProvider is
useful for offline use. snapshot_from_json parses the bounded robot battery
wire contract and intentionally never upgrades its unknown freshness.
"""
from dataclasses import dataclass
import json
import math
from typing import Callable

from .phrases import normalize_phrase


@dataclass(frozen=True)
class BatterySnapshot:
    percent: float | None = None
    voltage: float | None = None
    charging: bool | None = None
    freshness: str = 'unknown'  # fresh / stale / unknown; sensor age, not receipt age


@dataclass(frozen=True)
class BatteryProvider:
    """Fixed, side-effect-free reader callable (also accepts an unknown snapshot)."""
    snapshot: BatterySnapshot = BatterySnapshot()

    def __call__(self) -> BatterySnapshot:
        return self.snapshot


def _number(value, low, high):
    try:
        return (type(value) in (int, float) and low <= value <= high
                and math.isfinite(value))
    except OverflowError:
        return False


def snapshot_from_json(text: str) -> BatterySnapshot:
    """Parse at most 512 characters; malformed or missing fields stay unknown.

    Backend valid masks: voltage=1, percent=8, charging=16. Neither JSON timestamps
    nor freshness supplied by this transport establish sensor measurement age.
    """
    if not isinstance(text, str) or len(text) > 512:
        return BatterySnapshot()
    try:
        def invalid_constant(_):
            raise ValueError('non-finite JSON')
        data = json.loads(text, parse_constant=invalid_constant)
        if not isinstance(data, dict):
            return BatterySnapshot()
        valid = data.get('valid')
        if type(valid) is not int or not 0 <= valid <= 0xffffffff:
            return BatterySnapshot()
        percent, voltage, charging = (data.get(key) for key in ('percent', 'voltage', 'charging'))
        return BatterySnapshot(
            percent=percent if valid & 8 and _number(percent, 0, 100) else None,
            voltage=voltage if valid & 1 and _number(voltage, 0, float('inf')) and voltage > 0 else None,
            charging=charging if valid & 16 and type(charging) is bool else None,
        )
    except (ValueError, TypeError, OverflowError):
        return BatterySnapshot()


_ONES = ('ноль', 'один', 'два', 'три', 'четыре', 'пять', 'шесть', 'семь', 'восемь', 'девять',
         'десять', 'одиннадцать', 'двенадцать', 'тринадцать', 'четырнадцать', 'пятнадцать',
         'шестнадцать', 'семнадцать', 'восемнадцать', 'девятнадцать')
_TENS = ('', '', 'двадцать', 'тридцать', 'сорок', 'пятьдесят', 'шестьдесят', 'семьдесят', 'восемьдесят', 'девяносто')


def _percent_words(value: int) -> str:
    if value == 100:
        words = 'сто'
    elif value < 20:
        words = _ONES[value]
    else:
        words = _TENS[value // 10] + ((' ' + _ONES[value % 10]) if value % 10 else '')
    suffix = ('процентов' if 11 <= value % 100 <= 14 else
              'процент' if value % 10 == 1 else
              'процента' if value % 10 in (2, 3, 4) else 'процентов')
    return f'{words} {suffix}'


def battery_response(snapshot: BatterySnapshot) -> str:
    """Bounded spoken answer, with number words so a TTS transcript gate can match."""
    if not isinstance(snapshot, BatterySnapshot) or not _number(snapshot.percent, 0, 100):
        return 'Заряд пока неизвестен. Нет достоверных данных батареи.'
    if snapshot.freshness != 'fresh':
        return 'Текущий заряд неизвестен. Данные устарели или их давность неизвестна.'
    value = round(snapshot.percent)
    prefix = 'У меня' if value == snapshot.percent else 'Примерно'
    return f'{prefix} {_percent_words(value)} заряда.'


_BATTERY_INTENTS = frozenset(('какой у тебя заряд', 'сколько заряда', 'какой заряд',
                              'сколько у тебя заряда', 'какое тебя заряд', 'какой тебя заряд'))


def is_battery_command(text):
    return isinstance(text, str) and len(text) <= 512 and normalize_phrase(text) in _BATTERY_INTENTS


def handle_command(text: str, reader: Callable[[], BatterySnapshot], *, allow_reported=False) -> str:
    """Handle only exact normalized battery questions; never invoke unknown intents.

    Exceptions
    from a reader are contained; exception text is never spoken.
    """
    if not is_battery_command(text):
        return 'Неизвестная команда. Я могу сообщить заряд батареи.'
    try:
        snapshot = reader()
        if (allow_reported is True and isinstance(snapshot, BatterySnapshot) and
                _number(snapshot.percent, 0, 100) and snapshot.freshness in ('unknown', 'stale')):
            number = _percent_words(round(snapshot.percent))
            qualifier = 'Давность измерения неизвестна.' if snapshot.freshness == 'unknown' else 'Данные устарели.'
            approximate = 'примерно ' if round(snapshot.percent) != snapshot.percent else ''
            return f'Робот сообщает: {approximate}{number}. {qualifier}'
        return battery_response(snapshot)
    except Exception:
        return 'Заряд пока неизвестен. Не удалось прочитать данные батареи.'
