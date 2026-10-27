"""Daily local-time quiet windows are independent of microphone privacy."""
from datetime import datetime, timezone
import re
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


class OutputPolicy:
    def __init__(self, *, muted=False, quiet_hours=None, timezone_name='UTC', clock=None):
        if type(muted) is not bool:
            raise ValueError('speaker mute must be boolean')
        try:
            self.timezone = ZoneInfo(timezone_name)
        except (ValueError, TypeError, ZoneInfoNotFoundError):
            raise ValueError('unknown audio quiet-hours timezone') from None
        quiet_hours = [] if quiet_hours is None else quiet_hours
        if not isinstance(quiet_hours, list) or len(quiet_hours) > 8:
            raise ValueError('quiet_hours must be a list of at most eight daily windows')
        self.windows = []
        for window in quiet_hours:
            if not isinstance(window, str) or not re.fullmatch(r'\d{2}:\d{2}-\d{2}:\d{2}', window):
                raise ValueError('quiet hours use HH:MM-HH:MM')
            start, end = window.split('-')
            endpoints = []
            for endpoint in (start, end):
                hour, minute = map(int, endpoint.split(':'))
                if not 0 <= hour < 24 or not 0 <= minute < 60:
                    raise ValueError('quiet-hours time is outside a day')
                endpoints.append(hour * 60 + minute)
            if endpoints[0] == endpoints[1]:
                raise ValueError('equal quiet-hours endpoints are ambiguous; use explicit mute')
            self.windows.append(tuple(endpoints))
        self.muted = muted
        self.clock = clock or (lambda: datetime.now(timezone.utc))

    def set_muted(self, muted):
        if type(muted) is not bool:
            raise ValueError('speaker mute must be boolean')
        self.muted = muted

    @property
    def inhibited(self):
        if self.muted:
            return True
        now = self.clock()
        if now.tzinfo is None:
            raise ValueError('quiet-hours clock requires an aware datetime')
        local = now.astimezone(self.timezone)
        minute = local.hour * 60 + local.minute
        return any(start <= minute < end if start < end else minute >= start or minute < end
                   for start, end in self.windows)
