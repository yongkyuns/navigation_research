"""Bounded chronological rollback window; not a smoother or an integrity bound."""
from collections import deque


class ReplayWindow:
    def __init__(self, seconds=20.0, max_events=4096):
        if seconds <= 0 or max_events < 1:
            raise ValueError('Invalid recovery budget')
        self.seconds = seconds
        self.max_events = max_events
        self.nav = None
        self.t = None
        self.score_t = None
        self.key = None
        self.version = None
        self.events = deque()
        self.attempted = False
        self.status = 'empty'

    def capture(self, nav, t, score_t, key, version):
        self.clear()
        self.nav = nav.clone()  # map-free posterior, not a selected road branch
        self.t, self.score_t = t, score_t
        self.key, self.version = key, version
        self.status = 'available'

    def append(self, kind, value):
        if self.nav is None or self.attempted:
            return
        t = float(value if kind == 'map' else value[0])
        previous = self.t if not self.events else (
            self.events[-1][1] if self.events[-1][0] == 'map' else self.events[-1][1][0])
        if t < previous:
            self.invalidate('out_of_order')
            return
        if t - self.t > self.seconds or len(self.events) >= self.max_events:
            self.invalidate('expired')  # never replay a silently truncated prefix
            return
        self.events.append((kind, t if kind == 'map' else tuple(map(float, value))))

    def invalidate(self, reason):
        self.clear()
        self.status = reason
        self.attempted = True

    def clear(self):
        if self.nav is not None:
            self.nav.close()
        self.nav = None
        self.events.clear()
        self.status = 'empty'
        self.attempted = False

    def __del__(self):
        if getattr(self, 'nav', None) is not None:
            self.nav.close()
