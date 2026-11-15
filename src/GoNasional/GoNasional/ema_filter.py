from collections import deque

class DataFilter:
    def __init__(self, window_size=5, alpha=0.2):
        self.buffer = deque(maxlen=window_size)
        self.alpha = alpha
        self.filtered = None

    def update(self, data):
        # =========================
        # MEDIAN FILTER
        # =========================
        self.buffer.append(data)

        sorted_data = sorted(self.buffer)
        median = sorted_data[len(sorted_data) // 2]

        # =========================
        # EMA FILTER
        # =========================
        if self.filtered is None:
            self.filtered = median
        else:
            self.filtered = (
                self.alpha * median
                + (1 - self.alpha) * self.filtered
            )

        return self.filtered