class YawFilter:
    def __init__(self, threshold_deg=5.0):
        self.offset = 0.0
        self.threshold = threshold_deg
        self.last_good = 0.0
        # self.alpha = 0.0
        self.heading = 0.0
        self.raw_heading = 0.0
        self.frozen = False
        self.initialized = False

    def update(self, raw_yaw_deg, robot_state = None):
        self.raw_heading = raw_yaw_deg + self.offset

        if robot_state == "NORMAL":
            pass
        elif abs(self.raw_heading - self.last_good) > self.threshold:
            self.heading = self.raw_heading
            self.last_good = self.raw_heading

        self.frozen = (self.heading != self.raw_heading)
        return self.heading
    
    def reset(self, raw_yaw_deg):
        self.offset = -raw_yaw_deg
        self.last_good = 0.0
        self.heading = 0.0

    def set_offset(self, offset_deg):
        self.offset = offset_deg

    def snapshot(self):
        return {
            'offset': self.offset,
            'heading': self.heading,
            'raw_heading': self.raw_heading,
            'frozen': self.frozen,
        }


if __name__ == "__main__":
    print("=== YawFilter Demo ===")
    f = YawFilter(threshold_deg=5)
    print(f"Threshold: {f.threshold}°\n")

    print("--- Drift simulation (robot diam, yaw -2/tick) ---")
    print("tick   yaw  heading  frozen")
    for tick in range(8):
        raw = -tick * 2
        h = f.update(raw, None)
        print(f"{tick:4d}  {raw:4d}  {h:7.1f}  {'YES' if f.frozen else 'no '} "
              f"{'(exceeded threshold)' if not f.frozen and tick > 0 else ''}")

    print("\n--- Reset heading ---")
    f.reset(-10)
    print(f"reset(-10) → offset={f.offset}, last_good={f.last_good}")
    h = f.update(-10, None)
    print(f"update(-10) → heading={h:.1f} (should be 0)")

    print("\n--- NORMAL freeze ---")
    h = f.update(50, "NORMAL")
    print(f"update(50, NORMAL) → heading={h:.1f} (should stay {f.last_good:.1f})")

    print("\n--- Set offset manual ---")
    f.set_offset(15)
    h = f.update(0, None)
    print(f"set_offset(15), update(0) → heading={h:.1f} (should be 15)")

    print("\n=== Demo selesai ===")
