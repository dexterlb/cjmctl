#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
# "matplotlib",
# "PyQt6"
# ]
# ///

import matplotlib.pyplot as plt

class MercedesRotSensor:
    def __init__(self, storage):
        self._storage = storage
        self._cum_sector = storage.read_cum_sector()

    def update_raw_pos(self, raw_pos):
        self._raw_pos = raw_pos
        self._update_cum_sector()
        self._update_cum_pos()

    def _update_cum_pos(self):
        jumpovers = self._cum_sector // 4
        if self._raw_sector() == 0 and self._raw_pos > 0.75:
            jumpovers -= 1
        self.cum_pos = jumpovers + self._raw_pos

    def _update_cum_sector(self):
        zone_left, zone_right = {
            0: ((0.5, 0.75), (0.25, 0.5)),
            1: ((0.75, 1), (0.5, 0.75)),
            2: ((0, 0.25), (0.75, 1)),
            3: ((0.25, 0.5), (0, 0.25)),
        }[self._raw_sector()]

        if is_in(self._raw_pos, zone_right):
            self._cum_sector += 1
            self._storage.save_cum_sector(self._cum_sector)
        elif is_in(self._raw_pos, zone_left):
            self._cum_sector -= 1
            self._storage.save_cum_sector(self._cum_sector)

    def _raw_sector(self):
        # pay attention to use a `%` function that gives a positive result
        # for negative numbers, like python's
        return self._cum_sector % 4

    def save_to_storage(self):
        self.times_saved += 1


def is_in(x, interval):
    a, b = interval
    return x >= a and x <= b

# --------------- testing stuff ----------------
def test_main():
    do_test(
        't1',
        points=[
            (0, 0),
            (5, 1.5),
            (7, 0.5),
            (10, 4),
            (11, -0.2),
            (11.2, 0.2),
            (12, -2),
            (13, 0.1),
            (13.2, -0.1),
            (13.4, 0.1),
            (13.5, -0.1),
            (13.6, 0),
        ],
        plot=True
    )

def do_test(name, points, plot=False, expected_saves=None):
    actual_cum_trajectory = interpolate_points(points, 0.01)
    raw_trajectory = [(x, y % 1) for x, y in actual_cum_trajectory]

    calculated_cum_trajectory = []
    storage = FakeStorage()
    rot_sens = MercedesRotSensor(storage)
    for x, y in raw_trajectory:
        storage.update(x)
        rot_sens.update_raw_pos(y)
        calculated_cum_trajectory.append((x, rot_sens.cum_pos))

    err = calc_err(actual_cum_trajectory, calculated_cum_trajectory)
    num_saves = len(storage.storage_saves)

    if err < 0.00001:
        if expected_saves is not None and num_saves != expected_saves:
            print(f'{name}: expected to save {expected_saves} times to storage but instead saved {num_saves} times')
        else:
            print(f'{name}: test ok')
    else:
        print(f'{name}: test failed (err={err})')

    if plot:
        plot_trajs(name, {
            'actual': actual_cum_trajectory,
            'raw': raw_trajectory,
            'calc': calculated_cum_trajectory,
        }, storage.storage_saves)

class FakeStorage:
    def __init__(self):
        self.storage_saves = []
        self.cum_sector = 0
        self.t = 0

    def update(self, t):
        self.t = t

    def read_cum_sector(self):
        return self.cum_sector

    def save_cum_sector(self, cum_sector):
        self.cum_sector = cum_sector
        self.storage_saves.append(self.t)


def calc_err(l1, l2):
    if max((abs(x1 - x2) for (x1, _), (x2, _) in zip(l1, l2))) > 0.00000001:
        raise RuntimeError('cannot calculate error for lists with different X coordintate sets')
    return max((abs(y1 - y2) for (_, y1), (_, y2) in zip(l1, l2)))

def plot_trajs(name, data, epochs):
    plt.figure(figsize=(8, 6))

    for label, points in data.items():
        x, y = zip(*points)
        plt.plot(x, y, label=label)

    for i, xv in enumerate(epochs):
        plt.axvline(
            x=xv, color='red', linestyle='--', linewidth=1,
            label='storage save' if i == 0 else None
        )

    plt.legend()
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("name")
    plt.grid(True)
    plt.show()


def interpolate_points(points, step_x):
    """
    WARNING: VIBE CODED FUNCTION

    Linearly interpolate between (x, y) points with a given step in x.

    Args:
        points (list of tuple): List of (x, y) points, must be sorted by x.
        step_x (float): Step size for interpolated x values.

    Returns:
        list of tuple: Interpolated (x, y) points.
    """
    if not points or step_x <= 0:
        return []

    result = []
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]

        # Always include the starting point
        if not result:
            result.append((x0, y0))

        # Interpolate between x0 and x1
        x = x0 + step_x
        while x < x1:
            # Linear interpolation formula
            t = (x - x0) / (x1 - x0)
            y = y0 + t * (y1 - y0)
            result.append((x, y))
            x += step_x

        # Add the endpoint of this segment
        result.append((x1, y1))

    return result

if __name__ == '__main__':
    test_main()
