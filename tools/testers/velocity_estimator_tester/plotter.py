#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
# "matplotlib",
# "numpy",
# ]
# ///

import numpy as np
import matplotlib.pyplot as plt
import sys
import gzip
import subprocess
from time import sleep

with gzip.open(sys.argv[1], 'r') as f:
    timestamps_us, positions, vel_estimates_baseline = np.loadtxt(f).T

timestamps = timestamps_us / 1000000.0

input=''.join((f'{float(t)} {float(p)}\n' for t, p in zip(timestamps, positions)))

p = subprocess.run(sys.argv[2], shell=False, text=True, input=input, capture_output=True)

results = []
for line in (line for line in p.stdout.split('\n') if line):  # I <3 python
    results += [[float(s) for s in line.split()]]

sub_timestamps, pos_estimates, vel_estimates, acc_estimates = list(zip(*results))

sys.stderr.write(p.stderr)

for i, (ts, sub_ts) in enumerate(zip(timestamps, sub_timestamps)):
    if abs(ts - sub_ts) > 0.00001:
        raise RuntimeError(f"uhh, something's wrong (timestamp {i} is supposed to be {ts} but is actually {sub_ts}")

fig, axs = plt.subplots(3, figsize=(10, 11))

axs[0].set(xlabel="time")
axs[0].set(ylabel="position")
axs[0].plot(timestamps, pos_estimates, color="red")
axs[0].plot(timestamps, positions, color="green")

axs[1].set(xlabel="time")
axs[1].set(ylabel="estimated velocity")

axs[1].plot(timestamps, vel_estimates, color="red")
axs[1].plot(timestamps, vel_estimates_baseline, color="green")

axs[2].set(xlabel="time")
axs[2].set(ylabel="estimated acceleration")
axs[2].plot(timestamps, acc_estimates)

fig.savefig('/tmp/baba.png', dpi=200)
print('created /tmp/baba.png')
