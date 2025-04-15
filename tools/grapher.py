#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
# "matplotlib",
# "pyserial",
# "strictyaml",
# "PyQt6"
# ]
# ///

import random
import time
import sys
import re
import os
import stat
import traceback
import strictyaml
import tkinter as tk
from functools import wraps

from matplotlib import pyplot as plt
from matplotlib import animation

import serial
from dataclasses import dataclass

@dataclass
class DataLine():
    timestamp: float
    items: dict

class FrameParser:
    def __init__(self, cfg):
        self.cfg = cfg
        self.dataline_regex = re.compile(self.cfg['dataline_regex'])
        self.keyval_regex = re.compile(self.cfg['keyval_regex'])
        self.timestamp_regex = re.compile(self.cfg['timestamp_regex'])

    def parse_meas_line(self, line: str) -> DataLine:
        if not self.dataline_regex.search(line):
            sys.stdout.write(line.replace('\r\n', '\n').replace('\r', '\n') + '\n')
            return None


        try:
            tss = self.timestamp_regex.search(line).group(1)
            ts = float(tss) * float(self.cfg['timestamp_multiplier'])

            items = {m.group('key'): m.group('value') for m in self.keyval_regex.finditer(line)}
            items = {k: v for k, v in items.items() if v}
            items = {k: self.parse_val(v) for k, v in items.items()}
        except ValueError:
            print('received corrupted data: ' + line)
            return None # received corrupted text

        return DataLine(timestamp=ts, items=items)

    def parse_val(self, s):
        try:
            return float(s)
        except:
            return s.strip()

    def read_line_from_stream(self, ser):
        s = ''
        while True:
            c = ser.read(1)
            if type(c) is not str:
                try:
                    c = c.decode('ascii')
                except:
                    c = '?'
            s += c
            if c in ('\r', '\n'):
                return s

    def frames_from_stream(self, f):
        while True:
            line = self.read_line_from_stream(f)
            dataline = self.parse_meas_line(line)
            if dataline:
                yield [dataline]

    def frames_from_serial(self, port):
        try:
            baud = self.cfg['serial_baud']
        except KeyError:
            baud = 9600
        ser = serial.Serial(port, baud)
        return self.frames_from_stream(ser)

    def frames_from_pipe(self, filename):
        with open(filename, 'r') as f:
            return self.frames_from_stream(f)

    def frames_from_stdin(self, ):
        return self.frames_from_stream(sys.stdin)

    def frames_from_regular_file(self, filename):
        with open(filename, 'rb') as f:
            max_line_len = 2
            while True:
                pos = f.seek(-max_line_len, os.SEEK_END)
                lines = f.read().decode('utf-8').splitlines()
                if len(lines) < 2:
                    if pos == 0:
                        raise RuntimeError('no newlines in input file?')
                    max_line_len *= 2
                    sys.stderr.write(f'bumping line size to {max_line_len}\n')
                    continue

                dataline = self.parse_meas_line(lines[-1])
                if dataline:
                    yield [dataline]

    def frames_from_regular_file_noskip(self, filename):
        with open(filename, 'rb') as f:
            while True:
                data = f.read()
                if len(data) == 0:
                    # maybe file was truncated?
                    cur_pos = f.seek(0, os.SEEK_CUR)
                    end_pos = f.seek(0, os.SEEK_END)
                    if cur_pos > end_pos:
                        print(f'input file was truncated (cur {cur_pos}, end {end_pos}), re-reading')
                        f.seek(0, os.SEEK_SET)
                    else:
                        # file was not truncated, continue from where we left off
                        f.seek(cur_pos, os.SEEK_SET)
                    yield []
                    continue

                lines = data.decode('utf-8').splitlines()
                datalines = (self.parse_meas_line(line) for line in lines)
                ok_datalines = [dl for dl in datalines if dl]
                yield ok_datalines

    def frames_from_file(self, filename):
        mode = os.lstat(filename).st_mode
        if stat.S_ISFIFO(mode):
            return self.frames_from_pipe(filename)
        elif stat.S_ISCHR(mode):
            return self.frames_from_serial(filename)
        else:
            # return self.frames_from_regular_file(filename)
            return self.frames_from_regular_file_noskip(filename)

class GraphAnimator():
    def __init__(self, gen, cfg):
        self.cfg = cfg

        self.fig = plt.figure()

        self.subplot_regexes = [re.compile(r) for r in self.cfg['subplot_regexes']]

        if len(self.subplot_regexes) == 0:
            raise RuntimeError("no subplots defined")
        elif len(self.subplot_regexes) == 1:
            self.subplots = [self.fig.add_subplot(1, 1, 1)]
        else:
            self.subplots = [self.fig.add_subplot(2, (len(self.subplot_regexes) + 1) // 2, i + 1) for i, r in enumerate(self.subplot_regexes)]

        self.plots = dict()
        self.val_sequences = dict()
        self.timestamp_sequences = dict()

        self.gen = gen
        self.colour_map = list(plt.colormaps['viridis'].colors)
        random.shuffle(self.colour_map)
        self.colour_idx = 0
        self.prev_ts = 0
        self.timetravel_keys = set()

    def animation_callback(self, datalines):
        updated_keys = set()
        reset_keys = set()

        for dataline in datalines:
            cur_ts = dataline.timestamp
            if cur_ts < self.prev_ts:
                self.timetravel_keys |= set(self.val_sequences.keys())
            self.prev_ts = cur_ts

            for k, v in dataline.items.items():
                if k in self.val_sequences:
                    if k in self.timetravel_keys:
                        print(f'detected time travel for {k}')
                        self.val_sequences[k] = [v]
                        self.timestamp_sequences[k] = [dataline.timestamp]
                        self.timetravel_keys.remove(k)

                    self.val_sequences[k].append(v)
                    self.timestamp_sequences[k].append(dataline.timestamp)
                    updated_keys.add(k)
                else:
                    self.val_sequences[k] = [v]
                    self.timestamp_sequences[k] = [dataline.timestamp]
                    reset_keys.add(k)

        for k in reset_keys:
            ax = self.get_subplot(k)
            self.plots[k], = ax.plot(
                self.timestamp_sequences[k],
                self.val_sequences[k],
                color=self.next_colour(),
                label=k
            )
        for k in updated_keys - reset_keys:
            self.plots[k].set_data(self.timestamp_sequences[k], self.val_sequences[k])
            self.plots[k].set_label(k + ': ' + str(self.val_sequences[k][-1]))

        for ax in self.subplots:
            ax.relim()
            ax.autoscale_view()
            ax.legend()

        return self.plots.values()

    def get_subplot(self, k):
        for i, r in enumerate(self.subplot_regexes):
            if r.search(k):
                return self.subplots[i]
        raise RuntimeError(f'no subplot that matches value {k}')

    def next_colour(self):
            colour = self.colour_map[self.colour_idx]
            self.colour_idx += 1
            return colour

    def animate(self):
        anim = animation.FuncAnimation(
            self.fig,
            self.animation_callback,
            frames=self.gen,
            interval=1,
            blit=False, # if this is true, drawing is much faster, but text on axes is not animated
            cache_frame_data=False
        )
        plt.show()


    def frames_from_args(self):
        if len(sys.argv) == 1:
            return self.frames_from_stdin()
        elif len(sys.argv) == 2:
            return self.frames_from_file(sys.argv[1])
        else:
            raise RuntimeError("unknown args (read the source pls)")

def parse_cfg(filename):
    with open(filename, 'r') as f:
        return strictyaml.load(f.read()).data

if __name__ == '__main__':
    if len(sys.argv) == 2:
        cfg = parse_cfg(sys.argv[1])
        fp = FrameParser(cfg)
        gen = fp.frames_from_stdin
    elif len(sys.argv) == 3:
        cfg = parse_cfg(sys.argv[1])
        fp = FrameParser(cfg)
        gen = fp.frames_from_file(sys.argv[2])
        gengen = lambda: gen
    else:
        print(f'usage: {sys.argv[0]} <config file> [input file or serial port]')
        os.exit(1)

    GraphAnimator(gengen, cfg).animate()
