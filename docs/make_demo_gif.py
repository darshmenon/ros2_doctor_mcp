#!/usr/bin/env python3
"""Render docs/demo_raw.txt (real diagnose_robot results captured on a UR3 + MoveIt Gazebo sim) into docs/demo.gif."""
import os
import re

from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
raw = open(os.path.join(HERE, 'demo_raw.txt')).read()
FONT = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf', 15)
W, H, LH = 1040, 400, 19
BG, FG, DIM = (22, 24, 30), (205, 205, 205), (120, 125, 135)
RED, GREEN, YELLOW, CYAN = (255, 95, 95), (110, 220, 130), (240, 200, 80), (110, 200, 240)
frames = [b.strip('\n') for b in re.split(r'^=== frame: .*\n', raw, flags=re.M) if b.strip()]


def color(line):
    s = line.lstrip()
    if s.startswith('you'):
        return CYAN
    if s.startswith('tool'):
        return YELLOW
    if s.startswith('agent') or line.startswith('        Fix'):
        return GREEN
    if '"ERROR"' in line or '"ok": false' in line or 'ERROR' in line:
        return RED
    if '"ok": true' in line:
        return GREEN
    return FG


def render(lines):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, W, 26], fill=(40, 43, 52))
    d.text((12, 5), 'ros2-doctor-mcp   (real results captured through MCP on a UR3 + MoveIt Gazebo sim)', font=FONT, fill=DIM)
    y = 38
    for ln in lines:
        d.text((14, y), ln, font=FONT, fill=color(ln))
        y += LH
    return img


imgs, durs = [], []
for f in frames:
    lines = f.splitlines()
    # reveal: the question and the call first, then the result, then the answer
    cut = [i for i, l in enumerate(lines) if l.startswith('agent')][0]
    imgs += [render(lines[:2]), render(lines[:cut]), render(lines)]
    durs += [900, 1400, 3200]
imgs[0].save(os.path.join(HERE, 'demo.gif'), save_all=True, append_images=imgs[1:], duration=durs, loop=0, optimize=True)
print('wrote docs/demo.gif', len(imgs), 'frames')
