#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_smoother : 손으로 딴 삐뚤빼뚤한 path.csv 를 매끄럽게 만든다.
------------------------------------------------------------------
읽기:  path/path.csv         쓰기:  path/path_smooth.csv  (원본은 그대로 보존)
방법:  이동평균(moving average) - 주변 점들의 평균으로 흔들림 제거.
       WINDOW 를 키우면 더 매끄럽지만 코너가 더 깎인다.
실행:  python3 path_smoother.py   (ROS 불필요)
"""
import os
import sys
import csv
import numpy as np

# 스무딩 강도. 클수록 매끈(코너 더 깎임). 실행 시 인자로 조절 가능:
#   python3 path_smoother.py 15    <- WINDOW=15로 실행
WINDOW = int(sys.argv[1]) if len(sys.argv) > 1 else 9
if WINDOW % 2 == 0:
    WINDOW += 1       # 짝수면 홀수로


def load(path):
    xs, ys, zs = [], [], []
    with open(path) as f:
        r = csv.reader(f)
        next(r)                                  # 헤더 skip
        for row in r:
            xs.append(float(row[0]))
            ys.append(float(row[1]))
            zs.append(float(row[2]))
    return np.array(xs), np.array(ys), np.array(zs)


def smooth(arr, window, closed):
    """이동평균. closed(폐곡선)면 앞뒤를 이어붙여 경계도 매끄럽게."""
    half = window // 2
    if closed:
        padded = np.concatenate([arr[-half:], arr, arr[:half]])
    else:
        padded = np.concatenate([np.full(half, arr[0]), arr, np.full(half, arr[-1])])
    kernel = np.ones(window) / window
    return np.convolve(padded, kernel, mode='valid')


def total_turning_deg(x, y):
    """경로가 얼마나 '꺾이는지' 총합 (삐뚤수록 큼). 매끄러움 척도."""
    ang = np.arctan2(np.diff(y), np.diff(x))
    dang = np.diff(ang)
    dang = (dang + np.pi) % (2 * np.pi) - np.pi   # -pi~pi 정규화
    return np.sum(np.abs(dang)) * 180.0 / np.pi


def main():
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    path_dir = os.path.join(os.path.dirname(scripts_dir), 'path')
    src = os.path.join(path_dir, 'path.csv')
    dst = os.path.join(path_dir, 'path_smooth.csv')

    x, y, z = load(src)
    closed = np.hypot(x[0] - x[-1], y[0] - y[-1]) < 5.0     # 시작~끝 5m 이내면 폐곡선
    print('waypoint %d개 | 폐곡선(closed): %s | WINDOW=%d' % (len(x), closed, WINDOW))

    sx = smooth(x, WINDOW, closed)
    sy = smooth(y, WINDOW, closed)
    sz = smooth(z, WINDOW, closed)

    before = total_turning_deg(x, y)
    after = total_turning_deg(sx, sy)
    print('총 방향변화(작을수록 매끄러움):  전 %.0f°  ->  후 %.0f°' % (before, after))

    with open(dst, 'w') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'z'])
        for i in range(len(sx)):
            w.writerow([sx[i], sy[i], sz[i]])
    print('저장 완료:', dst)


if __name__ == '__main__':
    main()
