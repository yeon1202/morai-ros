#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
import_global_path : 대회 배포 전역경로를 우리 경로 CSV 형식으로 들여온다.

배포본은 공백 구분 "x y z" 텍스트이고, 우리 `path_tracker.py` 는 헤더 `x,y,z` 를
가진 CSV 를 읽는다. 그 변환과 정제를 한다.

정제하는 것: **간격이 0 인 중복점**. 배포본(4430점)에 38개 있다. 접선 방향을
구할 때 0 으로 나누기가 나므로 반드시 걸러야 한다.

들여온 뒤 달라지는 것 두 가지 (코드는 안 건드렸다. 필요하면 그때 판단):
  - 점 간격이 우리 기록본 ~0.6m 에서 **0.5m** 로 바뀐다. `LOCAL_PATH_SIZE=140`
    이 그대로면 local path 가 84m -> **70m** 가 된다. 60km/h 평형 간격이 26.3m
    이므로 ACC 지평선으로는 여전히 넉넉하다.
  - 배포본은 **닫힌 루프**(시작=끝)다. 그런데 `IS_CLOSED_PATH` 는 False 로 둔다.
    대회는 한 바퀴만 돌고 멈추는데, True 로 하면 끝에서 되감겨 두 바퀴째로
    넘어간다(2026-07-29 에 고친 그 문제). 기하학적으로 닫혀 있는 것과
    "닫힌 경로로 취급할지" 는 별개다.

사용법
  rosrun path_tracking import_global_path.py \
      --src ~/Downloads/CARSA_dataset/전역경로*/전역경로*/2026_molit_comp_global_path.txt

  --src   배포본 txt 경로
  --out   저장 경로 (기본: 패키지의 path/path_smooth.csv)
  --dry   쓰지 않고 통계만 출력
"""
import argparse
import csv
import math
import os
import shutil


def load(src):
    pts = []
    with open(src) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            pts.append(tuple(float(v) for v in parts[:3]))
    return pts


def dedup(pts, eps=1e-9):
    """간격 0 인 연속 중복점 제거."""
    out = [pts[0]]
    for p in pts[1:]:
        if math.dist(p[:2], out[-1][:2]) > eps:
            out.append(p)
    return out


def stats(pts, name):
    d = [math.dist(pts[i][:2], pts[i + 1][:2]) for i in range(len(pts) - 1)]
    print('  %-8s %5d 점  길이 %.1f m  간격 평균 %.3f (최소 %.3f, 최대 %.3f)'
          % (name, len(pts), sum(d), sum(d) / len(d), min(d), max(d)))
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True)
    ap.add_argument('--out', default=None)
    ap.add_argument('--dry', action='store_true')
    a = ap.parse_args()

    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = a.out or os.path.join(pkg, 'path', 'path_smooth.csv')

    raw = load(a.src)
    clean = dedup(raw)
    print('배포본 -> 정제')
    stats(raw, '원본')
    stats(clean, '정제후')
    print('  제거된 중복점 %d개' % (len(raw) - len(clean)))

    closed = math.dist(clean[0][:2], clean[-1][:2])
    print('  시작-끝 거리 %.3f m %s' % (closed, '(닫힌 루프)' if closed < 1.0 else ''))

    if a.dry:
        print('\n--dry 이므로 쓰지 않음')
        return

    if os.path.exists(out):
        bak = out.replace('.csv', '_handrecorded.csv.bak')
        shutil.copy2(out, bak)
        print('\n기존 경로 백업 -> %s' % os.path.basename(bak))

    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['x', 'y', 'z'])
        for p in clean:
            w.writerow(['%.6f' % p[0], '%.6f' % p[1], '%.6f' % p[2]])
    print('저장 -> %s' % out)


if __name__ == '__main__':
    main()
