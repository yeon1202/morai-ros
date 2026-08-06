#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_udp_dump : MORAI 가 보내는 UDP 패킷을 파싱하지 않고 원시 그대로 뜯어본다.

왜 필요한가
  우리는 "규정 포트 9109(Competition Vehicle Status)는 position 을 안 준다" 고 믿고
  개발해왔다. 근거는 "9109 로 바꿨더니 position 이 0,0,0 이었다" 는 관찰 하나뿐이다.

  그런데 브릿지는 9111 용 **고정 오프셋**(pos = raw[77:89])으로 파싱한다. 9109 가 다른
  구조의 패킷이면 그 자리는 엉뚱한 바이트고, 그게 0 으로 보일 수 있다.
  즉 "안 준다" 가 아니라 "우리가 못 읽는다" 일 가능성이 있다.

  그리고 morai_msgs 에 EgoNoisyStatus(noisy_position / noisy_velocity)가 있다.
  대회 규정의 "GPS/IMU 노이즈 인가됨" 과 맞물린다.

  이 스크립트는 **아무 가정도 하지 않는다.** 길이와 헤더와 바이트를 그대로 보여준다.

ROS 가 필요 없다. 컨테이너 안이든 호스트든 그냥 python3 로 돌면 된다.

사용법
  python3 diag_udp_dump.py --port 9109
  python3 diag_udp_dump.py --port 9111 --count 5     # 아는 것(9111)과 비교용
  python3 diag_udp_dump.py --port 9109 --bytes 160   # 더 많이 보기

  --port    수신 포트 (기본 9109)
  --count   몇 개 받고 끝낼지 (기본 10)
  --bytes   패킷 앞에서 몇 바이트를 16진수로 볼지 (기본 144)

먼저 MORAI Network Settings 에서 해당 항목을 **켜야** 한다. 포트만 맞추고 항목을
안 켜면 MORAI 가 송신 소켓을 안 잡아 아무것도 안 온다.

주의: 브릿지가 이미 쓰는 포트(9111 등)는 동시에 못 연다. 그 포트를 볼 때는
브릿지를 잠깐 끄고 돌린다. 9109 는 브릿지가 안 쓰므로 그대로 돌려도 된다.
"""
import argparse
import socket
import struct
from collections import Counter


def printable(b):
    """앞쪽 ASCII 헤더를 눈으로 보기 위한 변환."""
    return ''.join(chr(c) if 32 <= c < 127 else '.' for c in b)


def hexdump(b, width=16):
    out = []
    for off in range(0, len(b), width):
        chunk = b[off:off + width]
        hx = ' '.join('%02x' % c for c in chunk)
        out.append('  %4d  %-*s  %s' % (off, width * 3, hx, printable(chunk)))
    return '\n'.join(out)


def as_9111(b):
    """9111(MoraiInfo) 레이아웃으로 읽으면 뭐가 나오는지. 비교용일 뿐 정답 아님."""
    try:
        pos = struct.unpack('fff', b[77:89])
        rpy = struct.unpack('fff', b[89:101])
        vel = struct.unpack('fff', b[101:113])
        return ('  pos (%.3f, %.3f, %.3f)\n'
                '  rpy (%.3f, %.3f, %.3f)\n'
                '  vel (%.3f, %.3f, %.3f)' % (pos + rpy + vel))
    except Exception as e:
        return '  (길이가 짧아 읽을 수 없음: %s)' % e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=9109)
    ap.add_argument('--count', type=int, default=10)
    ap.add_argument('--bytes', type=int, default=144)
    a = ap.parse_args()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('0.0.0.0', a.port))
    except OSError as e:
        print('포트 %d 를 열 수 없다: %s' % (a.port, e))
        print('이미 다른 프로세스(브릿지 등)가 쓰는 중일 수 있다. 끄고 다시 시도할 것.')
        return
    s.settimeout(10.0)
    print('포트 %d 에서 %d개 대기 중... (10초 안에 안 오면 종료)\n' % (a.port, a.count))

    lens = Counter()
    heads = Counter()
    first = None
    got = 0
    while got < a.count:
        try:
            data, addr = s.recvfrom(65535)
        except socket.timeout:
            print('타임아웃. 아무것도 안 왔다.')
            print('  - MORAI Network Settings 에서 해당 항목이 켜져(초록) 있는지')
            print('  - Destination PORT 가 %d 인지' % a.port)
            print('  - 시나리오 로드 후 Load 를 눌렀는지')
            break
        got += 1
        lens[len(data)] += 1
        heads[printable(data[:16])] += 1
        if first is None:
            first = data
            print('[첫 패킷]  보낸 곳 %s:%s   길이 %d bytes' % (addr[0], addr[1], len(data)))
            print('  헤더(앞 16바이트 ASCII):  %r' % printable(data[:16]))
            print()
            print(hexdump(data[:a.bytes]))
            print()
            print('[9111 레이아웃으로 읽으면]  ※ 맞다는 뜻이 아니라 비교용')
            print(as_9111(data))
            print()

    if got:
        print('=' * 60)
        print('받은 패킷 %d개' % got)
        print('  길이 분포 :', dict(lens))
        print('  헤더 분포 :', dict(heads))
        print()
        print('판단 기준')
        print('  길이가 229 이고 헤더가 #MoraiInfo$ 면  -> 9111 과 같은 구조.')
        print('     그러면 position 0,0,0 은 진짜로 안 주는 것이다.')
        print('  길이나 헤더가 다르면                  -> 다른 구조.')
        print('     "안 준다" 가 아니라 "못 읽는다" 였던 것이고, 오프셋을 새로 찾아야 한다.')
        print('     MORAI-NetworkModule 저장소 lib/define/ 에 공식 정의가 있다.')
    s.close()


if __name__ == '__main__':
    main()
