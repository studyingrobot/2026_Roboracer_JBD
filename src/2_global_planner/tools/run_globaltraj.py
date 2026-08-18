#!/usr/bin/env python3
"""main_globaltraj_f110.py 를 opt_type 지정해서 실행하는 얇은 런처.

Raceline-Optimization/ 은 git subtree 라서 직접 고치지 않는다. 나중에
`git subtree pull` 할 때 충돌이 나기 때문이다. 그런데 최적화 방식을 고르는
opt_type 이 원본 스크립트 안에 하드코딩되어 있고 CLI 인자가 없다.

    main_globaltraj_f110.py:81   opt_type = 'mintime'

그래서 소스를 메모리에서만 치환한 뒤 exec 한다. 디스크의 원본은 그대로 둔다.

사용법:
    python3 run_globaltraj.py --opt-type mincurv_iqp \
        --map_name mytrack --export_path /work/outputs/mytrack_raceline.csv
"""

import argparse
import os
import re
import sys

# 원본이 자기 위치를 기준으로 inputs/, outputs/, params/ 를 찾으므로
# 경로를 정확히 넘겨줘야 한다.
HERE = os.path.dirname(os.path.abspath(__file__))
UPSTREAM_DIR = os.path.join(os.path.dirname(HERE), 'Raceline-Optimization')
UPSTREAM_MAIN = os.path.join(UPSTREAM_DIR, 'main_globaltraj_f110.py')

VALID_OPT_TYPES = ('shortest_path', 'mincurv', 'mincurv_iqp', 'mintime')


def main():
    parser = argparse.ArgumentParser(
        description='opt_type 을 지정해 F1TENTH 전역경로 최적화를 실행한다.',
        add_help=False,
    )
    parser.add_argument(
        '--opt-type',
        default='mincurv_iqp',
        choices=VALID_OPT_TYPES,
        help=(
            'shortest_path: 최단경로 / mincurv: 최소곡률 1회 / '
            'mincurv_iqp: 최소곡률 반복QP (기본, 권장) / '
            'mintime: 최소랩타임 (느리고 차량 파라미터에 민감)'
        ),
    )
    parser.add_argument('-h', '--help', action='store_true')
    known, passthrough = parser.parse_known_args()

    if known.help:
        parser.print_help()
        print('\n--- 아래 인자는 main_globaltraj_f110.py 로 그대로 전달된다 ---')
        passthrough = ['--help']
        known.opt_type = 'mincurv_iqp'

    if not os.path.isfile(UPSTREAM_MAIN):
        sys.exit(
            f'ERROR: 최적화기 원본을 찾을 수 없다: {UPSTREAM_MAIN}\n'
            '       git subtree 가 제대로 들어왔는지 확인할 것.'
        )

    source = open(UPSTREAM_MAIN, encoding='utf-8').read()
    patched, count = re.subn(
        r"^opt_type\s*=\s*['\"][a-z_]+['\"]",
        f"opt_type = '{known.opt_type}'",
        source,
        count=1,
        flags=re.MULTILINE,
    )
    if count != 1:
        sys.exit(
            'ERROR: main_globaltraj_f110.py 에서 opt_type 대입문을 찾지 못했다.\n'
            '       upstream 이 바뀐 것이니 이 런처를 손봐야 한다.'
        )

    print(f'[run_globaltraj] opt_type = {known.opt_type}')
    print(f'[run_globaltraj] upstream = {UPSTREAM_MAIN}')

    # 원본은 상대 import (opt_mintime_traj, helper_funcs_glob) 를 쓰므로
    # 자기 폴더가 cwd 이자 sys.path 에 있어야 한다.
    os.chdir(UPSTREAM_DIR)
    sys.path.insert(0, UPSTREAM_DIR)
    sys.argv = [UPSTREAM_MAIN] + passthrough

    namespace = {
        '__file__': UPSTREAM_MAIN,
        '__name__': '__main__',
    }
    exec(compile(patched, UPSTREAM_MAIN, 'exec'), namespace)


if __name__ == '__main__':
    main()
