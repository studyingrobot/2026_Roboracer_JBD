#!/usr/bin/env python3
"""맵 PGM <-> PNG 왕복 변환 도구 (그림판 등으로 맵을 수정할 때 사용).

    python3 map_png_tool.py to-png map.pgm            # -> map_edit.png
    python3 map_png_tool.py to-pgm map_edit.png map.pgm

to-pgm 은 편집 중 생긴 중간 회색값을 trinary 3색(0/205/254)으로 스냅하고,
원본과 크기가 다르면 거부한다 (yaml 의 origin/resolution 이 깨지므로).
"""
import sys

import numpy as np
from PIL import Image

OCCUPIED, UNKNOWN, FREE = 0, 205, 254


def to_png(pgm_path):
    png_path = pgm_path.rsplit(".", 1)[0] + "_edit.png"
    im = Image.open(pgm_path).convert("L")
    im.save(png_path)
    print(f"{pgm_path} -> {png_path}  ({im.size[0]}x{im.size[1]})")
    print("편집 시 주의: 크기 유지, 안티앨리어싱 없는 연필 툴, 검정/흰색/회색(205)만 사용")


def to_pgm(png_path, pgm_path):
    im = Image.open(png_path).convert("L")

    try:
        orig = Image.open(pgm_path)
        if orig.size != im.size:
            msg = f"크기 불일치: 원본 {orig.size}, 편집본 {im.size} — 리사이즈/크롭하면 안 됩니다"
            if abs(orig.size[0] - im.size[1]) <= 2 and abs(orig.size[1] - im.size[0]) <= 2:
                msg += "\n가로/세로가 뒤바뀐 걸 보니 편집기에서 이미지가 90도 회전된 것 같습니다."
            sys.exit(msg)
    except FileNotFoundError:
        pass

    # 중간값을 3색 중 "가장 가까운" 값으로 스냅.
    # threshold(occupied_thresh/free_thresh) 기준으로 하면 안 된다:
    # 그림판이 205 를 204 로 바꿔놓는 경우 p=0.2 < free_thresh 라서
    # 미탐색 영역이 통째로 자유공간이 되어버린다.
    arr = np.array(im, dtype=np.int16)
    pal = np.array([OCCUPIED, UNKNOWN, FREE], dtype=np.int16)
    snapped = int((~np.isin(arr, pal)).sum())
    arr = pal[np.abs(arr[..., None] - pal).argmin(-1)]

    Image.fromarray(arr.astype(np.uint8), "L").save(pgm_path)  # L 모드는 P5 로 저장됨
    print(f"{png_path} -> {pgm_path}  ({im.size[0]}x{im.size[1]}), 중간값 {snapped}px 스냅됨")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "to-png":
        to_png(sys.argv[2])
    elif len(sys.argv) == 4 and sys.argv[1] == "to-pgm":
        to_pgm(sys.argv[2], sys.argv[3])
    else:
        sys.exit(__doc__)
