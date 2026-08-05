# 2026 전BOT대 Roboracer

## 파이프라인

1. **Mapping** — SLAM으로 맵 생성 → 전역 경로 최적화 → CSV 저장
2. **Tracking** — CSV(목표 경로) + Localization(현재 위치) → 오차 계산 → 조향/속도 명령
3. **Avoidance** — 전역 경로는 유지, 장애물 구간만 로컬 경로 생성

## 폴더 구조

~~~
2026_Roboracer_JBD/
├── config/                 # 컨트롤러/파라미터 튜닝 yaml
├── README.md
└── src/
    ├── 1_slam_mapping/     # SLAM → 맵 저장
    ├── 2_global_planner/   # 전역 경로 최적화 → CSV
    ├── 3_localization/     # map 기준 현재 위치 추정
    ├── 4_local_planner/    # 장애물 회피 로컬 경로
    ├── 5_controller/       # Pure Pursuit / MPC
    └── 6_bringup/          # launch 파일
~~~

## 브랜치 규칙

- `main` : 차에 올려서 검증된 코드만
- `feat/*` : 기능 개발 중
- `fix/*` : 버그 수정
- `exp/*` : 실험
