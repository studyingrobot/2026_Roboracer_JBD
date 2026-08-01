# 2026 전BOT대 Roboracer

# 폴더 구조 

2026_Roboracer_JBD/
├── config/              # 컨트롤러/파라미터 튜닝 yaml
├── docs/                # 문서
├── maps/                # SLAM으로 딴 맵 (.pgm, .yaml)
├── paths/               # 전역경로 최적화 결과 (.csv)
├── README.md
└── src/
    ├── 1_slam_mapping/     # SLAM → 맵 저장
    ├── 2_global_planner/   # 전역 경로 최적화 → CSV
    ├── 3_localization/     # map 기준 현재 위치 추정
    ├── 4_local_planner/    # 장애물 회피 로컬 경로
    ├── 5_controller/       # Pure Pursuit / MPC
    └── 6_bringup/          # launch 파일
