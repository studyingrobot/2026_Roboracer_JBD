# Raceline-Optimization 전용 실행 환경.
#
# 이 툴은 Ubuntu 20.04 / Python 3.8 기준으로 개발됐고 requirements.txt 가
# numpy==1.18.1 처럼 옛 버전에 고정되어 있다. 호스트(Ubuntu 22.04 / Python 3.10,
# ROS Humble)에 그대로 설치하면 빌드가 깨지고 ROS 파이썬 환경도 오염된다.
#
# 최적화기는 ROS 노드가 아니라 CSV 한 번 뽑고 끝나는 오프라인 도구라서
# 컨테이너에 가둬두는 편이 안전하다.
#
# 빌드 컨텍스트는 src/2_global_planner (상위 폴더)다:
#   docker build -f tools/Dockerfile -t raceline-opt:py38 .

FROM python:3.8-slim-bullseye

# quadprog / scipy 등이 소스 빌드로 떨어질 때를 대비한 컴파일 도구.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        gfortran \
        pkg-config \
        libopenblas-dev \
        liblapack-dev \
        git \
    && rm -rf /var/lib/apt/lists/*

# 고정 버전 의존성은 옛 pip 해석기와 궁합이 좋다.
RUN python -m pip install --no-cache-dir "pip<24" "setuptools<60" wheel

COPY Raceline-Optimization/requirements.txt /tmp/requirements.txt
RUN python -m pip install --no-cache-dir -r /tmp/requirements.txt

# 소스는 COPY 하지 않고 실행 시 마운트한다. subtree 를 고쳐도 재빌드가 필요 없다.
WORKDIR /work

# 기본은 헤드리스. 플롯을 보려면 실행 시 -e MPLBACKEND=TkAgg 와 X11 포워딩을 준다.
ENV MPLBACKEND=Agg
ENV PYTHONUNBUFFERED=1
