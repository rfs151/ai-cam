# ai-cam 网页演示镜像：capd（C++ 采集引擎）+ Aravis 虚拟 GigE 相机 + FastAPI 后端 + 静态页面
# 构建参数（都可选，默认是官方源；国内可在 .env 里设置）：
#   REGISTRY    镜像仓库，如 docker.1ms.run
#   APT_MIRROR  Ubuntu apt 源，如 http://mirrors.aliyun.com
# 海康 MVS SDK 是闭源的，不放进镜像；镜像里只有开源的 Aravis。
ARG REGISTRY=docker.io

FROM ${REGISTRY}/library/ubuntu:24.04 AS build
ARG APT_MIRROR=""
RUN if [ -n "$APT_MIRROR" ]; then \
      sed -i "s#http://archive.ubuntu.com#$APT_MIRROR#g; s#http://security.ubuntu.com#$APT_MIRROR#g" /etc/apt/sources.list.d/ubuntu.sources; fi \
 && apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake pkg-config libaravis-dev libusb-1.0-0-dev libjpeg-turbo8-dev ca-certificates \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /src
COPY CMakeLists.txt ./
COPY include include
COPY src src
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
 && cmake --build build -j"$(nproc)" --target capd aravis_capture sim_capture
# aravis-tools 会连带装上 GTK / GStreamer（给 arv-viewer 用），这里只取出虚拟相机程序
RUN apt-get update && cd /tmp && apt-get download aravis-tools \
 && dpkg-deb -x aravis-tools_*.deb /tmp/arv && cp /tmp/arv/usr/bin/arv-fake-gv-camera-0.8 /src/build/

FROM ${REGISTRY}/library/ubuntu:24.04
ARG APT_MIRROR=""
RUN if [ -n "$APT_MIRROR" ]; then \
      sed -i "s#http://archive.ubuntu.com#$APT_MIRROR#g; s#http://security.ubuntu.com#$APT_MIRROR#g" /etc/apt/sources.list.d/ubuntu.sources; fi \
 && apt-get update && apt-get install -y --no-install-recommends \
      libaravis-0.8-0 libjpeg-turbo8 python3 python3-fastapi python3-uvicorn python3-websockets \
 && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY --from=build /src/build/capd /src/build/aravis_capture /src/build/sim_capture /app/build/
COPY --from=build /src/build/arv-fake-gv-camera-0.8 /usr/local/bin/
COPY tools/verify_record.py /app/tools/
COPY web /app/web
ENV CAPD=/app/build/capd VERIFY=/app/tools/verify_record.py REC_ROOT=/tmp/ai-cam-rec PYTHONUNBUFFERED=1
EXPOSE 8098
HEALTHCHECK --interval=30s --timeout=4s --retries=3 \
  CMD python3 -c "import urllib.request; urllib.request.urlopen('http://127.0.0.1:8098/api/health', timeout=3)"
WORKDIR /app/web/api
CMD ["python3", "-m", "uvicorn", "app:app", "--host", "0.0.0.0", "--port", "8098", "--proxy-headers", "--forwarded-allow-ips=*"]
