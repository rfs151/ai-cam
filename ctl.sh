#!/usr/bin/env bash
# ai-cam 网页演示 —— 启停 / 查看状态
# 部署方式：Docker（restart: unless-stopped，随 Docker 开机自启），页面 http://127.0.0.1:8098
# 经 gateway 访问：https://cam.edgebot.vip
#
#   ./ctl.sh start | stop | restart | rebuild | status | logs
#   ./ctl.sh dev     不用 Docker，本机直接运行（需要先编译 build/capd，见 README）
set -euo pipefail
cd "$(dirname "$0")"
PORT=${PORT:-8098}
case ${1:-status} in
  start)   docker compose up -d ;;
  stop)    docker compose stop ;;
  restart) docker compose restart ;;
  rebuild) docker compose up -d --build ;;
  logs)    docker compose logs -f --tail 100 ;;
  status)
    docker compose ps -a --format 'table {{.Service}}\t{{.Status}}'
    echo "健康检查 http://127.0.0.1:$PORT/api/health -> $(curl -s --noproxy '*' -m 5 "http://127.0.0.1:$PORT/api/health" || echo down)" ;;
  dev)
    [ -x build/capd ] || { echo "先编译：cmake -S . -B build && cmake --build build -j --target capd"; exit 1; }
    cd web/api && exec python3 -m uvicorn app:app --host 0.0.0.0 --port "$PORT" ;;
  *) sed -n '2,8p' "$0"; exit 1 ;;
esac
