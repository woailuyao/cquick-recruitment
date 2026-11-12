#!/usr/bin/env bash
# 把提交目录推送到自己的 Fork 仓库。
# 用法：bash tools/publish_to_github.sh <Fork仓库地址> [分支名] [代理端口]
# 例：  bash tools/publish_to_github.sh https://github.com/xixixixqw/cquick-recruitment week2-liyutong 7890
set -uo pipefail

REPO_URL="${1:-}"
BRANCH="${2:-week2-liyutong}"
PROXY_PORT="${3:-}"

if [[ -z "$REPO_URL" ]]; then
    echo "用法: bash $0 <Fork仓库地址> [分支名] [代理端口]" >&2
    exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
SUBMISSION_DIR="$PROJECT_DIR/submission/视觉组/Week2/liyutong"

if [[ ! -d "$SUBMISSION_DIR" ]]; then
    echo "找不到提交目录: $SUBMISSION_DIR" >&2
    exit 2
fi

if [[ -n "$PROXY_PORT" ]]; then
    export https_proxy="http://127.0.0.1:$PROXY_PORT"
    export http_proxy="http://127.0.0.1:$PROXY_PORT"
    echo "使用代理 127.0.0.1:$PROXY_PORT"
fi

echo "==> 检查能否访问仓库"
if ! git ls-remote "$REPO_URL" >/dev/null 2>&1; then
    echo "无法访问 $REPO_URL" >&2
    echo "请先确认代理已开启，或用浏览器打开仓库确认地址正确。" >&2
    exit 3
fi

WORK_DIR="$HOME/cquick-recruitment"
if [[ -d "$WORK_DIR/.git" ]]; then
    echo "==> 复用已有克隆: $WORK_DIR"
    cd "$WORK_DIR" || exit 4
    git remote set-url origin "$REPO_URL"
    git fetch origin || exit 4
else
    echo "==> 克隆到 $WORK_DIR"
    git clone "$REPO_URL" "$WORK_DIR" || exit 4
    cd "$WORK_DIR" || exit 4
fi

DEFAULT_BRANCH="$(git symbolic-ref --quiet --short refs/remotes/origin/HEAD 2>/dev/null | sed 's|^origin/||')"
DEFAULT_BRANCH="${DEFAULT_BRANCH:-main}"
echo "==> 默认分支: $DEFAULT_BRANCH"

git checkout "$DEFAULT_BRANCH" 2>/dev/null || git checkout -b "$DEFAULT_BRANCH" "origin/$DEFAULT_BRANCH" || exit 5
git pull --ff-only origin "$DEFAULT_BRANCH" 2>/dev/null || true

if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    git checkout "$BRANCH" || exit 5
else
    git checkout -b "$BRANCH" || exit 5
fi

TARGET="视觉组/Week2/liyutong"
mkdir -p "$TARGET"
echo "==> 复制文件到 $TARGET"
cp -r "$SUBMISSION_DIR/." "$TARGET/"

if [[ -z "$(git status --porcelain)" ]]; then
    echo "没有需要提交的改动。"
    exit 0
fi

echo "==> 本次将提交:"
git status --short | head -30
COUNT="$(git status --porcelain | wc -l)"
echo "共 $COUNT 项"

git add "$TARGET"
git commit -m "week2: 颜色目标检测与ROS2通信 - liyutong" || exit 6

echo "==> 推送（需要 GitHub 用户名和 Personal Access Token）"
if ! git push -u origin "$BRANCH"; then
    echo "推送失败。常见原因：" >&2
    echo "  1) 没开代理，GitHub 连不上" >&2
    echo "  2) 密码填成了账号密码，GitHub 现在只接受 Personal Access Token" >&2
    exit 7
fi

echo
echo "推送成功。打开下面的链接创建 Pull Request："
echo "  $REPO_URL/compare/$DEFAULT_BRANCH...$BRANCH?expand=1"
