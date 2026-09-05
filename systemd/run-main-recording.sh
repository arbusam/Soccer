#!/usr/bin/env bash
set -euo pipefail

project_dir="/home/arhan/Soccer"
recordings_dir="${project_dir}/recordings"
session_name="$(date +'%Y-%m-%d_%H-%M-%S')"

mkdir -p "${recordings_dir}"

exec "${project_dir}/.venv/bin/python" \
    "${project_dir}/main.py" \
    --record-session "${recordings_dir}/${session_name}"
