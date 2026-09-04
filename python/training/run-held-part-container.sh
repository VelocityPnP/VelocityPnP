#!/bin/sh
set -eu

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "usage: $0 DATASET [RUN_NAME]" >&2
    exit 2
fi

script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
repository=$(CDPATH= cd -- "$script_directory/../.." && pwd)
dataset=$1
run_name=${2:-held-part-pose-v1}

case "$dataset" in
    /*) dataset_path=$dataset ;;
    *) dataset_path=$repository/$dataset ;;
esac

case "$dataset_path" in
    "$repository"/*) ;;
    *)
        echo "dataset must be inside the repository: $dataset_path" >&2
        exit 2
        ;;
esac

dataset_in_container=/workspace/${dataset_path#"$repository"/}

docker build \
    --file "$repository/python/training/Dockerfile.spark" \
    --tag velocitypnp-training:spark \
    "$repository"

docker run --rm --gpus all --ipc host \
    --user "$(id -u):$(id -g)" \
    --volume "$repository:/workspace" \
    --workdir /workspace \
    velocitypnp-training:spark \
    python python/tools/train_held_part_pose.py \
    "$dataset_in_container" --name "$run_name" --device cuda
