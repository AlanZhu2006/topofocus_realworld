#!/usr/bin/env bash
# Install the executable-source RedNet + Detectron2 pixel stack into hub/.venv.
# This downloads only the source-referenced model/runtime artifacts. It does
# not fetch HM3D, simulator scenes, overlays, or SIF images.
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
python_bin="$workspace/hub/.venv/bin/python"
checkpoint_dir="$workspace/artifacts/checkpoints"
toolchain_dir="$workspace/artifacts/toolchains"
download_dir="$toolchain_dir/cuda-12.8.1-downloads"
cuda_home="$toolchain_dir/cuda-12.8.1-local"
detectron_commit="b4a4a3bd136852dae5fb1de37978dee412653e31"
detectron_archive="$toolchain_dir/detectron2-${detectron_commit}.tar.gz"
detectron_source="$toolchain_dir/detectron2-src-${detectron_commit}"

[[ -x "$python_bin" ]] || {
  echo "Missing Hub Python: $python_bin" >&2
  exit 1
}
mkdir -p "$checkpoint_dir" "$download_dir" "$toolchain_dir"

download_verified() {
  local url="$1"
  local destination="$2"
  local expected_size="$3"
  local expected_sha256="$4"
  local temporary="${destination}.partial.$$"
  if [[ ! -f "$destination" ]]; then
    curl -fL --retry 3 --output "$temporary" "$url"
    mv "$temporary" "$destination"
  fi
  local observed_size
  observed_size="$(stat -c '%s' "$destination")"
  [[ "$observed_size" == "$expected_size" ]] || {
    echo "Size mismatch for $destination: $observed_size != $expected_size" >&2
    exit 1
  }
  printf '%s  %s\n' "$expected_sha256" "$destination" | sha256sum -c -
}

rednet_checkpoint="$checkpoint_dir/rednet_semmap_mp3d_40.pth"
[[ -f "$rednet_checkpoint" ]] || {
  echo "Missing RedNet checkpoint: $rednet_checkpoint" >&2
  echo "Obtain the source-linked model from:" >&2
  echo "https://drive.google.com/file/d/1U0dS44DIPZ22nTjw0RfO431zV-lMPcvv/view" >&2
  exit 1
}
[[ "$(stat -c '%s' "$rednet_checkpoint")" == 656550984 ]] || {
  echo "RedNet checkpoint size mismatch: $rednet_checkpoint" >&2
  exit 1
}
printf '%s  %s\n' \
  f94d1c62a73bc05690ae29200d3dbd033ff243e7ce91755d1cd928bde844f995 \
  "$rednet_checkpoint" | sha256sum -c -

download_verified \
  "https://dl.fbaipublicfiles.com/detectron2/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x/137849600/model_final_f10217.pkl" \
  "$checkpoint_dir/detectron2_mask_rcnn_R_50_FPN_3x_model_final_f10217.pkl" \
  177841981 \
  9a737e290372f1f70994ebcbd89d8004dbb3ae30a605fd915a190fa4a782dd66

download_verified \
  "https://github.com/facebookresearch/detectron2/archive/${detectron_commit}.tar.gz" \
  "$detectron_archive" \
  1017574 \
  ad08474b62ba7fc12347126b40d536e14499aa5e4bc0505c3e4dd3e79d82a0ee

download_verified \
  "https://developer.download.nvidia.com/compute/cuda/redist/cuda_nvcc/linux-x86_64/cuda_nvcc-linux-x86_64-12.8.93-archive.tar.xz" \
  "$download_dir/cuda_nvcc-linux-x86_64-12.8.93-archive.tar.xz" \
  79015464 \
  9961b3484b6b71314063709a4f9529654f96782ad39e72bf1e00f070db8210d3
download_verified \
  "https://developer.download.nvidia.com/compute/cuda/redist/cuda_cudart/linux-x86_64/cuda_cudart-linux-x86_64-12.8.90-archive.tar.xz" \
  "$download_dir/cuda_cudart-linux-x86_64-12.8.90-archive.tar.xz" \
  1354240 \
  8d566b5fe745c46842dc16945cf36686227536decd2302c372be86da37faca68
download_verified \
  "https://developer.download.nvidia.com/compute/cuda/redist/cuda_cccl/linux-x86_64/cuda_cccl-linux-x86_64-12.8.90-archive.tar.xz" \
  "$download_dir/cuda_cccl-linux-x86_64-12.8.90-archive.tar.xz" \
  928524 \
  0740e9e01e4f15e17c5ab8d68bba4f8ec0eb6b84edccba4ac45112d2d2174e4b

if [[ ! -x "$cuda_home/bin/nvcc" ]]; then
  [[ ! -e "$cuda_home" ]] || {
    echo "Refusing incomplete CUDA toolchain directory: $cuda_home" >&2
    exit 1
  }
  mkdir -p "$cuda_home"
  tar -xf "$download_dir/cuda_nvcc-linux-x86_64-12.8.93-archive.tar.xz" \
    -C "$cuda_home" --strip-components=1
  tar -xf "$download_dir/cuda_cudart-linux-x86_64-12.8.90-archive.tar.xz" \
    -C "$cuda_home" --strip-components=1
  tar -xf "$download_dir/cuda_cccl-linux-x86_64-12.8.90-archive.tar.xz" \
    -C "$cuda_home" --strip-components=1
fi
if [[ ! -e "$cuda_home/lib64" ]]; then
  ln -s lib "$cuda_home/lib64"
fi
"$cuda_home/bin/nvcc" --version

if [[ ! -f "$detectron_source/setup.py" ]]; then
  [[ ! -e "$detectron_source" ]] || {
    echo "Refusing incomplete Detectron2 source directory: $detectron_source" >&2
    exit 1
  }
  mkdir -p "$detectron_source"
  tar -xzf "$detectron_archive" -C "$detectron_source" --strip-components=1
fi

site_packages="$("$python_bin" - <<'PY'
from pathlib import Path
import torch
print(Path(torch.__file__).resolve().parent.parent)
PY
)"
nvidia_root="$site_packages/nvidia"
for header in \
  "$nvidia_root/cusparse/include/cusparse.h" \
  "$nvidia_root/cublas/include/cublas_v2.h" \
  "$nvidia_root/cublas/include/cublasLt.h" \
  "$nvidia_root/cusolver/include/cusolverDn.h"; do
  [[ -f "$header" ]] || {
    echo "PyTorch CUDA development header is missing: $header" >&2
    exit 1
  }
done

"$python_bin" -m pip install 'ninja>=1.11,<2'
PATH="$workspace/hub/.venv/bin:$PATH" \
CUDA_HOME="$cuda_home" \
FORCE_CUDA=1 \
TORCH_CUDA_ARCH_LIST=8.9 \
MAX_JOBS="${MAX_JOBS:-6}" \
CPATH="$nvidia_root/cusparse/include:$nvidia_root/cublas/include:$nvidia_root/cusolver/include" \
LIBRARY_PATH="$cuda_home/lib:$nvidia_root/cusparse/lib:$nvidia_root/cublas/lib:$nvidia_root/cusolver/lib" \
  "$python_bin" -m pip install --no-build-isolation "$detectron_source"

"$python_bin" - <<'PY'
import detectron2
import detectron2._C
import torch
assert detectron2.__version__ == "0.6"
assert torch.cuda.is_available()
print(
    "source semantic runtime ready:",
    f"detectron2={detectron2.__version__}",
    f"torch={torch.__version__}",
    f"cuda={torch.version.cuda}",
    f"gpu={torch.cuda.get_device_name(0)}",
)
PY
