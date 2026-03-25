#!/bin/bash

# Force NVIDIA GPU (RTX 5070) via PRIME offload; use Vulkan renderer for best performance.
__NV_PRIME_RENDER_OFFLOAD=1 \
__NV_PRIME_RENDER_OFFLOAD_PROVIDER=NVIDIA-G0 \
VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  ./CARLA/CarlaUE4.sh -RenderOffScreen -preferNvidia -quality-level=Low &
