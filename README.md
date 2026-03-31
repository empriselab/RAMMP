# RAMMP

## Requirements

- **OS:** Ubuntu 22.04  
- **Python:** 3.10+  
- **ROS 2:** Humble  

---

## Installation

### 1. Create and activate Conda environment

    conda create -n compute python=3.10
    conda activate compute

---

### 2. Install PyTorch (CUDA 12.1)

    conda install pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
        pytorch-cuda=12.1 numpy=1.26.4 numpy-base=1.26.4 \
        -c pytorch -c nvidia -c defaults

---

### 3. Install PyTorch3D

    pip install pytorch3d \
        -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py310_cu121_pyt210/download.html

---

### 4. Create ROS 2 workspace and clone repository

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone https://github.com/empriselab/RAMMP
    cd RAMMP

---

### 5. Install additional Python dependencies

    pip install --no-build-isolation chumpy

---

### 6. Install RAMMP

    pip install -e ".[full]"

---

### 7. Install Pinocchio

    conda install -c conda-forge "pinocchio=3.1.*"

---

### 8. Configure environment variables

Add the following to your `~/.bashrc`:

    export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib:/usr/local/cuda-12.1/lib64:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu
    export LD_PRELOAD="/lib/x86_64-linux-gnu/libffi.so.7 /lib/x86_64-linux-gnu/libtiff.so.5"

Then reload:

    source ~/.bashrc

---

### 9. Build ROS 2 workspace

    cd ~/ros2_ws
    colcon build
    source install/setup.bash

---

## Running

    ros2 launch drink_actions_test real.launch.py

---
