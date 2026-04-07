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

### 10. Download DECA model files (robot mode only)

DECA head perception requires model files not included in the repository.

**Download GitHub-sourced files:**

    cd src/rammp/perception/head_perception/DECA/data
    wget https://github.com/YadiraF/DECA/raw/master/data/landmark_embedding.npy
    wget https://github.com/YadiraF/DECA/raw/master/data/head_template.obj
    wget https://github.com/YadiraF/DECA/raw/master/data/texture_data_256.npy
    wget https://github.com/YadiraF/DECA/raw/master/data/fixed_displacement_256.npy
    wget https://github.com/YadiraF/DECA/raw/master/data/uv_face_mask.png
    wget https://github.com/YadiraF/DECA/raw/master/data/uv_face_eye_mask.png
    wget https://github.com/YadiraF/DECA/raw/master/data/mean_texture.jpg

**`generic_model.pkl`:** Register at [flame.is.tue.mpg.de](https://flame.is.tue.mpg.de), run the provided `fetch_data.sh`, then move the file:

    mv data/FLAME2020/FLAME2020/generic_model.pkl data/

**`deca_model.tar`:** Download from [Google Drive](https://drive.google.com/file/d/1rp8kdyLPvErw2dTmqtjISRVvQLj6Yzje). Despite the extension it is a PyTorch checkpoint — do **not** extract it, just place it in `data/`.

**`resnet50_ft_weight.pkl`:** Download separately (face recognition model) and place in `data/`.

> **NumPy / chumpy compatibility:** The FLAME model was serialized with `chumpy`, which does not support NumPy >= 1.24. If you encounter `ImportError: cannot import name 'bool' from 'numpy'`, patch `chumpy/__init__.py` by replacing the offending import line with Python builtins equivalents.

---

## Running

    ros2 launch drink_actions_test real.launch.py

---
