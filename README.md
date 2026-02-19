# RAMMP

RAMMP can be deployed in one of the following configurations:

- **Two-machine setup**
  - **GPU Machine (GM):** Runs compute-intensive components.
  - **Controller Machine (CM):** Handles controllers and emergency stops.
- **Single-machine setup (SM):** Runs all components on a single system.

---

## Requirements

- **Operating System:** Ubuntu 20.04  
- **Python:** 3.10 or later  
- **ROS:** Noetic (including `rospy`)  

---

## Installation

The system uses two Conda environments:

- `compute` – for compute-intensive components  
- `controller` – for control and hardware interfaces  

In a **two-machine setup**, these environments typically live on separate machines (GM and CM).  
In a **single-machine setup**, both environments can exist on the same machine.

---

## Compute Setup

1. Create the Conda environment:

   ```bash
   conda create -n compute python=3.10
   ```

2. Activate the environment:

   ```bash
   conda activate compute
   ```

3. Install PyTorch (CUDA 12.1):

   ```bash
   conda install pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 \
       pytorch-cuda=12.1 numpy=1.26.4 numpy-base=1.26.4 \
       -c pytorch -c nvidia -c defaults
   ```

4. Install PyTorch3D:

   ```bash
   python -m pip install pytorch3d \
       -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py310_cu121_pyt210/download.html
   ```

5. Clone the RAMMP repository:

   ```bash
   git clone https://github.com/empriselab/RAMMP
   cd RAMMP
   ```

6. Install additional dependencies:

   ```bash
   pip install --no-build-isolation chumpy
   ```

7. Install RAMMP (full installation):

   ```bash
   pip install -e ".[full]"
   ```

8. Install Pinocchio:

   ```bash
   conda install -c conda-forge "pinocchio=3.1.*"
   ```

9. Add the following to your `~/.bashrc`:

   ```bash
   export LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7
   ```

   Then reload:

   ```bash
   source ~/.bashrc
   ```

---

## Controller Setup

1. Create the Conda environment:

   ```bash
   conda create -n controller python=3.10
   ```

2. Activate the environment:

   ```bash
   conda activate controller
   ```

3. Install the Kortex API (after downloading the wheel file):

   ```bash
   python -m pip install kortex_api-2.6.0.post3-py3-none-any.whl
   ```

4. Clone the RAMMP repository:

   ```bash
   git clone https://github.com/empriselab/RAMMP
   cd RAMMP
   ```

5. Install RAMMP (controller-only installation):

   ```bash
   pip install -e ".[controller]"
   ```

6. Install Pinocchio:

   ```bash
   conda install -c conda-forge "pinocchio=3.1.*"
   ```

---

## Running

_Work in progress._
