# RAMMP

RAMMP can be deployed in one of the following configurations:

- Two-machine setup
  - GPU Machine (GM): Runs compute-intensive components
  - Controller Machine (CM): Handles controllers and emergency stops
- Single-machine setup (SM): Runs all components on a single system

## Requirements

- Operating System: Ubuntu 20.04
- Python: 3.10 or later
- ROS Noetic (including rospy)

## Installation

### GPU Machine (GM) or Single Machine (SM)

1. Create a Conda environment: `conda create -n rammp python=3.10`
2. Activate the environment: `conda activate rammp`
3. (SM only) Install the Kortex API (after downloading the wheel file): `python -m pip install kortex_api-2.6.0.post3-py3-none-any.whl`
4. Install PyTorch (CUDA 12.1): `conda install pytorch==2.1.0 torchvision==0.16.0 torchaudio==2.1.0 pytorch-cuda=12.1 -c pytorch -c nvidia`
5. Install PyTorch3D: `python -m pip install pytorch3d -f https://dl.fbaipublicfiles.com/pytorch3d/packaging/wheels/py310_cu121_pyt210/download.html`
6. Clone the RAMMP repository: `git clone https://github.com/empriselab/RAMMP`
7. Navigate into the repository: `cd RAMMP`
8. Install RAMMP (full installation): `pip install -e ".[full]"`
9. Install Pinocchio: `conda install -c conda-forge "pinocchio=3.1.*"`
10. Add the following to your ~/.bashrc and source: `export LD_PRELOAD=/lib/x86_64-linux-gnu/libffi.so.7`

### Controller Machine (CM)

1. Create a Conda environment: `conda create -n rammp python=3.10`
2. Activate the environment: `conda activate rammp`
3. Install the Kortex API (after downloading the wheel file): `python -m pip install kortex_api-2.6.0.post3-py3-none-any.whl`
4. Clone the RAMMP repository: `git clone https://github.com/empriselab/RAMMP`
5. Navigate into the repository: `cd RAMMP`
6. Install RAMMP: `pip install -e .`
7. Install Pinocchio: `conda install -c conda-forge "pinocchio=3.1.*"`

## Running

WIP
