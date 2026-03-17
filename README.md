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
   export LD_LIBRARY_PATH=/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:/lib:/usr/local/cuda-12.1/lib64:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu
   export LD_PRELOAD="/lib/x86_64-linux-gnu/libffi.so.7 /lib/x86_64-linux-gnu/libtiff.so.5"
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

### Drinking Demo
1. Run the arm controller server on the NUC:
   - ssh to the NUC: `ssh emprise@192.168.1.3` with lab password
   - run the controller server:
       - `conda activate controller`
       - `cd /home/emprise/RAMMP/src/rammp/control/robot_controller`
       - `python arm_server.py`
2. Run bulldog on the NUC:
   - ssh to the NUC: `ssh emprise@192.168.1.3` with lab password
   - run bulldog:
       - `conda activate controller`
       - `cd /home/emprise/RAMMP/src/rammp/integration`
       - `./run_bulldog.sh`
2. Run a roscore on the compute system: `roscore`
3. Launch the roslaunch on compute system for camera / visualization / publish tfs:
    - `conda activate compute`
    - `cd /home/rammp/rammp_ws/src/RAMMP/launch`
    - `roslaunch robot.launch`
4. Run the watchdog / transfer button listener / other safety stuff:
    - `conda activate compute`
    - `cd /home/rammp/rammp_ws/src/RAMMP/src/rammp/integration`
    - `./launch_robot.sh`
5. Run the drinking demo:
    - `conda activate compute`
    - `source ~/rammp_ws/devel/setup.bash`
    - `cd /home/rammp/rammp_ws/src/RAMMP/src/rammp/integration`
    - `python run.py --user rammp --run_on_robot`
   
### Moving the robot to preset configurations

You can move the robot to preset configurations by running:
- `cd /home/rammp/rammp_ws/src/RAMMP/src/rammp/control/robot_controller/preset_actions`
- `python retract.py`
