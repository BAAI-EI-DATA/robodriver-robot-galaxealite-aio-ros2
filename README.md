# robodriver-robot-galaxealite-aio-ros2
## Get Start
Clone the repository
```
git clone --recurse-submodules https://github.com/BAAI-EI-DATA/robodriver-robot-galaxealite-aio-ros2.git && cd robodriver-robot-galaxealite-aio-ros2
```
Install uv:
```
pip install uv
```
Create and activate a virtual environment for galaxea ros node:
```
uv venv .venv -p 3.10
source .venv/bin/activate
```
Install dependencies
```
uv pip install -e .[hardware]
```
Install the project to Robodriver
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
cd /path/to/your/robodriver-robot-galaxealite-aio-ros2
uv pip install -e .
source .venv/bin/activate
```
Configure the dataflow.yml
```
cd /path/to/your/robodriver-robot-galaxealite-aio-ros2
```
Open the dataflow.yml file, then modify the VIRTUAL_ENV path, as well as DEVICE_SERIAL, ARM_IP, and ARM_PORT. 
### Parameter explanation:

- VIRTUAL_ENV: The path of the virtual environment used by the galaxea ros node
- DEVICE_SERIAL: Serial number of the RealSense camera
- ARM_IP: IP of the robotic arm
- ARM_PORT: PORT of the robotic arm

## Start collecting
Activate environment
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
```

```
Launch RoboXStudio
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
python robodriver/scripts/run.py \
  --robot.type=galaxealite-aio-ros2 
```

## Bug Fixes
1. Rerun video replay failure:  
   Edit `RoboDriver/robodriver/core/coordinator.py`, change `visual_worker(mode="distant")` to `mode="local"`.

2. OpenCV cvShowImage error on launch (`python robodriver/scripts/run.py --robot.type=galaxealite-aio-ros2`):  
   Comment out `cv2.imshow(key, img)` and `cv2.waitKey(1)` in `robodriver/scripts/run.py`.


## Data Information
