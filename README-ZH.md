# robodriver-robot-galaxealite-aio-ros2
## 快速开始
### 克隆代码仓库
```
git clone --recurse-submodules https://github.com/BAAI-EI-DATA/robodriver-robot-galaxealite-aio-ros2.git && cd robodriver-robot-galaxealite-aio-ros2
```

### 安装 uv 工具
```
pip install uv
```

### 为 galaxea ros 节点创建并激活虚拟环境
```
uv venv .venv -p 3.10
source .venv/bin/activate
```

### 安装依赖
```
uv pip install -e .[hardware]
```

### 将项目安装到 Robodriver
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
cd /path/to/your/robodriver-robot-galaxealite-aio-ros2
uv pip install -e .
source .venv/bin/activate
```

### 配置 dataflow.yml 文件
```
cd /path/to/your/robodriver-robot-galaxealite-aio-ros2
```
打开 dataflow.yml 文件，修改 VIRTUAL_ENV 路径，以及 DEVICE_SERIAL、ARM_IP 和 ARM_PORT 配置项。

### 参数说明：
- VIRTUAL_ENV: galaxea ros 节点使用的虚拟环境路径
- DEVICE_SERIAL: RealSense 相机的序列号
- ARM_IP: 机械臂的 IP 地址
- ARM_PORT: 机械臂的端口号

## 开始采集数据
### 激活环境
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
```

### 启动 RoboXStudio
```
cd /path/to/your/RoboDriver
source .venv/bin/activate
python robodriver/scripts/run.py \
  --robot.type=galaxealite-aio-ros2 
```

## 问题修复
1. 视频回放重新运行失败： 
   编辑 `RoboDriver/robodriver/core/coordinator.py` 文件，将 `visual_worker(mode="distant")` 修改为 `mode="local"`。

2. 启动时出现 OpenCV cvShowImage 错误（执行 `python robodriver/scripts/run.py --robot.type=galaxealite-aio-ros2` 时）： 
   注释掉 `robodriver/scripts/run.py` 文件中的 `cv2.imshow(key, img)` 和 `cv2.waitKey(1)` 这两行代码。

## 数据说明
```
