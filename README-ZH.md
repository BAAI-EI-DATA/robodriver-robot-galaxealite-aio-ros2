# robodriver-robot-galaxealite-aio-ros2
## 快速开始
### 若未申请具身平台和安装端侧服务，请先执行
1.具身平台：https://ei2data.baai.ac.cn/home
2.端侧服务：https://github.com/FlagOpen/RoboDriver-Server.git

### 接入要求
1.硬件要求
https://jwolpxeehx.feishu.cn/wiki/LYcNwC2rBirg4Dk9CoScemx3n8f?from=from_copylink
2.galaxea遥操作正常
3.准备的主机安装ROS2，可以接收到galaxea话题信息。
将主机和galaxea的主控连在同局域网内（推荐网线连接）
设置ROS export ROS_DOMAIN_ID=1

### 克隆代码仓库
1.克隆 RoboDriver 仓库，若已克隆，可跳过。
```
git clone https://github.com/FlagOpen/RoboDriver.git
```

1.克隆 galaxea 仓库
```
git clone https://github.com/BAAI-EI-DATA/robodriver-robot-galaxealite-aio-ros2.git
```

### 通过安装 miniconda 创建虚拟环境
1. 安装 Miniconda（若未安装，可选）, 下载 Miniconda3（适配 Linux/macOS）
‵‵‵
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
source ~/.bashrc
```

2. 安装依赖
创建 Python 3.10 的 conda 环境, 激活环境
```
conda create -n robodriver python=3.10 -y
conda activate robodriver
```

安装 RoboDriver 核心依赖
```
cd /path/to/your/RoboDriver  
conda activate robodriver
pip install -e .
```

安装 galaxea 机器人相关依赖
```
cd /path/to/your/robodriver-robot-galaxealite-aio-ros2
conda activate robodriver
pip install -e .[hardware]
```

### 根据机器的实际话题情况，配置 node.py 文件
开发者需知：采集哪些话题和消息类型。
该脚本实现了 GALAXEALITE 机器人的多话题数据同步订阅、数据格式转换、关节指令发布 以及 图像数据接收解析 核心功能，主要用于机器人臂部、夹爪、躯干的运动指令交互与状态反馈，适配 ROS 2 Humble/Iron 版本（Python 3.10）。
若机器人实际发布 / 订阅的话题名称与脚本默认值不一致，可按以下方式修改（核心修改位置：节点初始化函数 __init__ 中的订阅 / 发布器定义）。
1. 发布器话题（运动指令输出）
默认发布话题列表：
发布器变量	           默认话题	                                    功能
publisher_left_arm	/motion_target/target_joint_state_arm_left	左臂关节目标值发布
publisher_right_arm	/motion_target/target_joint_state_arm_right	右臂关节目标值发布
publisher_left_gripper	/motion_target/target_position_gripper_left	左夹爪位置目标值发布
publisher_right_gripper	/motion_target/target_position_gripper_right	右夹爪位置目标值发布
publisher_state_torso	/motion_target/target_joint_state_torso	躯干关节目标值发布
2. 跟随反馈订阅话题（机器人状态输入）
默认订阅话题列表（_init_message_follow_filters 函数）：
订阅器变量	     默认话题	               功能
sub_arm_left	/hdas/feedback_arm_left	左臂关节反馈订阅
sub_arm_right	/hdas/feedback_arm_right	右臂关节反馈订阅
sub_gripper_left	/hdas/feedback_gripper_left	左夹爪反馈订阅
sub_gripper_right	/hdas/feedback_gripper_right	右夹爪反馈订阅
sub_torso	/hdas/feedback_torso	躯干关节反馈订阅
3. 主运动指令订阅话题（目标指令输入）
默认订阅话题列表（_init_message_main_filters 函数）：
订阅器变量	      默认话题	                                    功能
sub_joint_left	/motion_target/target_joint_state_arm_left	左臂关节目标值订阅
sub_joint_right	/motion_target/target_joint_state_arm_right	右臂关节目标值订阅
sub_joint_torso	/motion_target/target_joint_state_torso	躯干关节目标值订阅
sub_pose_left	/motion_target/target_pose_arm_left	左臂位姿目标值订阅
sub_pose_right	/motion_target/target_pose_arm_right	右臂位姿目标值订阅
sub_torso	/motion_target/target_pose_torso	躯干位姿目标值订阅
sub_gripper_left	/motion_target/target_position_gripper_left	左夹爪位置目标值订阅
sub_gripper_right	/motion_target/target_position_gripper_right	右夹爪位置目标值订阅
4. 图像话题订阅（相机数据输入）
默认订阅话题列表（_init_image_message_filters 函数）：
订阅器变量	           默认话题	                                             功能
sub_camera_top_left	/hdas/camera_head/left_raw/image_raw_color/compressed	顶部左相机图像订阅
sub_camera_top_right	/hdas/camera_head/right_raw/image_raw_color/compressed	顶部右相机图像订阅
sub_camera_wrist_left	/hdas/camera_wrist_left/color/image_raw/compressed	左手腕相机图像订阅
sub_camera_wrist_right	/hdas/camera_wrist_right/color/image_raw/compressed	右手腕相机图像订阅
修改示例：将顶部左相机订阅话题改为 /my_robot/camera/top_left/compressed
5. 关键参数调整
1. QoS 配置
脚本默认定义了两种 QoS 配置（可靠传输 / 尽力传输），可根据网络环境调整：
可靠传输（默认用于发布器、关键反馈订阅）
self.qos = QoSProfile(
    durability=DurabilityPolicy.VOLATILE,  # 不持久化
    reliability=ReliabilityPolicy.RELIABLE, # 可靠传输（确保消息到达）
    history=HistoryPolicy.KEEP_LAST,        # 保留最后N条
    depth=10                                # 队列深度
)
尽力传输（用于非关键指令订阅）
self.qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, # 尽力传输（优先速度）
    depth=10
)
2. 同步参数
多话题同步的队列大小和时间容差（slop）可调整：
# 跟随反馈同步示例（原配置）
self.sync = ApproximateTimeSynchronizer(
    [sub_arm_left, sub_arm_right, sub_gripper_left, sub_gripper_right, sub_torso],
    queue_size=10,  # 队列大小（越大容错越高，内存占用越多）
    slop=0.1        # 时间容差（秒）：允许话题时间戳最大差值
)
3. 发布频率限制
默认限制发布频率为 30Hz（min_interval_ns = 1e9 / 30），可修改该值调整频率：
# 改为 10Hz
self.min_interval_ns = 1e9 / 10

### 配置 config.py
开发者需知：采集哪些动作和图像信息等。
该脚本基于 lerobot 框架定义了 GALAXEALITE 机器人（ROS 2 版本）的硬件配置模板，核心实现机器人关节电机、相机、麦克风等硬件的参数化配置，支持「主端（leader）」和「跟随端（follower）」双端电机配置分离，同时兼容角度 / 百分比等多种电机归一化模式，可直接对接 lerobot 框架的采集、控制逻辑。
示例：新增躯干第 4 关节（follower_motors）
原 follower_motors 中 torso 仅 3 个关节，新增 torso_joint_4
```
follower_motors: Dict[str, Motor] = field(
    default_factory=lambda norm_mode_body=norm_mode_body: {
        "follower_arms":{
            # ... 原有配置 ...
            "torso_joint_3":Motor(17, "sts3215", norm_mode_body),
            "torso_joint_4":Motor(18, "sts3215", norm_mode_body),  # 新增
        }
    }
)
```
视频录制开关：

use_videos: bool = False
若关闭，图像信息会采集滞后编码成视频, 需要更改lerobot源码：
进入 /path/to/your/miniconda3/envs/robodriver/lib/python3.10/site-packages/lerobot/datasets/pipeline_features.py
注释下面代码：
```
# 2. Apply filtering rules.
# if is_image and not use_videos:
#     continue
```

use_videos: bool = True
若打开，图像信息会采集时编码成视频，等待编码时长依赖电脑性能。


## 开始采集数据
### 激活环境
```
sudo systemctl start nginx
cd /path/to/your/RoboDriver
conda activate robodriver
```

### 启动 Galaxealite 话题
按机器本身的启动脚本启动。

### 启动 RoboDriver
```
sudo systemctl start nginx
python -m robodriver.scripts.run  --robot.type=galaxealite-aio-ros2 
```

### 进入具身一体化平台发布任务
https://ei2rmd.baai.ac.cn/userlogin

### 进入采集平台开始采集
http://localhost:5805/hmi/

## 回放采集数据
采集数据完成后，断开遥操作，点击回放。

## 问题修复
1. 视频回放重新运行失败： 
   编辑 `RoboDriver/robodriver/core/coordinator.py` 文件，将 `visual_worker(mode="distant")` 修改为 `mode="local"`。

2. 启动时出现 OpenCV cvShowImage 错误（执行 `python -m robodriver.scripts.run --robot.type=galaxealite-aio-ros2` 时）： 
   注释掉 `robodriver/scripts/run.py` 文件中的 `cv2.imshow(key, img)` 和 `cv2.waitKey(1)` 这两行代码。

3. 进入http://localhost:5805/hmi/失败：
sudo systemctl start nginx

4. 机器人响应超时：
检查网络、RoboDriver启动脚本

## 数据说明
数据默认按时间存储在/home/yourname/DoRobot文件夹下，
结构如下：
TaskName_TaskId/
├── audio  # 音频
│   └── chunk-000
│       ├── observation.audio.audio_left
│       │   ├── episode_000000.wav
│       │   ├── episode_000001.wav
│       │   └── episode_000002.wav
│       └── observation.audio.audio_right
│           ├── episode_000000.wav
│           ├── episode_000001.wav
│           └── episode_000002.wav
├── data  # 运动信息
│   └── chunk-000
│       ├── episode_000000.parquet
│       ├── episode_000001.parquet
│       └── episode_000002.parquet
├── depth  # 深度图像
│   └── chunk-000
│       ├── observation.images.image_depth_right
│       │   ├── episode_000000.avi
│       │   ├── episode_000001.avi
│       │   └── episode_000002.avi
│       └── observation.images.image_depth_top
│           ├── episode_000000.avi
│           ├── episode_000001.avi
│           └── episode_000002.avi
├── device # 设备本身信息
│   └── device_info.json  # Robot information
├── label  # 标注信息，标注后产生
│   └── data_annotation.json
├── meta  # 文本信息
│   ├── common_record.json  # Collection task infomation
│   ├── episodes.jsonl   # Each task description and frame length
│   ├── episodes_stats.jsonl   # Normalized statistics
│   ├── info.json   # Feature schema, frame rate, version, path template
│   ├── op_dataid.jsonl   # Machine Number
│   └── tasks.jsonl  
└── videos  # 可见光图像
    └── chunk-000
        ├── observation.images.image_left
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        ├── observation.images.image_left_tac_l
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        ├── observation.images.image_left_tac_r
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        ├── observation.images.image_right
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        ├── observation.images.image_right_tac_l
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        ├── observation.images.image_right_tac_r
        │   ├── episode_000000.mp4
        │   ├── episode_000001.mp4
        │   └── episode_000002.mp4
        └── observation.images.image_top
            ├── episode_000000.mp4
            ├── episode_000001.mp4
            └── episode_000002.mp4
```
