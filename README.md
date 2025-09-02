# secaro_ros2_converter

# 使用方法
ワークスペースを作成する
```
mkdir -p ros2_ws/src
```
このパッケージをワークスペースにクローンする
```
cd ros2_ws/src
git clone https://github.com/LifeTechRobotics/secaro_ros2_converter.git
```
ビルドする
```
cd ..
colcon build --symlink-install
```

**secaro_ros2_converter/launch/wifi.launch.py**を編集する。
以下は**wifi.launch.py**のコピペ
- **enable_log**はTrueでデバッグ出力が有効になる。
- **cmd_vel_topic_name**はTwist型のサブスクライバーのトピック名を決める
- **wheel_base**はロボットの左右間の車輪の距離を決める
```py:wifi.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('secaro_ros2_converter')

    # Declare the namespace argument (it can be provided when launching)

    return LaunchDescription([
        Node(
            package='secaro_ros2_converter',
            executable='wifi',
            parameters= [
                {'enable_log': True},
                {'cmd_vel_topic_name': '/cmd_vel'},
                {'wheel_base': 1.0}
            ]
        )
    ])
```

# 実行
```
ros2 launch secaro_ros2_converter wifi.launch.py
```