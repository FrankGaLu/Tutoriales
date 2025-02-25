# Basic ROS2 Humble Tutorial

***Author: Dr. Francesco Garcia-Luna***

---

## Index

1. [Installation](#1-installation)
    - [Setup Sources](#11-setup-sources)
    - [Install ROS2 packages](#12-install-ros2-packages)
    - [Environment Setup](#13-environment-setup)
    - [ROS Domain ID](#14-ros-domain-id)
    - [ROS Localhost only](#15-ros-localhost-only)
    - [Colcon](#16-colcon)
    - [(Optional) Uninstall ROS2](#14-optional-uninstall-ros2)
2. [Create a ROS2 Workspace](#2-create-a-ros2-workspace)
3. [Create a ROS2 Package](#3-create-a-ros2-package)
    - [Build the ROS2 Package](#31-build-the-ros2-package)
4. [Write a Publisher Node](#4-write-a-publisher-node)
    - [Add dependencies to the package](#41-add-dependencies-to-the-package)
    - [Add an entrypoint](#42-add-an-entrypoint)
5. [Write a Subscriber Node](#5-write-a-subscriber-node)
    - [Add an entrypoint](#52-add-an-entrypoint)
6. [Run the ROS2 Nodes](#6-run-the-ros2-nodes)
7. [Create a Basic URDF Model](#7-create-a-basic-urdf-model)
8. [Visualize the URDF Model in RVIZ2](#8-visualize-the-urdf-model-in-rviz2)
9. [Create a Basic Launch File to visualize the URDF Model](#9-create-a-basic-launch-file-to-visualize-the-urdf-model)

---

## 1. Installation

For detailed installation instructions, please refer to the [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation.html).

### 1.1. Setup Sources

```sh
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.2. Install ROS2 packages

```sh
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop-full
```

### 1.3. Environment Setup

```sh
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.4. ROS Domain ID

The `ROS_DOMAIN_ID` environment variable is used to isolate ROS 2 networks. It is recommended to set it to a unique value for each network. For example, if you are running multiple ROS 2 networks on the same machine, you can set `ROS_DOMAIN_ID` to different values for each network.

```sh
echo "export ROS_DOMAIN_ID=<your_domain_id>" >> ~/.bashrc
source ~/.bashrc
```

### 1.5. ROS Localhost only

To restrict ROS 2 to use only the localhost interface, you can set the `ROS_LOCALHOST_ONLY` environment variable to `1`.

```sh
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
source ~/.bashrc
```

### 1.6. Colcon

Colcon is a command-line tool to build ROS 2 packages. You can install it using the following command:

```sh
sudo apt install python3-colcon-common-extensions
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
source ~/.bashrc
```

### (Optional) Uninstall ROS2

```sh
sudo apt remove ~nros-humble-* && sudo apt autoremove
sudo rm /etc/apt/sources.list.d/ros2.list
sudo apt update
sudo apt autoremove
sudo apt upgrade
```

---

## 2. Create a ROS2 Workspace

To create a new ROS 2 workspace, you can use the following commands:

```sh
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 3. Create a ROS2 Package

To create a new ROS 2 package, you can use the following command:

```sh
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake <package_name>
```

### 3.1. Build the ROS2 Package

To build the ROS 2 package, you can use the following command:

```sh
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

---

## 4. Write a Publisher Node

To write a simple publisher node, you can use the following Python code:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.1. Add dependencies to the package

A dependency is a package that your package depends on. To add dependencies to the ROS 2 package, you can edit the `package.xml` file in the package directory.

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

### 4.2. Add an entrypoint

An entrypoint is a Python script that is executed when the package is run. To add an entrypoint to the ROS 2 package, you can edit the `setup.py` file in the package directory.

```python
entry_points={
    'console_scripts': [
        'talker = <package_name>.<script_name>:main',
    ],
},
```

---

## 5. Write a Subscriber Node

To write a simple subscriber node, you can use the following Python code:

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.2. Add an entrypoint

To add an entrypoint to the ROS 2 package, you can edit the `setup.py` file in the package directory.

```python
entry_points={
    'console_scripts': [
        'talker = <package_name>.<script_name>:main',
        'listener = py_pubsub.subscriber_member_function:main',
    ],
},
```

---

## 6. Run the ROS2 Nodes

To run the ROS 2 nodes, you can use the following commands:

```sh
cd ~/ros2_ws
source install/setup.bash
ros2 run <package_name> <script_name>
```

---

## 7. Create a Basic URDF Model

To create a basic URDF model, you can use the following XML code:

```xml
<?xml version="1.0"?>
<robot name="myfirst">
    <link name="base_link">
        <visual>
            <geometry>
                <cylinder length="0.6" radius="0.2"/>
            </geometry>
        </visual>
    </link>
</robot>
```

It is recomended to be saved in a directory called `urdf` inside the package directory.

Also, some modifications need to be done in the `setup.py` file, by adding the following lines:

```python
import os
from glob import glob

data_files = [
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
]
```

---

## 8. Visualize the URDF Model in RVIZ2

In order to visualize an URDF Model in RVIZ2, first you need to open RVIZ2 and create a config file in the `rviz2` directory of the package. 

```bash
rviz2
```

![rviz2](ros2_tutorial_images/rviz2_main.png)

Then, you can add a `RobotModel` display by selecting the `Add` button and selecting `RobotModel`.

![rviz2_window](ros2_tutorial_images/rviz2_window.png)

Change the `Fixed Frame` in the `Global Options` from `map` to `base_link`. Now, change the `Description Topic` in the `RobotModel` display to `/robot_description`.

Now, save the configuration file in the `rviz2` directory of the package by clicking on the `Save Config As` button in the `File` menu.

Finally, add the following lines to the `setup.py` file:

```python
data_files = [
    (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
]
```

In order to visualize the URDF Model in RVIZ2, you can use the following command:

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat src/<package_name>/urdf/<model_name>.urdf)"
```

![rviz2_urdf](ros2_tutorial_images/rviz2_final.png)

---

## 9. Create a Basic Launch File to visualize the URDF Model

To create a basic launch file to visualize the URDF Model in RVIZ2, you can use the following XML code:

```python
import os
import launch
from launch import LaunchDescription
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    pruebas_path: str = get_package_share_directory('<package_name>')
    urdf_filename: str = os.path.join(pruebas_path, 'urdf', '<model_name>.urdf')
    
    if not os.path.exists(urdf_filename):
            raise FileNotFoundError(f'El archivo URDF no se encontró en: {urdf_filename}')

    with open(urdf_filename, 'r') as infp:
        robot_description: str = infp.read()

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pruebas_path, 'rviz', '<rviz_config_name>.rviz')]
        )
    ])
```

where `<model_name>` is the name of the URDF model file, and `<rviz_config_name>` is the name of the RVIZ2 configuration file, and `<package_name>` is the name of the ROS2 package. And save the file as `display.launch.py` for example, in the `launch` directory of the package.

Then, you need to add the following lines to the `setup.py` file:

```python
data_files = [
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
]
```

And, the next lines to the `package.xml` file:

```xml
<export>
    <launch_file>display.launch.py</launch_file>
</export>
```

Finally, compile and source the workspace, and run the launch file:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch <package_name> display.launch.py
```
