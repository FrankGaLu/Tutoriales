# ROS2 Documentation

**Dr. Francesco Garcia Luna**

---

## INDEX

1. [Installation](#1-installation)
2. [Create a Workspace](#2-create-a-workspace)
3. [Create a Package](#3-create-a-package)
4. [Create a Publisher Node](#4-create-a-publisher-node)
5. [Create a Subscriber Node](#5-create-a-subscriber-node)

---

## 1. Installation
### 1.1 Prerequisites

Ensure your system is up to date:
```bash
sudo apt update && sudo apt upgrade
```

### 1.2 Install ROS 2 Packages

For more information, you can visit the official ROS 2 installation guide [here](https://docs.ros.org/en/humble/Installation.html).

### 1.3 Setup Sources

You will need to add the ROS 2 apt repository to your system. To do this, run the following commands:

```bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.4 Install ROS 2

Now you can install ROS 2 with the following command:

```bash
sudo apt update && sudo apt install ros-humble-desktop-full -y
sudo apt install python3-colcon-common-extensions
```

### 1.5 Environment Setup

To complete the installation, you will need to source the ROS 2 setup script. You can do this by running the following command:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

### 1.6 Verify Installation

To verify the installation, you can run the following command:

```bash
source ~/.bashrc
ros2 --version
```

---

## 2. Create a Workspace

### 2.1 Create a Workspace

To create a new workspace, you can run the following command:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
```

### 2.2 Source the Workspace

To source the workspace, you can run the following command:

```bash
source ~/ros2_ws/install/setup.bash
```

---

## 3. Create a Package

### 3.1 Create a Package

To create a new package, you can run the following command:

```bash
ros2 pkg create --build-type ament_python my_package --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
```

### 3.2 Build the Package

To build the package, you can run the following command:

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

---

## 4. Create a Publisher Node

### 4.1 Create a Node

In the `my_package` directory, create a new Python file called `my_publisher.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__('my_publisher')
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.2 Add the Node to the Package via setup.py

In the `my_package` directory, open the `setup.py` file and add the following line to the `entry_points` section:

```python
entry_points={
    'console_scripts': [
        'my_publisher = my_package.my_publisher:main',
    ],
},
```

Where `my_publisher` is the name of the executable and `my_package.my_publisher:main` is the Python file and function to run.

### 4.3 Build the Package

To build the package, you can run the following command:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4.4 Run the Node

To run the node, you can use the following command:

```bash
ros2 run my_package my_publisher
```

---

## 5. Create a Subscriber Node

### 5.1 Create a Node

In the `my_package` directory, create a new Python file called `my_subscriber.py` with the following content:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):

    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(String, 'my_topic', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5.2 Add the Node to the Package via setup.py

In the `my_package` directory, open the `setup.py` file and add the following line to the `entry_points` section:

```python
entry_points={
    'console_scripts': [
        'my_subscriber = my_package.my_subscriber:main',
    ],
},
```

Where `my_subscriber` is the name of the executable and `my_package.my_subscriber:main` is the Python file and function to run.

### 5.3 Build the Package

To build the package, you can run the following command:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 5.4 Run the Node

To run the node, you can use the following command:

```bash
ros2 run my_package my_subscriber
```

---
