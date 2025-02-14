# MQTT Documentation
**Author: Dr. Francesco Garcia Luna**

---

## INDEX

1. [Installation](#1-installation)
2. [Configuration](#2-configuration)
3. [Subscribe to a Topic via Terminal](#3-subscribe-to-a-topic-via-terminal)
4. [Publish to a Topic via Terminal](#4-publish-to-a-topic-via-terminal)
5. [Publish to a Topic via Terminal (JSON)](#5-publish-to-a-topic-via-terminal-json)
6. [Subscribe to a Topic (Python)](#6-subscribe-to-a-topic-python)
7. [Publish to a Topic (Python)](#7-publish-to-a-topic-python)
8. [Publish to a Topic (Python) (JSON)](#8-publish-to-a-topic-python-json)
9. [Conclusion](#9-conclusion)

---

## 1. Installation

### Installation on Ubuntu 22.04

1. **Update the package list:**
    ```bash
    sudo apt update
    ```

2. **Install the MQTT broker (Mosquitto):**
    ```bash
    sudo apt install mosquitto mosquitto-clients python3-paho-mqtt -y
    ```

3. **Start the Mosquitto service:**
    ```bash
    sudo systemctl start mosquitto
    ```

4. **Enable Mosquitto to start on boot:**
    ```bash
    sudo systemctl enable mosquitto
    ```

5. **Verify the installation:**
    ```bash
    mosquitto -v
    ```

### Automatic Startup

To ensure that the Mosquitto service starts automatically on system boot, the following command has already been executed during installation:

```bash
sudo systemctl enable mosquitto
```

You can check the status of the Mosquitto service with:

```bash
sudo systemctl status mosquitto
```

---

## 2. Configuration

The Mosquitto configuration file is located at `/etc/mosquitto/mosquitto.conf`. The default configuration file is well-documented and contains many useful examples.

### Basic Configuration

The default configuration file is well-documented and contains many useful examples. Here are some basic settings that you may want to adjust:

1. **Port:**
    ```bash
    port 1883
    ```
    The default MQTT port is 1883. If you want to use a different port, you can change it here.

2. **Listener:**
    ```bash
    listener 1883
    ```

3. **Allow Anonymous Connections:**
    ```bash
    allow_anonymous true
    ```
    By default, anonymous connections are allowed. If you want to require authentication, set this to `false`.

4. **Password File:**
    ```bash
    password_file /etc/mosquitto/passwd
    ```
    If you want to require authentication, you can create a password file and specify it here.

---

## 3. Subscribe to a Topic via Terminal

To subscribe to a topic using the Mosquitto command-line client, use the following command

```bash
mosquitto_sub -h localhost -t "topic"
```

Replace `localhost` with the IP address or hostname of the MQTT broker and `topic` with the topic you want to subscribe to.

---

## 4. Publish to a Topic via Terminal

To publish a message to a topic using the Mosquitto command-line client, use the following command:

```bash
mosquitto_pub -h localhost -t "topic" -m "message"
```

Replace `localhost` with the IP address or hostname of the MQTT broker, `topic` with the topic you want to publish to, and `message` with the message you want to publish.

---

## 5. Publish to a Topic via Terminal (JSON)

To publish a JSON message to a topic using the Mosquitto command-line client, use the following command

```bash
mosquitto_pub -h localhost -t "topic" -m '{"key": "value"}'
```

Replace `localhost` with the IP address or hostname of the MQTT broker, `topic` with the topic you want to publish to, and `{"key": "value"}` with the JSON message you want to publish.

---

## 6. Subscribe to a Topic (Python)

To subscribe to a topic using the Paho MQTT Python client, use the following code:

```python
import paho.mqtt.client as mqtt

def on_connect(client: mqtt.Client, userdata: None, flags: dict, rc: int) -> None:
    print("Connected with result code " + str(rc))
    client.subscribe("topic")

def on_message(client: mqtt.Client, userdata: None, message: mqtt.MQTTMessage) -> None:
    print(message.topic+" " + str(message.payload))

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()

```

Replace `localhost` with the IP address or hostname of the MQTT broker and `topic` with the topic you want to subscribe to.

---

## 7. Publish to a Topic (Python)

To publish a message to a topic using the Paho MQTT Python client, use the following code:

```python
import paho.mqtt.client as mqtt

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.publish("topic", "message")
    client.disconnect()

```

Replace `localhost` with the IP address or hostname of the MQTT broker, `topic` with the topic you want to publish to, and `message` with the message you want to publish.

---

## 8. Publish to a Topic (Python) (JSON)

To publish a JSON message to a topic using the Paho MQTT Python client, use the following code:

```python
import paho.mqtt.client as mqtt

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.publish("topic", '{"key": "value"}')
    client.disconnect()

```

Replace `localhost` with the IP address or hostname of the MQTT broker, `topic` with the topic you want to publish to, and `{"key": "value"}` with the JSON message you want to publish.

---

## 9. Conclusion

In this guide, we have covered the installation and basic configuration of the Mosquitto MQTT broker on Ubuntu 22.04. We have also shown how to subscribe to and publish messages to topics using the Mosquitto command-line client and the Paho MQTT Python client.

For more information on Mosquitto and MQTT, refer to the official documentation:

- [Mosquitto Documentation](https://mosquitto.org/documentation/)
- [MQTT Essentials](https://www.hivemq.com/mqtt-essentials/)
