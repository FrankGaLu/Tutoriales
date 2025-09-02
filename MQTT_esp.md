# Documentación de MQTT
**Autor: Dr. Francesco Garcia Luna**

---

## ÍNDICE

1. [Instalación](#1-instalacion)
2. [Configuración](#2-configuracion)
3. [Suscribirse a un Tópico por Terminal](#3-suscribirse-a-un-topico-por-terminal)
4. [Publicar en un Tópico por Terminal](#4-publicar-en-un-topico-por-terminal)
5. [Publicar en un Tópico por Terminal (JSON)](#5-publicar-en-un-topico-por-terminal-json)
6. [Suscribirse a un Tópico (Python)](#6-suscribirse-a-un-topico-python)
7. [Publicar en un Tópico (Python)](#7-publicar-en-un-topico-python)
8. [Publicar en un Tópico (Python) (JSON)](#8-publicar-en-un-topico-python-json)
9. [Conclusión](#9-conclusion)

---

## Introducción

Este tutorial proporciona una guía completa para instalar y configurar el broker MQTT Mosquitto en Ubuntu 24.04. Cubre la configuración básica, configuración y ejemplos de uso para suscribirse y publicar mensajes por terminal y Python.

---

## 1. Instalación

### Instalación en Ubuntu 24.04

1. **Actualiza la lista de paquetes:**
    ```bash
    sudo apt update
    ```

2. **Instala el broker MQTT (Mosquitto):**
    ```bash
    sudo apt install mosquitto mosquitto-clients python3-paho-mqtt -y
    ```

3. **Inicia el servicio Mosquitto:**
    ```bash
    sudo systemctl start mosquitto
    ```

4. **Habilita Mosquitto para que inicie al arrancar el sistema:**
    ```bash
    sudo systemctl enable mosquitto
    ```

5. **Verifica la instalación:**
    ```bash
    mosquitto -v
    ```

### Inicio Automático

Para asegurarte de que el servicio Mosquitto se inicie automáticamente al arrancar el sistema, el siguiente comando ya se ejecutó durante la instalación:

```bash
sudo systemctl enable mosquitto
```

Puedes comprobar el estado del servicio Mosquitto con:

```bash
sudo systemctl status mosquitto
```

---

## 2. Configuración

El archivo de configuración de Mosquitto se encuentra en `/etc/mosquitto/mosquitto.conf`. El archivo de configuración por defecto está bien documentado y contiene muchos ejemplos útiles.

### Configuración Básica

El archivo de configuración por defecto está bien documentado y contiene muchos ejemplos útiles. Aquí tienes algunos ajustes básicos que puedes modificar:

1. **Puerto:**
    ```bash
    port 1883
    ```
    El puerto MQTT por defecto es 1883. Si deseas usar otro puerto, puedes cambiarlo aquí.

2. **Listener:**
    ```bash
    listener 1883
    ```

3. **Permitir Conexiones Anónimas:**
    ```bash
    allow_anonymous true
    ```
    Por defecto, se permiten conexiones anónimas. Si deseas requerir autenticación, cambia esto a `false`.

4. **Archivo de Contraseñas:**
    ```bash
    password_file /etc/mosquitto/passwd
    ```
    Si deseas requerir autenticación, puedes crear un archivo de contraseñas y especificarlo aquí.

---

## Teoría del Protocolo MQTT

MQTT (Message Queuing Telemetry Transport) es un protocolo ligero de publicación-suscripción diseñado para IoT y comunicaciones M2M. Usa un broker central para gestionar mensajes entre publicadores y suscriptores. Características clave incluyen niveles de Calidad de Servicio (QoS) (0: como máximo una vez, 1: al menos una vez, 2: exactamente una vez), mensajes retenidos, y Última Voluntad y Testamento (LWT) para manejar desconexiones.

## Ejemplos Avanzados

### Publicar con QoS

```bash
mosquitto_pub -h localhost -t "topico" -m "mensaje" -q 1
```

QoS 1 asegura que el mensaje se entregue al menos una vez.

### Mensajes Retenidos

```bash
mosquitto_pub -h localhost -t "topico" -m "mensaje retenido" -r
```

Los mensajes retenidos se almacenan por el broker y se envían a nuevos suscriptores inmediatamente al suscribirse.

### Última Voluntad y Testamento

```bash
mosquitto_pub -h localhost -t "topico" -m "cliente desconectado" --will-topic "estado" --will-payload "offline"
```

Esto establece un mensaje a publicar si el cliente se desconecta inesperadamente.

---

## 3. Suscribirse a un Tópico por Terminal

Para suscribirte a un tópico usando el cliente de línea de comandos de Mosquitto, usa el siguiente comando:

```bash
mosquitto_sub -h localhost -t "topico"
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT y `topico` por el tópico al que deseas suscribirte.

---

## 4. Publicar en un Tópico por Terminal

Para publicar un mensaje en un tópico usando el cliente de línea de comandos de Mosquitto, usa el siguiente comando:

```bash
mosquitto_pub -h localhost -t "topico" -m "mensaje"
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT, `topico` por el tópico al que deseas publicar y `mensaje` por el mensaje que deseas enviar.

---

## 5. Publicar en un Tópico por Terminal (JSON)

Para publicar un mensaje JSON en un tópico usando el cliente de línea de comandos de Mosquitto, usa el siguiente comando:

```bash
mosquitto_pub -h localhost -t "topico" -m '{"clave": "valor"}'
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT, `topico` por el tópico al que deseas publicar y `{"clave": "valor"}` por el mensaje JSON que deseas enviar.

---

## 6. Suscribirse a un Tópico (Python)

Para suscribirte a un tópico usando el cliente Paho MQTT en Python, usa el siguiente código:

```python
import paho.mqtt.client as mqtt

def on_connect(client: mqtt.Client, userdata: None, flags: dict, rc: int) -> None:
    print("Conectado con código de resultado " + str(rc))
    client.subscribe("topico")

def on_message(client: mqtt.Client, userdata: None, message: mqtt.MQTTMessage) -> None:
    print(message.topic+" " + str(message.payload))

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    client.loop_forever()
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT y `topico` por el tópico al que deseas suscribirte.

---

## 7. Publicar en un Tópico (Python)

Para publicar un mensaje en un tópico usando el cliente Paho MQTT en Python, usa el siguiente código:

```python
import paho.mqtt.client as mqtt

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.publish("topico", "mensaje")
    client.disconnect()
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT, `topico` por el tópico al que deseas publicar y `mensaje` por el mensaje que deseas enviar.

---

## 8. Publicar en un Tópico (Python) (JSON)

Para publicar un mensaje JSON en un tópico usando el cliente Paho MQTT en Python, usa el siguiente código:

```python
import paho.mqtt.client as mqtt

if __name__ == "__main__":
    client: mqtt.Client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.publish("topico", '{"clave": "valor"}')
    client.disconnect()
```

Reemplaza `localhost` por la dirección IP o el nombre del host del broker MQTT, `topico` por el tópico al que deseas publicar y `{"clave": "valor"}` por el mensaje JSON que deseas enviar.

---

## 9. Conclusión

En esta guía, cubrimos la instalación y configuración básica del broker Mosquitto MQTT en Ubuntu 24.04. También mostramos cómo suscribirse y publicar mensajes en tópicos usando el cliente de línea de comandos de Mosquitto y el cliente Paho MQTT en Python.

Para más información sobre Mosquitto y MQTT, consulta la documentación oficial:

- [Documentación de Mosquitto](https://mosquitto.org/documentation/)
- [MQTT Essentials](https://www.hivemq.com/mqtt-essentials/)
