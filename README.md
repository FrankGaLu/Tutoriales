[![CI](https://github.com/FrankGaLu/Tutoriales/actions/workflows/ci.yml/badge.svg)](https://github.com/FrankGaLu/Tutoriales/actions/workflows/ci.yml)
[![Licencia: MIT](https://img.shields.io/badge/licencia-MIT-2f855a.svg)](LICENSE)

# Tutoriales técnicos

Colección bilingüe de guías prácticas para aprender herramientas de robótica,
mensajería y automatización desde ejemplos pequeños y reproducibles.

## Contenido

| Tema | Español | English | Qué encontrarás |
| --- | --- | --- | --- |
| ROS 2 Jazzy | [Guía ROS 2](ROS2_esp.md) | [ROS 2 guide](ROS2_eng.md) | Workspace, nodos publisher/subscriber, URDF, RViz2 y launch files |
| MQTT | [Guía MQTT](MQTT_esp.md) | [MQTT guide](MQTT_eng.md) | Mosquitto, `mosquitto_pub/sub` y clientes Paho en Python |
| Bash | [Guía Bash](bash_esp.md) | [Bash guide](bash_eng.md) | Terminal, archivos, procesos, permisos y búsquedas |
| Graphviz | [Guía Graphviz](graphviz_tutorial_esp.md) | [Graphviz guide](graphviz_tutorial_eng.md) | DOT, atributos, subgrafos y renderizado |

También hay instaladores opcionales para Ubuntu: [Mosquitto](install_mosquitto.sh)
y [ROS 2](install_ros2.sh). Revísalos antes de ejecutarlos, especialmente si el
equipo está conectado a una red compartida.

## Ruta rápida

### Antes de empezar

- Los tutoriales de ROS 2 y MQTT están orientados a Ubuntu 24.04 LTS.
- En Windows, usa WSL 2 con Ubuntu para ejecutar los comandos Bash, ROS 2 y
	Mosquitto. Graphviz también tiene instalador nativo para Windows.
- Los instaladores `.sh` modifican el sistema y requieren `sudo`; léelos antes
	de ejecutarlos y empieza siempre con `--dry-run`.

1. Clona el repositorio y entra en él:

	```bash
	git clone https://github.com/FrankGaLu/Tutoriales.git
	cd Tutoriales
	```

2. Elige una guía en la tabla anterior. Las instrucciones están escritas para
	Ubuntu 24.04 cuando se indica explícitamente.
3. Para los ejemplos Python, usa Python 3.10 o posterior y un entorno virtual:

	```bash
	python3 -m venv .venv
	source .venv/bin/activate
	python -m pip install --upgrade pip
	python -m pip install paho-mqtt  # Solo para los ejemplos MQTT
	```

4. Para probar la instalación sin modificar el sistema, ejecuta el modo de
	simulación de los instaladores:

	```bash
	bash install_mosquitto.sh --dry-run
	bash install_ros2.sh --dry-run
	```

### Orden recomendado para una primera práctica

1. [Bash](bash_esp.md): navegación, archivos y permisos.
2. [Graphviz](graphviz_tutorial_esp.md): genera tu primer diagrama desde
   [graph.dot](graph.dot).
3. [MQTT](MQTT_esp.md): publica y recibe mensajes localmente.
4. [ROS 2](ROS2_esp.md): crea un workspace, nodos y un modelo URDF.

## Validación local

La integración continua comprueba los bloques Python, la sintaxis Bash y los
enlaces Markdown. Para reproducir las comprobaciones Python y Bash localmente:

```bash
python3 - <<'PY'
import glob
import py_compile
import tempfile

with tempfile.TemporaryDirectory() as directory:
	for path in glob.glob('**/*.md', recursive=True):
		text = open(path, encoding='utf-8').read()
		fence = '`' * 3 + 'python'
		for index, block in enumerate(text.split(fence)[1:], 1):
			source = block.split('```', 1)[0].lstrip('\n')
			target = f'{directory}/block_{index}.py'
			open(target, 'w', encoding='utf-8').write(source)
			py_compile.compile(target, doraise=True)
PY
bash -n install_mosquitto.sh install_ros2.sh
```

## Contribuir

Consulta [CONTRIBUTING.md](CONTRIBUTING.md) para el flujo de trabajo, las
convenciones de ejemplos y las comprobaciones previas a un Pull Request.

## Citar este repositorio

Si utilizas estas guías en una clase, proyecto o publicación, consulta
[CITATION.cff](CITATION.cff) o usa esta referencia BibTeX:

```bibtex
@misc{garcia_luna_tutoriales,
  author       = {Garcia-Luna, Francesco},
  title        = {Tutoriales técnicos},
  year         = {2026},
  publisher    = {GitHub},
  howpublished = {\url{https://github.com/FrankGaLu/Tutoriales}},
  note         = {Accedido el 2026-09-03}
}
```

## Licencia

El contenido se distribuye bajo la [licencia MIT](LICENSE). Las herramientas y
dependencias de terceros conservan sus propias licencias y marcas.
