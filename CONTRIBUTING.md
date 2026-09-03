# Contribuir a Tutoriales

Este repositorio contiene guías Markdown y ejemplos dirigidos a estudiantes y
personas que aprenden por su cuenta.

## Flujo de trabajo

1. Crea una rama a partir de `main`.
2. Haz cambios pequeños y enfocados, con mensajes de commit descriptivos.
3. Ejecuta `bash -n install_mosquitto.sh install_ros2.sh` y la comprobación
   Python documentada en el [README](README.md).
4. Revisa los enlaces Markdown y abre un Pull Request describiendo el cambio,
   el entorno usado y cualquier limitación conocida.

## Criterios para nuevos ejemplos

- Mantén los ejemplos simples, autocontenidos y ejecutables.
- Declara la versión del sistema y las dependencias cuando sean relevantes.
- Evita comandos destructivos o accesibles desde Internet por defecto; explica
  sus implicaciones cuando sean necesarios.
- Añade imágenes bajo `ros2_tutorial_images/` o `docs/images/` y usa rutas relativas.
- Mantén las versiones española e inglesa sincronizadas cuando exista ambas.

## Licencia

Las contribuciones se publican bajo la [licencia MIT](LICENSE).
