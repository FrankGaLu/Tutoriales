# Tutorial Básico de Terminal

***Autor: Dr. Francesco Garcia-Luna***

---

## Índice
1. [Introducción](#1-introduccion)
2. [Abrir la Terminal](#2-abrir-la-terminal)
3. [Comandos Básicos](#3-comandos-basicos)
    - [pwd](#pwd)
    - [ls](#ls)
    - [ls -a](#ls-a)
    - [cd](#cd)
    - [mkdir](#mkdir)
    - [rm](#rm)
    - [rm -r](#rm-r)
4. [Operaciones con Archivos](#4-operaciones-con-archivos)
    - [cp](#cp)
    - [mv](#mv)
    - [cat](#cat)
    - [nano](#nano)
    - [touch](#touch)
5. [Información del Sistema](#5-informacion-del-sistema)
    - [uname](#uname)
    - [top](#top)
    - [df](#df)
    - [kill](#kill)

---

## 1. Introducción

Este tutorial te guiará a través del uso básico de la terminal. La terminal es una herramienta poderosa que te permite interactuar con tu sistema usando comandos de texto.

---

## 2. Abrir la Terminal

Para abrir la terminal, puedes usar el atajo de teclado `Ctrl + Alt + T` o buscar "Terminal" en el menú de aplicaciones.

---

## 3. Comandos Básicos

### `pwd`
El comando `pwd` significa "print working directory". Muestra el directorio actual en el que te encuentras.
```sh
pwd
```

### `ls`
El comando `ls` lista los archivos y directorios en el directorio actual.
```sh
ls
```

### `ls -a`
El comando `ls -a` lista todos los archivos y directorios, incluyendo los ocultos (los que empiezan con un punto `.`).
```sh
ls -a
```

### `cd`
El comando `cd` se usa para cambiar de directorio.
```sh
cd /ruta/al/directorio
```

### `mkdir`
El comando `mkdir` crea un nuevo directorio.
```sh
mkdir nuevo_directorio
```

### `rm`
El comando `rm` elimina archivos o directorios.
```sh
rm nombre_archivo
```

### `rm -r`
El comando `rm -r` elimina directorios y su contenido de forma recursiva.
```sh
rm -r nombre_directorio
```

---

## 4. Operaciones con Archivos

### `cp`
El comando `cp` copia archivos o directorios.
```sh
cp archivo_origen archivo_destino
```

### `mv`
El comando `mv` mueve o renombra archivos o directorios.
```sh
mv nombre_viejo nombre_nuevo
```

### `cat`
El comando `cat` muestra el contenido de un archivo.
```sh
cat nombre_archivo
```

### `nano`
El comando `nano` abre un editor de texto simple en la terminal.
```sh
nano nombre_archivo
```

### `touch`
El comando `touch` crea un archivo vacío.
```sh
touch nombre_archivo
```

---

## 5. Información del Sistema

### `uname`
El comando `uname` muestra información del sistema.
```sh
uname -a
```

### `top`
El comando `top` muestra los procesos en ejecución y el uso de recursos del sistema.
```sh
top
```

### `df`
El comando `df` muestra el uso del espacio en disco.
```sh
df -h
```

### `kill`
El comando `kill` envía una señal a un proceso para terminarlo.
```sh
kill id_proceso
```
