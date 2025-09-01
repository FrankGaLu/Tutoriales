# Basic Terminal Tutorial

***Author: Dr. Francesco Garcia-Luna***

---

## Index
1. [Introduction](#1-introduction)
2. [Opening the Terminal](#2-opening-the-terminal)
3. [Basic Commands](#3-basic-commands)
    - [pwd](#pwd)
    - [ls](#ls)
    - [ls -a](#ls-a)
    - [cd](#cd)
    - [mkdir](#mkdir)
    - [rm](#rm)
    - [rm -r](#rm-r)
4. [File Operations](#4-file-operations)
    - [cp](#cp)
    - [mv](#mv)
    - [cat](#cat)
    - [nano](#nano)
    - [touch](#touch)
5. [System Information](#5-system-information)
    - [uname](#uname)
    - [top](#top)
    - [df](#df)
    - [kill](#kill)

---

## 1. Introduction
This tutorial will guide you through the basic usage of the terminal. The terminal is a powerful tool that allows you to interact with your system using text commands.

---

## 2. Opening the Terminal
To open the terminal, you can use the keyboard shortcut `Ctrl + Alt + T` or search for "Terminal" in the application menu.

---

## 3. Basic Commands

### `pwd`
The `pwd` command stands for "print working directory". It displays the current directory you are in.
```sh
pwd
```

### `ls`
The `ls` command lists the files and directories in the current directory.
```sh
ls
```

### `ls -a`
The `ls -a` command lists all files and directories in the current directory, including hidden ones (those starting with a dot `.`).
```sh
ls -a
```

### `cd`
The `cd` command is used to change directories.
```sh
cd /path/to/directory
```

### `mkdir`
The `mkdir` command creates a new directory.
```sh
mkdir new_directory
```

### `rm`
The `rm` command removes files or directories.
```sh
rm file_name
```

### `rm -r`
The `rm -r` command removes directories and their contents recursively.
```sh
rm -r directory_name
```

---

## 4. File Operations

### `cp`
The `cp` command copies files or directories.
```sh
cp source_file destination_file
```

### `mv`
The `mv` command moves or renames files or directories.
```sh
mv old_name new_name
```

### `cat`
The `cat` command displays the contents of a file.
```sh
cat file_name
```

### `nano`
The `nano` command opens a simple text editor in the terminal.
```sh
nano file_name
```

### `touch`
The `touch` command creates an empty file.
```sh
touch file_name
```

---

## 5. System Information

### `uname`
The `uname` command displays system information.
```sh
uname -a
```

### `top`
The `top` command shows the running processes and system resource usage.
```sh
top
```

### `df`
The `df` command displays disk space usage.
```sh
df -h
```

### `kill`
The `kill` command sends a signal to a process to terminate it.
```sh
kill process_id
```

---

## Advanced Examples

### `grep`
The `grep` command searches for patterns in files using regular expressions.
```sh
grep "error" logfile.txt
```
This is useful for finding specific text in large files or logs.

### `find`
The `find` command searches for files in a directory hierarchy based on various criteria.
```sh
find /home -name "*.pdf" -type f
```
It helps in locating files by name, type, size, etc.

### `chmod`
The `chmod` command changes file permissions.
```sh
chmod 755 script.sh
```
Permissions are represented in octal notation for owner, group, and others.

---

## Theoretical Background

The terminal, or command-line interface (CLI), is a text-based way to interact with the operating system. It allows users to execute commands, run scripts, and manage files more efficiently than graphical interfaces for certain tasks. Commands are processed by the shell (e.g., Bash), which interprets and executes them. The shell provides features like piping (`|`), redirection (`>`, `>>`), and scripting for automation.

---

## Conclusion

This tutorial has covered essential basic commands for using the terminal effectively, including navigation, file manipulation, and system monitoring. Practice these commands to become proficient in terminal usage.
