# rosshell

 Two simple nodes that can be used as mediators for shell commands.

    Author: André Dietrich
    License: BSD
    Source: https://github.com/andre-dietrich/rosshell

## 1. Overview

This package provides the two nodes, that enable to run and interact with
different non-ros-programs. rosshell can be used to start a program or any kind
of shell-command. If some kind of interaction with the programs is required, use
rosshellX to receive and send messages from stdin, stdout, and stderr.

## 2. Usage

The required commands are always defined as a string:

```bash
$ rosrun rosshell rosshell.py "command"
```

```bash
$ rosrun rosshell rosshellX.py "command"
```

have a look at the examples or at the included launch-files.

## 3. Examples

### 3.1 rosshell

This command is just used to generate some random noise ...

```bash
$ rosrun rosshell rosshell.py "aplay -c 2 -f S16_LE -r 44100 /dev/urandom"
```
Pipe the results of /dev/random into a text file ...

```bash
$ rosrun rosshell rosshell.py "cat /dev/random > test.txt"
```
or the some information about the current directory ...

```bash
$ rosrun rosshell rosshell.py "ls -Shal > test.txt"
```

### 3.2 rosshellX

This node is used if you require some interaction with stdin, stdout, and
stderr.

rosshellX is by default subscribed to `/rosshellX/stdin` and publishes at topic
`/rosshellX/stdout` and `/rosshellX/stderr`, all of type `std_msgs/String`.

### 3.2.1 Some basic calculations

bc is just a simple commandline-calculator

1. open three shells

2. start bc with the following command:

   ```bash
   $ rosrun rosshell rosshellX.py "bc"
   ```

3. print the the results of bc stdout by running:

   ```bash
   $ rostopic echo /rosshellX/stdout
   ```

4. and now, send some inputs like:

   ```bash
   $ rostopic pub /rosshellX/stdin std_msgs/String "99*99\n"
   $ rostopic pub /rosshellX/stdin std_msgs/String "sqrt(2.0)\n"
   $ rostopic pub /rosshellX/stdin std_msgs/String "33^3\n"
   $ rostopic pub /rosshellX/stdin std_msgs/String "quit\n"
   ```

5. to interact a bit with stderr, simply subscribe for:

   ```bash
   $ rostopic echo /rosshellX/stderr
   ```

   and publish some nonsense like

   ```bash
   $ rostopic pub /rosshellX/stdin std_msgs/String "wtf\n"
   ```

### 3.2.1 Additional functionality

To get some filesystem-information try:

```bash
$ rosrun rosshell rosshellX.py "ls -Shal"
```
or

```bash
$ rosrun rosshell rosshellX.py _command:="ls -Shal"
```
both are the same but you also have the possibility to change the command and
topics, also within the launch-file, have a look at rosshellX.launch ...

```xml
<node pkg="rosshell" type="rosshellX.py" name="rosshellBC">
  <param name="command" type="String" value="bc" />
  <param name="stdout"  type="String" value="bc/stdout" />
  <param name="stdout"  type="String" value="bc/stderr" />
  <param name="stdin"   type="String" value="bc/stdin" />
</node>
```

## 4. Nodes

### 4.1 rosshell
Allows to run commands with no interaction possibilities.

### 4.2 rosshellX
Allows to run commands and enables an interaction over stdin and stdout.

#### 4.2.1 Subscribed Topics

`/rosshellX/stdin (std_msgs/String)`

The standard topic for stdin.

#### 4.2.2 Published Topics

`/rosshellX/stdout (std_msgs/String)`

The current line of the stdout.

`/rosshellX/stderr (std_msgs/String)`

The current line of the stderr.

#### 4.2.3 Parameters

`command (string, default: "")`

Commands that will be executed.

`stdout (string, default: /rosshellX/stdout)`

Change the topic for `stdout`.

`stdin (string, default: /rosshellX/stdin)`

Change the topic for stdin.

`stderr (string, default: /rosshellX/stderr)`

Change the topic for `stderr`.
