---
layout: post
title: Ubuntu 16.04 에서 ROS Kinectic 의 Docker image 만들기
tags: [Docker, ROS]
---

이번 포스트에서는 Ubuntu 16.04 (Xenial) 환경에서 ROS Kinetic 의 Docker container 를 만드는 방법을 설명합니다.

- [Procedure to make a Docker image](#procedure-to-make-a-docker-image)
	- [1. Install Docker](#1-install-docker)
	- [2. Make a Docker container of Ubuntu from Docker hub](#2-make-a-docker-container-of-ubuntu-from-docker-hub)
	- [3. ROS Kinetic installation and make a new Docker image](#3-ros-kinetic-installation-and-make-a-new-docker-image)

- [(Optional) Basic Docker operation](#optional-basic-docker-operation)
	- [Make a Docker container](#make-a-docker-container)
	- [Exit and start Docker container](#exit-and-start-docker-container)
	- [Delete Docker image and container](#delete-docker-image-and-container)
	- [Solution for "Could not connect to display" problem](#solution-for-could-not-connect-to-display-problem)

영문 포스트는 다음을 참조 하세요. [How to make a Docker image of ROS Kinectic on Ubuntu 16.04]({% post_url 2020-01-07-tutorial_docker-ros-kinetic-ubuntu16.04 %})


## Procedure to make a Docker image

### 1. Install Docker

Docker 인스톨 하기:
```bash
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
yes | sudo apt-get install docker-ce docker-ce-cli containerd.io
```

동작확인 :
```bash 
sudo docker run hello-world  # To verify that Docker engine
sudo gpasswd -a [username] docker  # Not to input sudo every time when use docker command. re-login after belows
```

### 2. Make a Docker container of Ubuntu from Docker hub

Docker 클라이언트-서버 아키텍쳐를 사용합니다. Docker client talks to the Docker daemon, which does the heavy lifting of building, running, and distributing your Docker containers. The Docker client and daemon can run on the same system, or you can connect a Docker client to a remote Docker daemon.

Firstly, pull the Ubunut 16.04 (Xenial) image from Docker hub. On Docker client PC (local PC):
```bash
# docker pull [image name]
docker pull ubuntu:xenial
```

Secondly, make a Docker container from the downloaded image. On Docker client PC (local PC):
```bash
# docker run -it --name [container name] [image name]
docker run -it --name ubuntu_xenial ubuntu:xenial
apt-get update
yes | apt-get upgrade
```
Then Docker client PC's terminal becomes to a Docker deamons's one.


### 3. ROS Kinetic installation and make a new Docker image

There are 2 ways of install ROS on Docker.

#### 3.1 By using **Dockerfile**

On the Docker Client terminal, make a local directory for ROS Kinetic installation:
```bash
cd ~/[local workspace directory]
mkdir -p docker_ws/ros/kinetic/
cd -p docker_ws/ros/kinetic/
```

Make a **Dockerfile** on the current installation directory.
**Dockerfile** :  
```
FROM ubuntu:xenial_nvidia-384

#====================================================================
# Install ros kinetic-desktop-full packages
#====================================================================
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update
RUN apt upgrade
RUN yes | apt-get install ros-kinetic-desktop-full
RUN rosdep init
RUN rosdep update
RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
#RUN source ~/.bashrc

#====================================================================
# (Optional) Ohter ROS & Python Packages
#====================================================================
RUN yes | sudo apt-get install ros-kinetic-moveit
RUN yes | sudo apt-get install ros-kinetic-industrial-core
RUN yes | sudo apt-get install ros-kinetic-rosbridge-server 
RUN yes | sudo apt-get install ros-kinetic-tf2-web-republisher

RUN yes | sudo apt-get install python-pip python-dev python-rosinstall python-rosinstall-generator python-wstool build-essential 
RUN sudo pip install --upgrade pip 
RUN sudo pip install scipy
RUN sudo pip install rospy-message-converter
RUN sudo pip install pandas

RUN sudo pip install memory_profiler
RUN sudo pip install numba
RUN sudo pip install networkx
RUN sudo pip install pymunk
RUN sudo pip install transitions
RUN yes | sudo apt-get install graphviz libgraphviz-dev pkg-config
RUN sudo pip install pygraphviz
```

To make a Docker image from **Dockerfile**, on the Docker Client terminal, execute the below :
```bash
docker build --tag ros:kinetic .
```

After building a image, you can confirm the new Docker image by executing below :
```bash
docker images
```

#### 3.2 Without using **Dockerfile**

You can install ROS on Docker container directly and create a new image from the container's changes.  
After finishing installation of ROS on your container, execute the below :  
```bash
docker commit [a new container name] ros:kinetic   # docker commit [OPTIONS] CONTAINER [REPOSITORY[:TAG]]
```


## (Optional) Basic Docker operation

Here is a description for basic Docker operaition using the ROS Kinetic Docker image.

### Make a Docker container

You can make a Docker container from the new Docker image which is made above.
Launch a new Docker client terminal and execute the below :
```bash
cd ~/
# docker run -it --name [a new container name] [REPOSITORY[:TAG]]
docker run -it --name ros_kinetic ros:kinetic
``` 

Then the terminal becomes to Docker deamon and it will look like below :
```bash
root@39d2a5125296:/# ls
bin   dev  home  lib64  mnt  proc  ros_entrypoint.sh  sbin  sys  usr
boot  etc  lib   media  opt  root  run                srv   tmp  var
root@39d2a5125296:/#
```

To find the container which is currently running, launch a new terminal and execute the below :
```bash
docker ps
# docker ps -a : You can see both running container and stopped container
```

Then you can see the result like below :
```
root@39d2a5125296:/# ls
bin   dev  home  lib64  mnt  proc  ros_entrypoint.sh  sbin  sys  usr
boot  etc  lib   media  opt  root  run                srv   tmp  var
root@39d2a5125296:/#
```

### Exit and start Docker container

On the Docker deamon terminal, you can get out of Docker deamon like below.
```bash
root@39d2a5125296:/# exit
exit
$
```

Check the exited or stopped container currently.
```bash
# To see the container which is runnging or stopped
docker ps -a
```

The result will be like below :
```bash
$ docker ps -a
CONTAINER ID        IMAGE                  COMMAND                  CREATED             STATUS                        PORTS               NAMES
39d2a5125296        ros:kinetic            "/ros_entrypoint.sh …"   12 hours ago        Exited (127) 22 seconds ago                       ros_kinetic
```

You can see the CONTAINER ID 39d2a5125296's status exited, and its name is **ros-kinetic**. 

To start the container, execute the below:
```bash
# To see the container which is runnging or stopped
docker ps -a
# To start container
docker start ros-kinetic # after starting container, its status becomes to Up.
docker attach ros_kinetic
```


### Delete Docker image and container

To delete container,
```bash
docker rm ros_kinetic # docker rm [container name / container ID]
```

To delete image,
```bash
docker rmi ros_kinetic # docker rmi [image name / image ID]
```

### Solution for "Could not connect to display" problem

I referred to the following site.

* [Launch GUI on Ubuntu on Docker](https://qiita.com/mocobt/items/726024fa1abf54d843e1)
* [Launch GUI on Ubuntu on Docker (Referenced from above URL) ](https://unskilled.site/docker%E3%82%B3%E3%83%B3%E3%83%86%E3%83%8A%E3%81%AE%E4%B8%AD%E3%81%A7gui%E3%82%A2%E3%83%97%E3%83%AA%E3%82%B1%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E3%82%92%E8%B5%B7%E5%8B%95%E3%81%95%E3%81%9B%E3%82%8B/)  


I solved this problem as follows : 

- Add socket setting as a volume of to Docker host and set X-server display number on Docker host. Execute below :
```bash
docker run -it --name [a new container name] -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix [REPOSITORY:[TAG]]
```

- Opening up xhost only to the specific system, the Docker container
On a new terminal, execute below :
```bash
xhost +local:[container ID]  # container ID can be like **c3a3a0bbe4d0**
```

- Test whether **xterm** launch normally. On Docker host terminal, execute `xterm`. If **xterm** launches, it means OK.

- Test `rosrun` or `roslaunch` and see whether GUI will come out.

<!---
### Solution for "Inbound TCP/IP connection failed" problem

For now, the reason for this is that using **roslaunch** in Docker requires a special way because it needs that multiple processes(terminal) shoud be launched from Docker and communicate with each other. 

[Docker + ROS(kinetic) tutorial including Docker compose](https://qiita.com/Leonardo-mbc/items/cfd38a4fae8667593cf1)
-->