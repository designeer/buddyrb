---
layout: post
title: How to make a Docker image of ROS
---

# How to make a Docker image of ROS

## 1. System environment of local PC
- OS : Ubuntu 16.04
- Graphic board : NVIDIA GeForce GTX 1050

## 2. Procedure to make a Docker image

### 2.1 Install Docker

```shell-session
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
yes | sudo apt-get install docker-ce docker-ce-cli containerd.io
sudo docker run hello-world  # To verify that Docker engine
sudo gpasswd -a [username] docker  # Not to input sudo every time when use docker command. re-login after belows
```

### 2.2 Make a Docker image of Ubuntu where NVIDIA driver is installed

#### About Docker image
- Ubuntu 16.04 (Xenial)

#### Pull Ubunut 16.04 image from Docker hub
On Docker Client PC :
```shell-session
docker pull ubuntu:xenial  # docker pull [image name]
```
#### Make a container
On Docker Client PC :
```shell-session
docker run -it --name ubuntu_xenial ubuntu:xenial  # docker run -it --name [container name] [image name]
```
Then Docker Client terminal becomes to a Docker host terminal

#### NVIDIA driver installation
On Docker host terminal : 
```shell-session
apt-get update
yes | apt-get upgrade
yes | apt-get install software-properties-common  # this command is necessary for using add-apt-repository on next line
apt-get update
yes "" | add-apt-repository ppa:graphics-drivers
apt-get update 
yes | apt-get install nvidia-384
```
While executing `yes | apt-get install nvidia-384`, it might be needed to input password for Secure boot and reboot.  

#### Make a new Docker image where NVIDIA driver is installed
Open a new terminal as a Docker Client  
```shell-session
docker commit ubuntu_xenial ubuntu:xenial_nvidia-384   # docker commit [container name] [tag name of a new Docker image]
```

### 2.3 Make a Docker image where ROS Kinetic desktop-full is installed 

#### Make a Dockerfile
On the Docker Client terminal, make a local directory :
```shell-session
cd ~/
mkdir -p Docker_local/ros/kinetic/nvidia-384/ubuntu/xenial/desktop-full/
cd ~/Docker_local/ros/kinetic/nvidia-384/ubuntu/xenial/desktop-full/
```

Make a **Dockerfile** on the current directory  
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
# Ohter ROS & Python Packages
#====================================================================
RUN yes | sudo apt-get install ros-kinetic-moveit
#RUN source /opt/ros/kinetic/setup.bash
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

#### Build a Docker image

On the Docker Client terminal of which current path is ~/Docker_local/ros/kinetic/nvidia-384/ubuntu/xenial/desktop-full/ :
```shell-session
docker build --tag ros:kinetic-desktop-full-nvidia-384 .
```

After building a image, you can confirm the new Docker image by executing `Docker images`


### 2.4 Make a Docker image where Tsumitsuke is installed 

#### Install Assimp and Ensenso driver

Make a Docker container from a new Docker image which is made above :
```shell-session
cd ~/
docker run -it --name ros_kinetic_full_nvidia ros:kinetic-desktop-full-nvidia-384  # docker run -it --name [container name] [image name]
``` 
Then the terminal becomes to Docker client terminal. On this terminal, install Assimp :
```shell-session
cd ~/
git clone https://github.com/assimp/assimp.git 
cd assimp
cmake CMakeLists.txt -G 'Unix Makefiles'
sudo make install
cd port/PyAssimp
sudo python setup.py install
``` 

On Docker client terminal, install Ensenso driver
```shell-session
mkdir ~/Downloads
cd ~/Downloads
sudo apt-get install wget -y  # To use wget command
wget -O ensenso.deb "https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.2.170-x64.deb" 
#wget -O ensenso.deb "https://download.ensenso.com/s/ensensosdk/download?files=ensenso-sdk-2.2.65-x64.deb"  # this is an old one
dpkg -i ensenso.deb
#sudo reboot now
exit 
```

After installing Ensenso driver, in the real PC environment we usually do reboot, but we can not use reboot on Docker.  
So I just stopped Docker container and I'll restart Docker daemon like below (If you don't restart it here, ensenso driver will not be detected) :

```shell-session
sudo systemctl restart docker.service  # restart Docker deamon
docker restart ros_kinetic_full_nvidia  # restart Docker container
docker attach ros_kinetic_full_nvidia
export ENSENSO_INSTALL=/opt/ensenso
``` 

#### Install Tsumitsuke

On Docker host terminal, Clone **cpr_ros.git** from GIT repository and build it :

```shell-session
cd ~/
git clone https://[user ID]:[password]@github.com/KyotoRobotics/cpr_ros.git
cd cpr_ros
git submodule update --init --recursive
catkin_make
echo "source ~/cpr_ros/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Make a new Docker image where Tsumitsuke is installed

Open a new terminal as a Docker Client, then execute below :   
```shell-session
docker commit ros_kinetic_full_nvidia ros:kinetic-tsumitsuke_nvidia   # docker commit [container name] [tag name of a new Docker image]
```

#### Some problems on confirming that CPR works

To confirm that CPR works normally, I executed procedures below :  

- On Docker client terminal, make a Docker container : 
	- `docker run -it --name ros_tsumitsuke_nvidia ros:kinetic-tsumitsuke_nvidia`
- On Docker host terminal, execute : 
	- `roslaunch kr24_gui kr24_gui.launch`

Message like belows come out :
```
QXcbConnection: Could not connect to display 
[kr24_gui-2] process has died [pid 94, exit code -6, cmd /root/cpr_ros/src/kr24_gui/scripts/kr24_gui.py __name:=kr24_gui __log:=/root/.ros/log/a36b6584-074b-11ea-a552-0242ac110002/kr24_gui-2.log].
log file: /root/.ros/log/a36b6584-074b-11ea-a552-0242ac110002/kr24_gui-2*.log
```
Solution for this problem is ["Could not connect to display" problem](#Solution for 'Could not connect to display' problem)  

After you clear the ["Could not connect to display" problem](#Solution for 'Could not connect to display' problem), On Docker host terminal, execute :
```shell-session
roslaunch kr24_gui kr24_gui.launch
```

Then message like belows come out :
```
ResourceNotFound: timed_roslaunch
ROS path [0]=/opt/ros/kinetic/share/ros
ROS path [1]=/root/cpr_ros/src
ROS path [2]=/opt/ros/kinetic/share
[WARN] [1573795307.844979]: Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details.
[INFO] [1573795309.534404]: Failed to kill node"/supervisor"
[INFO] [1573795309.819485]: Failed to kill node"/tsumi_manager"
[INFO] [1573795310.100411]: Failed to kill node"/motion_planner"
[INFO] [1573795310.384467]: Failed to kill node"log_server"
Traceback (most recent call last):
  File "/root/cpr_ros/src/kr24_gui/scripts/kr24_gui.py", line 399, in <module>
    sys.exit(app.exec_())
```


#### Solution for "Could not connect to display" problem

I referred to the following :  
[Launch GUI on Ubuntu on Docker](https://qiita.com/mocobt/items/726024fa1abf54d843e1)  
[Launch GUI on Ubuntu on Docker (Referenced from above URL) ](https://unskilled.site/docker%E3%82%B3%E3%83%B3%E3%83%86%E3%83%8A%E3%81%AE%E4%B8%AD%E3%81%A7gui%E3%82%A2%E3%83%97%E3%83%AA%E3%82%B1%E3%83%BC%E3%82%B7%E3%83%A7%E3%83%B3%E3%82%92%E8%B5%B7%E5%8B%95%E3%81%95%E3%81%9B%E3%82%8B/)  

I solved this problem as follows : 

- Add socket setting as a volume of to Docker host and set X-server display number on Docker host. Execute below :
```shell-session
docker run -it --name ros_tsumitsuke_nvidia -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix ros:kinetic-tsumitsuke_nvidia
```
- Opening up xhost only to the specific system, the Docker container
On a new terminal, execute below :
```shell-session
xhost +local:[container ID]  # container ID can be like **c3a3a0bbe4d0**
```
- Test whether **xterm** launch normally. On Docker host terminal, execute `xterm`. If **xterm** launches, it means OK.
- Test below again. GUI will come out.
```
roslaunch kr24_gui kr24_gui.launch
```


#### Solution for "Inbound TCP/IP connection failed" problem

For now, the reason for this is that using **roslaunch** in Docker requires a special way because it needs that multiple processes(terminal) shoud be launched from Docker and communicate with each other. 

[Docker + ROS(kinetic) tutorial including Docker compose](https://qiita.com/Leonardo-mbc/items/cfd38a4fae8667593cf1)
