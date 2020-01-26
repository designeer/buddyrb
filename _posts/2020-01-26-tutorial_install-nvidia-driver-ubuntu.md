---
layout: post
title: How to install a NIDIA driver on Ubuntu 16.04
tags: [Ubuntu, GPU]
---

To use GPU on your PC, you need to install NVIDIA driver first. In this post, you can see how to install a NVIDIA driver on Ubuntu.  

### System environment of my local PC

- OS : Ubuntu 16.04
- Graphic board : NVIDIA GeForce GTX 1050

### NVIDIA driver installation

Installation :
```bash
yes | apt-get install software-properties-common  # this command is necessary for using add-apt-repository on next line
apt-get update
yes "" | add-apt-repository ppa:graphics-drivers
apt-get update 
yes | apt-get install nvidia-384
```

While executing `yes | apt-get install nvidia-384`, it might be needed to input password for Secure boot and reboot.

To check NVIDIA's version :

```bash
nvidia-smi
```

To check where your nvidia driver is installed :
```bash
which nvidia-smi
```