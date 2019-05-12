# DUAV2019
### ESTIA's participation to Dassault's UAV Challenge

## Introduction

This repository contains the structure, code, and tutos of our project. We tried to make it as reusable and understandable as possible. 
We use YOLO for detection, with Joseph Redmon's Darknet framework. Which we build and run in docker to make the installation easier (with nvidia-docker for GPU use).

 #### Structure
 
 ```
 .
├── .gitignore
├── .docker
│   ├── Detector
│   │   └── Dockerfile
├── Detector
│   ├── train.log
│   ├── backup
│   │   ├── darknet53.conv.74
│   │   └── yolov3-tiny.conv.15
│   ├── data
│   │   ├── labels
│   │   ├── obj.data
│   │   ├── obj.names
│   │   ├── test.txt
│   │   └── train.txt
│   ├── cfg
│   │   ├── yolov3.cfg
│   │   └── yolov3-tiny.cfg
│   ├── Dataset
│   ├── TestSet
│   ├── Utils
│   │   ├── clean_directory.py
│   │   ├── plot_loss.py
│   │   ├── split_directory.py
│   └── darknet
│   │   ├── ...
├── media
└── tmp

 ```

## Installation

### This is all on Linux

#### Requirements

* First of all you will need to have Linux installed, we recommend Ubuntu if you are not familiar with linux.
You can choose to install it directly on your personnal computer by following instructions [here](https://doc.ubuntu-fr.org/installation).
Or you can use it in a Virtual machine using [VirtualBox](https://www.virtualbox.org/) by following [these instructions](https://doc.ubuntu-fr.org/virtualbox).

* Then you will need Git, open a terminal and run:
```
sudo apt update
sudo apt install git
```

* Next in line is Docker, with nvidia-docker IF you have a nvidia GPU.

before installing you have to set up Docker's repository, run successively:
```
sudo apt update
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```
Then install Docker:
```
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io
```
Check that docker is installed correctly:
```
sudo docker run hello-world
```
**Otherwise, if you are in a hurry you can just run:**
```
curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh get-docker.sh
```
* Finally you can install nvidia docker(Optionnal):

**Make sure you have installed the [NVIDIA driver](https://github.com/NVIDIA/nvidia-docker/wiki/Frequently-Asked-Questions#how-do-i-install-the-nvidia-driver).**

Add the package repositories
```
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update
```
Install nvidia-docker2 and reload the Docker daemon configuration
```
sudo apt-get install -y nvidia-docker2
sudo pkill -SIGHUP dockerd
```
Test nvidia-smi with the latest official CUDA image
```
docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi
```

clone git repo
build darknet in docker for gpu

## Tutos

Train
labelImg
dronekit
