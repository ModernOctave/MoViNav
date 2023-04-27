#!/bin/bash

sudo apt install -y python3.8 &&\
python3.8 -m pip install --upgrade pip &&\
python3.8 -m pip install pip install -U torch==1.7.1+cu110 torchvision==0.8.2+cu110 torchaudio==0.7.2 -f https://download.pytorch.org/whl/torch_stable.html &&\
python3.8 -m pip install -U -r requirements.txt &&\
cp -rf src/* ~/catkin_ws/src/ &&\
cd ~/catkin_ws &&\
catkin_make