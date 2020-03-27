#!/bin/bash

sudo apt-get update
sudo apt-get upgrade -y

# # Install CUDA v10 and CUDNN v7.6.4
sudo apt-get install gcc-6 g++-6
sudo apt-get install build-essential cmake unzip pkg-config libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran libcanberra-gtk-module libcanberra-gtk3-module -y

cd ~
mkdir Installers
cd ~/Installers/
wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_Installers/cuda_10.0.130_410.48_linux
mv cuda_10.0.130_410.48_linux cuda_10.0.130_410.48_linux.run
chmod +x cuda_10.0.130_410.48_linux.run
sudo ./cuda_10.0.130_410.48_linux.run --override

echo 'export PATH=/usr/local/cuda-10.0/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64' >> ~/.bashrc
source ~/.bashrc

# # !!! Download cudnn first https://developer.nvidia.com/rdp/cudnn-archive
tar -zxf cudnn-10.0-linux-x64-v7.6.4.38.tgz
cd cuda
sudo cp -P lib64/* /usr/local/cuda/lib64/
sudo cp -P include/* /usr/local/cuda/include/
cd ~

# Install virualenv
cd ~/Installers/
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
sudo pip3 install virtualenv virtualenvwrapper
cd ~

echo 'export WORKON_HOME=$HOME/.virtualenvs' >> ~/.bashrc
echo 'export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3' >> ~/.bashrc
echo 'source /usr/local/bin/virtualenvwrapper.sh' >> ~/.bashrc

source ~/.bashrc

mkvirtualenv py3venv -p python3
workon py3venv
sudo pip3 install numpy defusedxml

# Install OpenCV
cd ~/Installers/
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.2.0.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.2.0.zip
unzip opencv.zip
unzip opencv_contrib.zip
mv opencv-4.2.0 opencv
mv opencv_contrib-4.2.0 opencv_contrib
cd ~

cd ~/Installers/opencv
rm -rf build
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=OFF \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D CUDA_ARCH_BIN=6.1 \
	-D WITH_CUBLAS=1 \
	-D OPENCV_EXTRA_MODULES_PATH=~/Installers/opencv_contrib/modules \
	-D HAVE_opencv_python3=ON \
	-D PYTHON_EXECUTABLE=~/.virtualenvs/py3venv/bin/python \
	-D BUILD_EXAMPLES=OFF ..

make -j$(nproc)
sudo make install
sudo ldconfig

ls -l /usr/local/lib/python3.6/site-packages/cv2/python-3.6
cd ~/.virtualenvs/py3venv/lib/python3.6/site-packages/
ln -s /usr/local/lib/python3.6/site-packages/cv2/python-3.6/cv2.cpython-36m-x86_64-linux-gnu.so cv2.so

# (Optional) for testing
python -c "import cv2; print(cv2.__version__)"
pip install cvlib

# Install catkin
sudo apt-get install python-catkin-tools python3-dev python3-numpy python3-empy

# (Optional) for https://github.com/wg-perception/people.git
sudo apt-get install ros-melodic-easy-markers ros-melodic-kalman-filter ros-melodic-bfl 

# (Optional) for ignisbot
pip uninstall em
pip uninstall Pillow -y
pip install --upgrade cython
pip install pyaml rospkg empy torch torchvision tqdm pycocotools matplotlib Pillow==6.2.2 gpustat

mkdir modules
cd modules
git clone https://github.com/NVIDIA-AI-IOT/torch2trt
cd torch2trt
~/.virtualenvs/py3venv/bin/python setup.py install
cd ..

git clone https://github.com/NVIDIA-AI-IOT/trt_pose
cd trt_pose
~/.virtualenvs/py3venv/bin/python setup.py install
cd ..

git clone https://github.com/NVIDIA-AI-IOT/jetcam
cd jetcam
~/.virtualenvs/py3venv/bin/python setup.py install
cd ..

tar xzvf ~/Installers/TensorRT-6.0.1.5.Ubuntu-18.04.x86_64-gnu.cuda-10.0.cudnn7.6.tar.gz
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Workspace/jetbot_dqn/modules/TensorRT-6.0.1.5/lib' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Workspace/jetbot_dqn/modules/TensorRT-6.0.1.5/targets/x86_64-linux-gnu/lib' >> ~/.bashrc
source ~/.bashrc

cd modules/TensorRT--6.0.1.5/python
pip install tensorrt-6.0.1.5-cp36-none-linux_x86_64.whl
cd ../
cd uff/
pip2 install uff-0.6.5-py2.py3-none-any.whl

# (Optional) Install baselines for reinforcement learning
pip install tensorflow==1.15rc2
cd src
git clone https://github.com/openai/baselines.git
cd baselines
pip install -e .
pip install gym
cd ..

catkin_make -DPYTHON_EXECUTABLE:FILEPATH=~/.virtualenvs/py3venv/bin/python
source devel/setup.bash