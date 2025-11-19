# SLAMON

SLAMON is an improved and extended version of stereo_slam.
Like the original project, it implements a stereo visual SLAM 
system designed for underwater environments.

SLAMON runs three main threads:
1. A robust visual odometer for underwater environments.
2. A topological map creator where the nodes are clusters 
   of keypoints and the edges are the number of matches 
   and the relative movement between nodes.
3. A loop closer that detects revisited areas by proximity and hashes.


## Dependencies

- ROS Noetic
- OpenCV 4.2.0 + contrib
- G2O
- Eigen3
- SuiteSparse
- cmake_modules (ROS)


## Installation

**0. Install dependencies**

ROS NOETIC:
```bash
# 0. Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 1. Setup your keys
sudo apt -y install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 2. Keep ubuntu up to date
sudo apt -y update
sudo apt -y upgrade

# 3. Install ROS Noetic
sudo apt -y install ros-noetic-desktop-full

# 4. Install catkin tools
sudo apt -y install python3-catkin-tools
```

OPENCV-4.2.0 WITH CONTRIB MODULES:
```bash
# 0. Keep ubuntu or debian up to date
sudo apt -y update
sudo apt -y upgrade

# 1. Install the dependencies.
# Build tools:
sudo apt-get install -y build-essential cmake

# GUI (if you want to use GTK instead of Qt, replace 'qt5-default' with 'libgtkglext1-dev' and remove '-DWITH_QT=ON' option in CMake):
sudo apt-get install -y libgtkglext1-dev libvtk6-dev

# Media I/O:
sudo apt-get install -y zlib1g-dev libjpeg-dev libwebp-dev libpng-dev libtiff5-dev libopenexr-dev libgdal-dev libgphoto2-dev

# Video I/O:
sudo apt-get install -y libdc1394-22-dev libavcodec-dev libavformat-dev libswscale-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev yasm libopencore-amrnb-dev libopencore-amrwb-dev libv4l-dev libxine2-dev v4l-utils

# Parallelism and linear algebra libraries:
sudo apt-get install -y libtbb-dev libeigen3-dev

# Python:
sudo apt-get install -y python-dev python-tk python-numpy python3-dev python3-tk python3-numpy

# 2. Install the library (you can change '4.2.0' for the last stable version)
sudo apt-get install -y unzip wget
wget https://github.com/opencv/opencv/archive/refs/tags/4.2.0.zip -O opencv420.zip
unzip opencv420.zip
rm opencv420.zip
mv opencv-4.2.0 OpenCV420
cd OpenCV420
touch OpenCV420withContrib

# 3. Install the opencv_contrib library (you can change '4.2.0' for the last stable version)
wget https://github.com/opencv/opencv_contrib/archive/refs/tags/4.2.0.zip -O opencv_contrib420.zip
unzip opencv_contrib420.zip
rm opencv_contrib420.zip
mv opencv_contrib-4.2.0 OpenCV420_contrib

# 4. Build OpenCV with contrib
## Fix Flow control statements are not properly nested issue
sed -i '21,22 s/^/#/' cmake/OpenCVCompilerOptions.cmake
## Fix ffmpeg version issue
sed -i '1 i #define AV_CODEC_FLAG_GLOBAL_HEADER (1 << 22)\n#define CODEC_FLAG_GLOBAL_HEADER AV_CODEC_FLAG_GLOBAL_HEADER\n#define AVFMT_RAWPICTURE 0x0020' modules/videoio/src/cap_ffmpeg_impl.hpp
## Fix build.make:56 Error
sed -i 's/PyString_AsString(obj);/(char*)PyString_AsString(obj);/' modules/python/src2/cv2.cpp
## Requires version 10 or lower
if [[ `gcc -dumpversion` -ge 11 ]]; then
    sudo apt-get install -y gcc-10 g++-10 libtbb2-dev
    CC="gcc-10"
    CXX="g++-10"
fi

mkdir build
cd build
CC=${CC} CXX=${CXX} cmake -DOPENCV_ENABLE_NONFREE=ON -DOPENCV_EXTRA_MODULES_PATH=../OpenCV420_contrib/modules -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=RELEASE -DWITH_QT=OFF -DWITH_OPENGL=ON -DFORCE_VTK=ON -DWITH_TBB=ON -DINSTALL_C_EXAMPLES=ON -DINSTALL_PYTHON_EXAMPLES=ON -DWITH_GDAL=ON -DWITH_XINE=ON -DBUILD_EXAMPLES=OFF -DENABLE_PRECOMPILED_HEADERS=OFF ..
CC=${CC} CXX=${CXX} make -j`nproc`
sudo make install
sudo ldconfig
```

G2O:
```bash
sudo apt install ros-noetic-libg2o

```
CMAKE-MODULES:
```bash
sudo apt install ros-noetic-cmake-modules
```

**1. Create catkin_ws**
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin build
cd ..
source devel/setup.bash
```

**2. Copy and compile SLAMON**
```bash
cd ~/catkin_ws/src
git clone git@github.com:bomiquel/slamon.git
catkin build slamon
```


## Usage
Modify or create your own launch file based on the templates in launch/, then run:
```bash
roslaunch slamon launch_file_name.launch
```


## License

This project is distributed under **BSD-4-Clause**. 

See the [LICENSE](LICENSE) file for the full license text.


## Acknowledgements

Bo Miquel Nordfeldt-Fiol was supported by the Conselleria d'Educació i Universitats de les Illes Balears under the contract FPU-2024-006-C CAIB-2024.


## Contact

If you have problems or questions using SLAMON, please contact the author (bo-miquel.nordfeldt@uib.cat). [Feature requests](https://github.com/bomiquel/slamon/issues) and [contributions](https://github.com/bomiquel/slamon/pulls) are totally welcome.