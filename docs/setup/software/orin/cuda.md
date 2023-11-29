# GPU Acceleration Libraries
## CUDA Update
Jetson Orin devices with Jetpack version larger than 5 have the possibility of CUDA updates without changing the Jetpack. To do this, simply go to the CUDA download link (e.g. [CUDA 11.8](https://developer.nvidia.com/cuda-11-8-0-download-archive?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=20.04&target_type=deb_local)) and follow the steps. For ease of reference, the steps are also mentioned here:

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-ubuntu2004.pinsudo 

mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600

wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.debsudo 

dpkg -i cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.deb

sudo cp /var/cuda-tegra-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/

sudo apt-get updatesudo apt-get -y install cuda
```

Then check the installation by opening the python interpreter and running the following:

```python
import torch 
torch.cuda.is_available()
```
## Installing the Pytorch with CUDA Support
To install pytorch with CUDA support, simply run the following [source](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html):

```bash
sudo apt-get -y update

sudo apt-get -y install autoconf bc build-essential g++-8 gcc-8 clang-8 lld-8 gettext-base gfortran-8 iputils-ping libbz2-dev libc++-dev libcgal-dev libffi-dev libfreetype6-dev libhdf5-dev libjpeg-dev liblzma-dev libncurses5-dev libncursesw5-dev libpng-dev libreadline-dev libssl-dev libsqlite3-dev libxml2-dev libxslt-dev locales moreutils openssl python-openssl rsync scons python3-pip libopenblas-dev

export TORCH_INSTALL=https://developer.download.nvidia.cn/compute/redist/jp/v511/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

python3 -m pip install --upgrade pip

python3 -m pip install aiohttp numpy=='1.19.4' scipy=='1.5.3' 

export "LD_LIBRARY_PATH=/usr/lib/llvm-8/lib:$LD_LIBRARY_PATH"

python3 -m pip install --upgrade protobuf

python3 -m pip install --no-cache $TORCH_INSTALL
```


## Installing the NVIDA Warp
Installing [NVIDA warp](https://github.com/NVIDIA/warp) through pip is not possible and the library has to be compiled and installed from source. To do this, first update the CUDA to a version larger than 11.5 and proceed through the following steps:

### Add CUDA Compiler Path

First let's add the CUDA binary path to the system PATH. 

```bash
export CUDA_PATH=/path/to/cuda # e.g. /usr/local/cuda-11.8
export PATH=$PATH:${CUDA_PATH}/bin
```
Then make sure that g++ and gcc are installed and have the same version:

```bash
g++ --version
gcc --version
```
If the version is different, install the correct version of the compiler and change the symbolic link to the correct version:

```bash
sudo rm /usr/bin/g++ # or gcc
sudo ln -s /usr/bin/g++-<version> /usr/bin/g++ # or gcc
```
### Clone The Repository:

First clone the repository and make the library:
```bash
git clone https://github.com/NVIDIA/warp.git
cd warp
python3 build_lib.py --cuda_path $CUDA_PATH
```
After successful compilation, install the library as follows:

```bash
python3 -m pip install .
```
**Note**: There might be some linking errors after making the library. It doesn't essentially imply that the installation will not be successful.

### Test the Library

Finally, run the following in the python3 interpreter to check the installation:

```python 
import warp as wp
wp.init()
```

If successful, the following output will be printed out:

```
Warp 1.0.0-beta.5 initialized:
   CUDA Toolkit: 11.8, Driver: 11.4
   Devices:
     "cpu"    | aarch64
     "cuda:0" | Orin (sm_87)
   Kernel cache: /home/robocaster-orin/.cache/warp/1.0.0-beta.5
```



