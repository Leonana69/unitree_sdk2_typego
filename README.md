# unitree_sdk2
Unitree robot sdk version 2.

### Prebuild environment
* OS  (Ubuntu 20.04 LTS)  
* CPU  (aarch64 and x86_64)   
* Compiler  (gcc version 9.4.0) 

### Environment Setup

Before building or running the SDK, ensure the following dependencies are installed:

- CMake (version 3.10 or higher)
- GCC (version 9.4.0)
- Make

You can install the required packages on Ubuntu 20.04 with:

```bash
apt-get update
apt-get install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev
```

Install the Livox-SDK2:
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```
Then edit the last two digits of `lidar_ip` in `scripts/mid360_config.json` based on the last two digits of your Livox lidar serial number.

Install v4l2 tool:
```bash
apt-get install v4l-utils
```

### Build examples

To build the examples inside this repository:

```bash
mkdir build
cd build
cmake ..
make
```

## Start System Service
Create the following file:
```
touch /etc/systemd/system/mystartup.service
```

Paste this content, you may need to change the `ExecStart` based on the location of this repo on the dog.
```
[Unit]
Description=Run TypeGo sensory data forwarding
After=network.target

[Service]
ExecStart=/root/unitree_sdk2_typego/scripts/run.sh
Restart=on-failure
User=root

[Install]
WantedBy=multi-user.target
```