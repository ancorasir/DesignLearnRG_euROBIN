# Design & Learning Research Group Submission - euROBIN MSVC @ IROS 2024

[[Report Page]](https://msvc-dlrg.github.io) | [[Video]]()

<img src="assets/images/teaser.jpg" width="100%">

## Overview

This repository contains the code, and brief introduction of our approach used for [the euROBIN Manipulation Skill Versatility Challenge (MSVC)](https://sites.google.com/view/eurobin-msvc/) at [the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024)](https://iros2024-abudhabi.org/). For more details, please refer to the [report page](https://msvc-dlrg.github.io).

## Hardware Setup

The robot system used in the competition consists of:

- Collebrative robot: [Universal Robots UR10e](https://www.universal-robots.com/products/ur10-robot/)
- Adaptive gripper: [Robotiq Hand-E](https://robotiq.com/products/adaptive-grippers#Hand-E)
- Fingertip: [3D-printed fingertip](https://cad.onshape.com/documents/43edc50e275c72eace7a4839)
- RGB-D Camera: [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- Camera bracket: [CNC-milled camera bracket](https://cad.onshape.com/documents/01d4267b0af8aab9d6acb1ab)
- Fill light: [ZHIYUN FIVERAY M20](https://www.zhiyun-tech.com/en/product/detail/867)

And there are also the [task board](docs/task_board.pdf) and [Hikvision NP-Y1-S smoke detector](https://detail.tmall.com/item.htm?id=654643896582) used as task objects.

## Software Dependencies

The code is developed with C/C++ and tested on Ubuntu 20.04 with the following dependencies:

- [Intel RealSense SDK 2.0](https://www.intelrealsense.com/sdk-2/)

## Quick Start

First, download the latest release:

```bash
git clone https://github.com/ancorasir/DesignLearnRG_euROBIN.git
cd DesignLearnRG_euROBIN
```

Then, build the code:

```bash
mkdir build
cd build
cmake ..
make
```

## 

## Data Availability

The data recorded during trials is available at [Google Drive](), which includes:

- [x] Positions and velocities of UR10e's joints
- [x] Trajectories of UR10e's tool center point (TCP)
- [x] Forces of Robotiq Hand-E gripper

All data is recorded at 30 Hz, and saved as csv files.

## License

This repository is released under the MIT License. See [LICENSE](LICENSE) for more information.
