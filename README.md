# Design and Learning Research Group Submission - euROBIN MSVC @ IROS 2024

[[Report Page]](https://msvc-dlrg.github.io) | [[Video]]()

## Overview

This repository contains the code, and brief introduction of our approach used for [the euROBIN Manipulation Skill Versatility Challenge (MSVC)](https://sites.google.com/view/eurobin-msvc/) at [the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024)](https://iros2024-abudhabi.org/). For more details, please refer to the [report page](https://msvc-dlrg.github.io).

## Hardware Setup

The robot system used in the competition consists of:

- Collebrative robot: [Universal Robots UR10e](https://www.universal-robots.com/products/ur10-robot/)
- Adaptive gripper: [Robotiq Hand-E](https://robotiq.com/products/adaptive-grippers#Hand-E)
- Fingertip: [3D-printed fingertip](src/meshes/fingertip.STEP)
- RGB-D Camera: [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- Camera bracket: [CNC-milled camera bracket](src/meshes/camera_bracket.STEP)
- Fill light: [ZHIYUN FIVERAY M20](https://www.zhiyun-tech.com/en/product/detail/867)

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

## License

This repository is released under the MIT License. See [LICENSE](LICENSE) for more information.