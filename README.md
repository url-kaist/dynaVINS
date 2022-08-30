# DynaVINS : A Visual-Inertial SLAM for Dynamic Environments

## :bookmark_tabs: About DynaVINS (IEEE RA-L'22)

* A robust **visual-inertial state estimator algorithm** for dynamic environments.
* The robust bundle adjustment is used for dynamic objects such as cars :car:, humans :man-running:, buses :bus:, etc.

<p align="center"><img src=readme_resource/city_main.gif alt="animated" width="30%"/>  <img src=readme_resource/parking_main.gif alt="animated" width="30%"/></p>

* Please refer our [paper][arXivlink] for detailed explanantions and experimental results\!
    * Validated on [VIODE dataset][VIODElink] dataset\.
    * Algorithm is based on [VINS\-Fusion][vinsfusionlink]\.
* Youtube video for the detailed explanation and the demo video.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/a9jXZYc4tYw/0.jpg)](http://www.youtube.com/watch?v=a9jXZYc4tYw "DynaVINS")

[arXivlink]: https://arxiv.org/abs/2208.11500
[VIODElink]: https://github.com/kminoda/VIODE
[vinsfusionlink]: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

## :heavy_plus_sign: Additional package : VINS-Fusion-SC (SwitchableConstraints)

* In our paper\, [VINS\-Fusion][vinsfusionlink] combined with [Switchable Constraints][switchpaperlink] \([git][switchgitlink]\)was compared with DynaVINS\.
* VINS-Fusion-SC, an algorithm that integrates Switchable Constraints into the loop closure module of VINS-Fusion, is also useful, so we released the algorithm.
* You can visit [here][vfsclink] to use VINS\-Fusion\-SC\.
* When using VINS-Fusion-SC for your paper, don't forget to cite our paper!:wink:

[switchpaperlink]: https://ieeexplore.ieee.org/document/6385590
[switchgitlink]: https://github.com/sswan940505/switchable\_constraints
[vfsclink]:https://github.com/url-kaist/VinsFusionSC

## :package: Prerequisites
Prerequisites for DynaVINS are as same as for VINS-Fusion!

### 1. **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## :gear: Parameters

> Parameters of DynaVINS. You can find the results of each parameters on [wiki page][wikilink]

TODO

[wikilink]: https://github.com/url-kaist/dynaVINS/wiki/About-Parameters

## :building_construction: How to build

> Please follow below codes to build DynaVINS (on ROS).

``` bash
$ cd ~/catkin_ws/src 
$ git clone https://github.com/url-kaist/dynaVINS
$ cd ../
$ catkin_make  
(or if you use catkin tools) catkin build
$ source ~/catkin_ws/devel/setup.bash
```


## :runner: To run the demo codes




### VIODE dataset (Only BA) examples
### 1. **Parking lot sequence with monocular camera + IMU**

``` bash
# TODO
$ roslaunch dynavins blabla
```

### 2. **Parking lot sequence with stereo camera + IMU**

``` bash
# TODO
$ roslaunch dynavins blabla
```

### Our dataset (with Loop Closure module) examples

``` bash
# TODO
$ roslaunch dynavins blabla
```

## :bookmark: Citation

If you use our codes\, please cite our paper \([arXiv][arXivLink]\)

```
@article{song2022dynavins,
    title={DynaVINS: A Visual-Inertial SLAM for Dynamic Environments},
    author={Song, Seungwon and Lim, Hyungtae and Lee, Alex Junho and Myung, Hyun},
    journal={IEEE Robotics and Automation Letters},
    year={2022}
}
```

## :mailbox: Contact Information

If you have any questions, please do not hesitate to contact us

*[Seungwon Song][swlink] :envelope: `sswan55 at kaist.ac.kr`

[swlink]: [https://github.com/sswan940505](https://github.com/sswan940505)