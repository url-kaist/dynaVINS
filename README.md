# DynaVINS: A Visual-Inertial SLAM for Dynamic Environments

## :bookmark_tabs: About DynaVINS (IEEE RA-L'22)

* A robust **visual-inertial state estimator algorithm** for dynamic environments.
* The robust bundle adjustment is used for dynamic objects such as cars :car:, humans :runner:, buses :bus:, etc.

<p align="center"><img src=readme_resource/city_main.gif alt="animated" width="30%"/>  <img src=readme_resource/parking_main.gif alt="animated" width="30%"/></p>

* Please refer our [paper][arXivlink] for detailed explanantions and experimental results\!
    * The BA module validated on [VIODE dataset][VIODElink] dataset\.
    * The loop closure module validated on [Our dataset][ourdatalink]\.
    * Algorithm is based on [VINS\-Fusion][vinsfusionlink]\.
* Youtube video for the detailed explanation and the demo video.

[![IMAGE ALT TEXT](http://img.youtube.com/vi/a9jXZYc4tYw/0.jpg)](http://www.youtube.com/watch?v=a9jXZYc4tYw "DynaVINS")

[arXivlink]: https://arxiv.org/abs/2208.11500
[VIODElink]: https://github.com/kminoda/VIODE
[ourdatalink]: http://gofile.me/4355j/tsjdofd6S
[vinsfusionlink]: https://github.com/HKUST-Aerial-Robotics/VINS-Fusion

## :heavy_plus_sign: Additional package : VINS-Fusion-SC (SwitchableConstraints)

* In our paper\, [VINS\-Fusion][vinsfusionlink] combined with [Switchable Constraints][switchpaperlink] \([git][switchgitlink]\)was compared with DynaVINS\.
* VINS-Fusion-SC, an algorithm that integrates Switchable Constraints into the loop closure module of VINS-Fusion, is also useful, so we released the algorithm.
* You can visit [here][vfsclink] to use VINS\-Fusion\-SC\.
* When using VINS-Fusion-SC for your paper, don't forget to cite our paper!:wink:

[switchpaperlink]: https://ieeexplore.ieee.org/document/6385590
[switchgitlink]: https://github.com/sswan940505/switchable\_constraints
[vfsclink]:https://github.com/url-kaist/VinsFusionSC

--- 

## Test Env.

This code is tested on

* Linux 18.04 LTS
* ROS Melodic
* Ceres Solver 1.14.0
* OpenCV 3.4.1

## :package: Prerequisites

The dependency of DynaVINS is equal to that of VINS-Fusion.

### 1. **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 3. **Support file from VINS-Fusion**

Due to the limiting file size of Github, we need **one** package and **two** files from the [VINS-Fusion repository](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/master/support_files).

1. Set the `camera_models` package in your workspace, which is included in VINS-Fusion.
2. Copy `support_files/brief_k10L6.bin` in VINS-Fusion into our `support_files` folder 
3. Copy `support_files/brief_pattern.yml` in VINS-Fusion into our `support_files` folder

## :building_construction: How to build

> Please follow the below commands to build DynaVINS (on ROS).

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

For convenience, we also provide `3_high.bag` file in the `parking_lot` scene. You can download the file by the following command:

```bash
$ wget https://urserver.kaist.ac.kr/publicdata/dynaVINS/VIODE_dataset/parking_lot/3_high.bag
```

Note that the larger the number of bag files in the VIODE dataset is, the more dynamic objects exist.

#### 1. **VIODE sequence with monocular camera + IMU**

``` bash
$ roslaunch dynaVINS viode_mono.launch
$ rosbag play 3_high.bag (or 0_none.bag, 1_low.bag, ...)
```

#### 2. **VIODE sequence with stereo camera + IMU**

``` bash
$ roslaunch dynaVINS viode_stereo.launch
$ rosbag play 3_high.bag (or 0_none.bag, 1_low.bag, ...)
```


### Our dataset (with Loop Closure module) examples
> You can use your own intel realsense d455! (calibration is required)

You can easily download our bag file by the following command:

```bash
$ wget https://urserver.kaist.ac.kr/publicdata/dynaVINS/d455_urban_robotics/e_shape.bag
```

``` bash
$ roslaunch dynaVINS d455_mono.launch
$ rosbag play e_shape.bag (or loop_tempstatic.bag, ...)
```

``` bash
$ roslaunch dynaVINS d455_stereo.launch
$ rosbag play e_shape.bag (or loop_tempstatic.bag, ...)
```

---

## :gear: Parameters

> Parameters of DynaVINS. You can find the results of each parameter on the [wiki page (param)][wikilink1]

> Time comparison according to various parameters can be found on the [wiki page (time)][wikilink2].

+ `regularization_lambda`

    The Lambda value of regularization term in paper. (Section III-C)

+ `momentum_on`

    Using the momentum factor or not (true/false)

+ `momentum_lambda`

    The Lambda value of momentum term in paper. (Section III-D)

+ `alternating_converge`

    The threshold for checking the convergence of the alternating optimization.\
    90% is usually enough. If you want faster speed, please try to reduce it.\
    Time comparison can be found on the wiki page.

+ `margin_feature_thresh`

    Features which have less weight than this value are not used in marginalization.\
    This may affect accuracy, but is effective at reducing time costs.\
    You can try uncomment line 848 of "vins_estimator/estimator/estimator.cpp" to enable these features also in optimization.

    ```c++
    //ADDITIONAL FEATURE : NO USE IN OPT.
    //if(it_per_id.weight<DYN_FEAT_MARGIN_THRESH) continue;
    ```

+ `hypodiff_dist`

    The distance threshold for grouping constraints into the hypothesis. (Section IV-B)

+ `hypodiff_yaw`

     The angle threshold for grouping constraints into the hypothesis. (Section IV-B)

+ `hypo_regularization`

    The Lambda value of regularization term in loop closure module. (Section IV-C)

+ `hypo_alternating_converge`

    The threshold for checking the convergence of the alternating optimization in the loop closure module.

[wikilink1]: https://github.com/url-kaist/dynaVINS/wiki/About-Parameters
[wikilink2]: https://github.com/url-kaist/dynaVINS/wiki/Time-cost-comparison


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

If you have any questions, please do not hesitate to contact us:

* [Seungwon Song][swlink] :envelope: `sswan55 at kaist.ac.kr`
* [Hyungta Lim][htlink] :envelope: `shapelim at kaist.ac.kr`

[swlink]: [https://github.com/sswan940505](https://github.com/sswan940505)
[htlink]: [https://github.com/LimHyungTae](https://github.com/LimHyungTae)
