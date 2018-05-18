# Online Photometric Calibration

Recent direct visual odometry and SLAM algorithms have demonstrated
impressive levels of precision. However, they require a photometric
camera calibration in order to achieve competitive results. Hence, the
respective algorithm cannot be directly applied to an
off-the-shelf-camera or to a video sequence acquired with an unknown
camera. In this work we propose a method for online photometric
calibration which enables to process auto exposure videos with visual
odometry precisions that are on par with those of photometrically
calibrated videos. Our algorithm recovers the exposure times of
consecutive frames, the camera response function, and the attenuation
factors of the sensor irradiance due to vignetting. Gain robust KLT
feature tracks are used to obtain scene point correspondences as input
to a nonlinear optimization framework. We show that our approach can
reliably calibrate arbitrary video sequences by evaluating it on
datasets for which full photometric ground truth is available. We
further show that our calibration can improve the performance of a
state-of-the-art direct visual odometry method that works solely on
pixel intensities, calibrating for photometric parameters in an online
fashion in realtime. For more details please refer to our paper.

If you use this code in your research, we would appreciate if you cite
the respective publication.

**Online Photometric Calibration of Auto Exposure Video for Realtime Visual Odometry and SLAM** (P. Bergmann, R. Wang, D. Cremers),<br/>
*In IEEE Robotics and Automation Letters (RA-L)*, volume 3, 2018. [[pdf](https://vision.in.tum.de/_media/spezial/bib/bergmann17calibration.pdf)] [[video](https://youtu.be/nQHMG0c6Iew)]

For more information, see also
https://vision.in.tum.de/research/vslam/photometric-calibration.

## Install

We support Ubuntu 14.04 and 16.04, but it might work on a variety of
other platforms that meet the dependency requirements.

### Dependencies

The main dependencies are cmake 3.2 or later, a C++11 compiler, and
OpenCV 2.

#### Ubuntu 14.04

On Ubuntu 14.04 you need to get a more recent version of cmake.

```
sudo add-apt-repository ppa:george-edison55/cmake-3.x
```

Now continue to install depedencies like for Ubuntu 16.04.

#### Ubuntu 16.04

**Required:**

```
sudo apt-get update
sudo apt-get install \
    build-essential \
    g++ \
    cmake \
    libopencv-dev
```

**Optional:**

CCache can help you speed up repeated builds.

*Note:* You need at least cmake version 3.4 for ccache to work
 automatically.

```
sudo apt-get install ccache
```

### Compile

Start in the package directory.

```
mkdir build
cd build
cmake ..
make -j4
```

Optionally you can install the built libraries and exectuables.

```
sudo make install
```


## Usage

**TODO**


## License

This project was orginally developed at the [TUM computer vision
group](https://vision.in.tum.de) in 2017 by Paul Bergmann.

Later contributions were made by [Rui Wang](https://vision.in.tum.de/members/wangr) and [Nikolaus Demmel](https://vision.in.tum.de/members/demmeln).

This project is available under a BSD 3-Clause license.
See [LICENSE.txt](LICENSE.txt)

Among others, we make use of the following libraries:

 * OpenCV ([BSD](https://opencv.org/license.html))
 * CLI11 ([BSD](https://github.com/CLIUtils/CLI11/blob/master/LICENSE))


