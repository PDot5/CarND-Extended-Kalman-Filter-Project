# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

This project will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

### Programs Need:
To accomplish this projects you will need:
- src/FusionEKF.cpp
- src/FusionEKF.h
- kalman_filter.cpp
- kalman_filter.h, tools.cpp
- tools.h

The program main.cpp has already been filled out.

### UWebSocketsIO:
Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Project output:

Here is a screenshot of the output of Dataset 1: (A movie of how the simulator performed will also be uploaded to the repository: Movie ExtendedKF Dataset 1)
![Dataset 1](https://raw.githubusercontent.com/PDot5/CarND-Extended-Kalman-Filter-Project/master/build/Sim_Images/Dataset_1.png)

Here is a screenshot of the output of Dataset 2: (A movie of how the simulator performed will also be uploaded to the repository: Movie ExtendedKF Dataset 2)
![Dataset 2](https://raw.githubusercontent.com/PDot5/CarND-Extended-Kalman-Filter-Project/master/build/Sim_Images/Dataset_2.png)

## Additional:
### Xcode
In order to create an xcode file, a separate build-xcode file was created. Using previous build instructions, you can modify it slightly to generate the xcode.proj file in order to run your code with xcode: In my case, I made a build-xcode directory inside of the build directory, so ../.. would need to be used in order to access thte CMakelist.
1. Clone this repo.
2. Make a build directory: `mkdir build-xcode && cd build-xcode`
3. Compile: `cmake ../.. -GXcode && make` 
4. Run it: `open ExtendedKF.xcodeproj`
(This will open up Xcode and you can build and execute the program).


## Useful cmds for Mac:

- brew install openssl libuv cmake zlib
- git clone https://github.com/uWebSockets/uWebSockets 
- cd uWebSockets
- git checkout e94b6e1
- patch CMakeLists.txt < ../cmakepatch.txt
- mkdir build
- export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
- cd build
- OPENSSL_VERSION=`brew list --versions openssl | cut -d' ' -f2`
- cmake -DOPENSSL_ROOT_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION -DOPENSSL_LIBRARIES=$(brew --cellar openssl)/$OPENSSL_VERSION/lib ..
- make 
- sudo make install
- cd ..
- cd ..
- sudo rm -r uWebSockets

### References: 

https://www.youtube.com/watch?v=TpQv0k2ZQjo
https://stackoverflow.com/questions/16700415/cmake-generate-xcode-project-from-existing-sources
https://github.com/jeremy-shannon/CarND-Extended-Kalman-Filter-Project/blob/master/README.md#Call%20for%20IDE%20Profiles%20Pull%20Requests
https://www.haidynmcleod.com/extended-kalman-filter
https://github.com/jessicayung/self-driving-car-nd
https://medium.com/@serrano_223/extended-kalman-filters-for-dummies-4168c68e2117
https://github.com/Heych88/udacity-sdcnd-extended-kalman-filter
https://cmake.org/install
https://tuannguyen68.gitbooks.io/learning-cmake-a-beginner-s-guide/content/chap1/chap1.html

