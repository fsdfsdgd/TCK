# TCK-VILO:A Tightly-Coupled and Keyframe-Based Visual-Inertial-Lidar Odometry System for Mobile Robots With Adaptive Sensor Reliability Evaluation


## 1. Required Dependencies

### boost
c++ Librairies (thread and system are needed). Install with
	
		sudo apt-get install libboost-all-dev

### Eigen3
Linear algebra.

		sudo apt-get install libeigen3-dev

### OpenCV3
Dowload and install instructions can be found at: http://opencv.org.


### Pangolin
Used for 3D visualization & the GUI.
Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## 2. Build and Compile

		cd vilo
		chmod +x build.sh
		./build.sh

This will create the executable `test_vilo` in *bin* folder.

## 3. Run code

		cd bin
                ./test_vilo \
                lidars=PATH_TO_LIDAR_FOLDER \
                images=PATH_TO_IMAGE_FOLDER \
                imus=PATH_TO_IMU_FOLDER/imu.txt \
                calib=PATH_TO_CALIBRATION_FILE/newer_college.yaml


- `lidars=XXX` where XXX is the folder containing lidar data(both .txt and .bin are ok).
- `images=XXX` where XXX is the folder containing images.
- `imus=XXX` where XXX is the files of imu data (see the *imu.txt* file in *example* folder).
- `calib=XXX` where XXX is a calibration file (see the *newer_college.yaml* file in *example* folder).


		
