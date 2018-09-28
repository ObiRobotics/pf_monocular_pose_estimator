##

PF-MPE: PARTICLE-FILTER-ENHANCED MONOCULAR POSE ESTIMATION
======================

This code is heavily based on https://github.com/uzh-rpg/rpg_monocular_pose_estimator. In this extension, the main contribution is a particle filter in the tracking algorithm. 
Our version can handle plenty of outliers detection while keeps the tracking in real-time.
This extension was developed by Marco Moos during his master thesis at ETH Zurich in 2016.

Disclaimer and License
---------------

The PF-MPE has been tested under the following setups:

* ROS-Indigo and Ubuntu 14.04

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

The source code is released under a GPLv3 licence and Commercial use is not permitted.


Package Summary
---------------

The PF-MPE uses RGB or IR LEDs , we used Adafruit NeoPixel LEDs, mounted on a target object, and a monocular camera to estimate the pose of the target object.

The positions of the LEDs on the target object are provided by the user in a YAML configuration file. The LEDs are detected in the image and a pose estimate for the target object is subsequently calculated.

### Publications

By default this code is correspont to the following publication

L. Teixeira and F. Maffra and M. Moos and M. Chli: 
**VI-RPE: Visual-Inertial Relative Pose Estimation for Aerial Vehicles**
IEEE Robotics and Automation Letters (RA-L), Spain, 2018.

    @ARTICLE{Teixeira18ral, 
      author={L. Teixeira and F. Maffra and M. Moos and M. Chli}, 
      journal={IEEE Robotics and Automation Letters}, 
      title={VI-RPE: Visual-Inertial Relative Pose Estimation for Aerial Vehicles}, 
      year={2018}, 
      volume={3}, 
      number={4}, 
      pages={2770-2777}, 
      doi={10.1109/LRA.2018.2837687},
      month={Oct}
}

If you disable the particle filter, please cite the following [ICRA 2014 publication](http://rpg.ifi.uzh.ch/docs/ICRA14_Faessler.pdf):

M. Faessler, E. Mueggler, K. Schwabe, D. Scaramuzza: 
**A Monocular Pose Estimation System based on Infrared LEDs.**
IEEE International Conference on Robotics and Automation (ICRA), Hong Kong, 2014.

    @InProceedings{Faessler14icra,
      author        = {Matthias Faessler and Elias Mueggler and Karl Schwabe and
                      Davide Scaramuzza},
      title         = {A Monocular Pose Estimation System based on Infrared {LED}s},
      booktitle     = {{IEEE} Int. Conf. Robot. Autom. (ICRA)},
      year          = 2014,
      pages         = {907--913},
      doi           = {10.1109/ICRA.2014.6906962}
    }




Installation
------------

### Installation of the package

#### Dependencies

The PF-MPE is built on the Robotic Operating System (ROS). In order to install the package, ROS has to be installed.

- In order to install the Robot Operating System (ROS), please follow the instructions provided in the [link](http://wiki.ros.org).
- Make sure you have properly set up a ROS catkin workspace as described [here](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

The two folling ros packages need to be download into the catkin workspace:
- git clone git@github.com:catkin/catkin_simple.git
- git clone git@github.com:ethz-asl/eigen_catkin.git

#### Main Installation

In order to install the PF-MPE, clone this repository inside your catkin workspace and compile the package using ROS catkin.


Basic Usage
-----------

The PF-MPE makes use of a number of parameters to estimate the pose of the tracked object. These include the location of the LEDs on the object, the intrinsic parameters of the camera, and various runtime parameters, such as thresholding values.

#### Setting up the Marker Positions Parameter File

In order to predict the pose of the target object, the Pose Estimator needs to know the positions of the LEDs on the object in the object's frame of reference. These positions are given to the program using a YAML file located within the `roscd pf_mpe/marker_positions` folder.

The file is called `nameOfTheMarkerPositionFile.YAML` and has the following format.

    # The marker positions in the trackable's frame of reference:
    #
    marker_positions:
  	- x:  0.000  
  	  y:  0.000 
  	  z:  0.000
  	- x:  0.004
  	  y: -0.188
  	  z:  0.039
  	- x:  0.228
  	  y: -0.140
  	  z:  0.000
  	- x:  0.264
  	  y:  0.124
 	  z:  0.000
  	- x:  0.076
  	  y:  0.128
  	  z:  0.005

**Note** that each new LED position is indicated by a dash (-). The position is given in the x, y, and z coordinates of the object frame in meters.

If you would like to use your own marker positions file, place it in the `pf_mpe/marker_positions` folder and alter the launch file as explained below in the section 'Launch Files'.

#### Running the pose estimator with a USB camera 

Ensure that you have a working USB camera.

The camera needs to be calibrated. Follow the instructions at http://www.ros.org/wiki/camera_calibration/Tutorials/MonocularCalibration.

The pose estimator listens to the mv_25001329/image_raw topic and the mv_25001329/camera_info topic of the camera at launch. An example of such a camera_info topic is:
    
    header: 
      seq: 4525
      stamp: 
        secs: 1452243541
        nsecs: 483605772
      frame_id: camera_mv
    height: 480
    width: 752
    distortion_model: plumb_bob
    D: [-0.2987656130625547, 0.090512327786479, 0.0006983134447049677, 0.0004069824038616868, 0]
    K: [475.4220992391843, 0, 378.1094270899403, 0, 475.6638941565314, 236.6226272309063, 0, 0, 1]
    R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    P: [376.2829284667969, 0, 379.3125311122712, 0, 0, 435.74609375, 235.8427641757298, 0, 0, 0, 1, 0]

The camera should be adjusted so that the gain and shutter/exposure time of the camera are fixed. (Use 'rqt_reconfigure'  to adjust the camera parameters while the camera is running). Ideally, the LEDs should appear very bright in the image so that they can easily be segmented from the surrounding scene by simple thresholding. See 'Parameter Settings' below on how to change the thresholding value.

#### Inputs, Outputs

##### Configuration Files

The PF-MPE requires the LED positions on the target object. These are entered in a YAML file and are loaded at runtime. See 'Setting up the Marker Positions Parameter File' above for more details.

##### Subscribed Topics

The PF-MPE subscribes to the following topics:

* src_image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  The image form the camera. The LEDs will be detected in this image. 

* src_camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

  The camera calibration parameters.

* own_odometry ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)) 

  In case there is second source of odometry about the robot carrying the camera. If you only have one target, you can also feed the inverse transformation of the target odometry.

* ground_truth_transform 
  
  This topic is only used to evaluation as reference. It does not affect our algorithm.

##### Published Topics

The PF-MPE publishes the following topics:

* estimated_pose_UAV1 ([geometry_msgs/PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

     The estimated pose of the target object with respect to the camera. (in case of multiple UAVs: estimated_pose_UAV1, estimated_pose_UAV2, ...)

* image_with_detections ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

  	The image with the detected LEDs circled in green, the region of interest of the image that was processed is bounded by a yellow rectangle and the orientation trivectors (from all particles) projected onto the object.

* PoseParticles1 ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

	The pose of the (not resampled) particles with respect to the camera (in case of multiple UAVs: PoseParticles1, PoseParticles2, ...)

* ResampledParticles1  ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))

	The pose of the resampled particles with respect to the camera (in case of multiple UAVs: ResampledParticles1, ResampledParticles2, ...)

* FailFlag ([std_msgs/Float32MultiArray](http://docs.ros.org/api/std_msgs/html/msg/Float32MultiArray.html))

	Fail flag (explained in monocular_pose_estimator.hpp)

* timePoseEst ([std_msgs/Duration](http://docs.ros.org/api/std_msgs/html/msg/Duration.html))

	Time needed for processing the frame

* timeInitEst ([std_msgs/Duration](http://docs.ros.org/api/std_msgs/html/msg/Duration.html))

	Time needed for the initialisation




#### Parameter Settings

The following parameters can be set dynamically during runtime. (Use 'rqt_reconfigure' to adjust the parameters).


* threshold_value (int, default: 240, min: 0, max: 255)

  This is the pixel intensity that will be used to threshold the image. All pixels with intensity values below this value will be set to zero. Pixels with intensity values equal to or higher than this value will retain their intensities.

* gaussian_sigma (double, default: 0.6, min: 0, max: 6)

  This is the standard deviation of the of the Gaussian that will be used to blur the image after being thresholded.

* min_blob_area (double, default: 20, min: 0, max: 100)

  This is the minimum blob area (in pixels squared) that will be detected as a cluster/LED at a distance of 1 meter. Clusters having an area smaller than this will not be detected as LEDs.

* max_blob_area (double, default: 160, min: 0, max: 1000)

  This is the maximum blob area (in pixels squared) that will be detected as a cluster/LED at a distance of 1 meter. Clusters having an area larger than this will not be detected as LEDs.

* max_width_height_distortion (double, default: 0.7, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected clusters. It is the maximum allowable distortion of a bounding box around the detected cluster calculated as the ratio of the width to the height of the bounding rectangle. Ideally the ratio of the width to the height of the bounding rectangle should be 1. 

* max_circular_distortion (double, default: 0.7, min: 0, max: 1)

  This is a parameter related to the circular distortion of the detected clusters. It is the maximum allowable distortion of a bounding box around the detected cluster, calculated as the area of the cluster divided by pi times half the height or half the width of the bounding rectangle. 

* back_projection_pixel_tolerance (double, default: 5, min: 0, max: 100)

  This is the tolerance (in pixels) between a back projected LED and an image detection. If a back projected LED is within this threshold, then the pose for that back projection is deemed to be a possible candidate for the UAV pose.

* nearest_neighbour_pixel_tolerance (double, default: 7, min: 0, max: 10)

  This is the tolerance for the prediction of the correspondences between object LEDs and image detections. If the predicted position of an LED in the image is within this tolerance of the image detections, then the LED and image detection are considered to correspond to each other.

* certainty_threshold (double, default: 1, min: 0, max: 1)

  This is the proportion of back projected LEDs into the image that have to be within the back_projection_pixel_tolerance, for a pose to be considered valid.

* valid_correspondence_threshold (double, default: 0.5, min: 0, max: 1)

  This is the ratio of all combinations of 3 of the correspondences that yielded valid poses (i.e., were within the certainty_threshold), for a set of correspondences to be considered valid.

* roi_border_thickness (int, default: 10, min: 10, max: 200)

  This is the thickness of the boarder around the region of interest in pixels.

* number_of_occlusions (int, default: 0, min: 0, max: 5)

  This is the number of occlusion which can be introduced to check the Pose Estimator due to its robustness against occlusions. (The occlusions are randomly applied.)

* number_of_false_detections (int, default: 0, min: 0, max: 30)
	
  This is the number of false detections which can be introduced to check the Pose Estimator due to its robustness against false detections (noise). (They are randomly applied inside the region of interest.)

* active_markers (bool, default: true)

  This parameter defines if active or passive markers are used.

* bUseParticleFilter (bool, default: true)

  This parameter defines if the particle filter approach is used, or if the IPE from Faessler et al. (ICRA 2014) is used.

* N_Particle (int, default: 1, min: 1, max: 20000)

  This is the number of particles which is used in the particle filter. (Should not be changed online!!!!)

* maxAngularNoise (int, default: 0, min: -1.5, max: 1.5)

  This is the maximum angular noise which is used in the motion model.

* minAngularNoise (int, default: 0, min: -1.5, max: 1.5)

  This is the minimal angular noise which is used in the motion model.

* maxTransitionNoise (int, default: 0, min: -0.5, max: 0.5)

  This is the maximum transition noise which is used in the motion model.

* minTransitionNoise (int, default: 0, min: -0.5, max: 0.5)

  This is the minimal transition noise which is used in the motion model.

* back_projection_pixel_tolerance_PF (int, default: 10, min: 0, max: 100)

  This is the tolerance (in pixels) between a back projected LED and an image detection during the weighting phase in the particle filter. If a back projected LED is within this threshold, the weight of the pose will be increased.

* bMarkerNrX (bool, default: false)

  This parameter tells if the marker number X is downgraded or not. (Can only be used if one UAV is tracked.)

* useOnlineExposeTimeControl (bool, default: false)

  If this parameter is true, the exposure time will be adapted online. A change takes about 1ms, and is therefore not recommended.

* expose_time_base (int, default: 2000, min: 10, max: 8000

  This is the initial expose time which is used in case the expose time is calculated online.

* bUseCamPos (bool, default: false)

  This boolean tells if the movement of the camera is considered in the pose prediction or not. (Should only be used if reliable information is available.)



#### Launch Files

The Pose Estimator needs to be launched with a launch file, since the location of the YAML configuration file containing the LED positions on the object needs to be specified.
An example launch file is presented below.


    <launch> 

	   <!-- Name of the YAML file containing the marker positions -->
	   <arg name="YAML_file_name" default="nameOfTheMarkerPositionFile"/>

	   <!-- File containing the the marker positions in the trackable's frame of reference -->
	   <arg name="marker_positions_file" default="$(find pf_mpe)/marker_positions/$(arg YAML_file_name).yaml"/> 

	   <group ns="pf_mpe" >
	       	<node name="pf_mpe" pkg="pf_mpe" type="pf_mpe" output="screen" respawn="false">  
			 <rosparam command="load" file="$(arg marker_positions_file)"/>
			 <param name= "threshold_value" value = "240" />
			 <param name= "gaussian_sigma" value = "0.6" /> 
			 <param name= "min_blob_area" value = "20" />
			 <param name= "max_blob_area" value = "160" />
			 <param name= "max_width_height_distortion" value = "0.7" />
			 <param name= "max_circular_distortion" value = "0.7" />
			 <param name= "back_projection_pixel_tolerance" value = "5" /> 
			 <param name= "nearest_neighbour_pixel_tolerance" value = "7" /> 
			 <param name= "certainty_threshold" value = "1" />
			 <param name= "valid_correspondence_threshold" value = "0.5" /> 
			 <param name= "number_of_occlusions" value = "0" />
			 <param name= "number_of_false_detections" value = "0" />
			
			 <!-- UAV SPECIFICATIONS -->													
			 <param name= "active_markers" value = "True" />							
			 <param name= "numUAV" value = "1" />										
			 <param name= "numberOfMarkersUAV1" value = "5" />
			 <param name= "numberOfMarkersUAV2" value = "0" />
			 <param name= "numberOfMarkersUAV3" value = "0" />
			 <param name= "numberOfMarkersUAV4" value = "0" />
			
			
			 <!-- PARTICLE FILTER PARAMETER -->
			 <param name= "bUseParticleFilter" value = "True" />
			 <param name= "N_Particle" value = "100" />
			 <param name= "maxAngularNoise" value = "0.015" /> 		
			 <param name= "minAngularNoise" value = "-0.015" />		
			 <param name= "maxTransitionNoise" value = "0.035" />		
			 <param name= "minTransitionNoise" value = "-0.035" />	
			 <param name= "back_projection_pixel_tolerance_PF" value = "4" /> 
			
			
			 <!-- MARKER DOWNGRADE -->
			 <param name= "bMarkerNr1" value = "False" />
			 <param name= "bMarkerNr2" value = "False" />
			 <param name= "bMarkerNr3" value = "False" />
			 <param name= "bMarkerNr4" value = "False" />
			 <param name= "bMarkerNr5" value = "False" />
			
			 <!-- MISCELLANEOUS -->
			 <param name= "useOnlineExposeTimeControl" value = "False" />			
			 <param name= "expose_time_base" value = "2000" />		 
			 <param name= "bUseCamPos" value = "False" />			
		  </node> 
	
	   <node name="view_visualisation_image" pkg="image_view" type="image_view" args="image:=/pf_mpe/image_with_detections" />

	   </group>

    </launch>

In case the image and the camera info are coming from a bagfile, the file has to be added with the following lines
	<!-- rosbag play -->
	  <node pkg="rosbag" type="play" name="player" args=" path/of/the/bagfile/fileName.bag "/>

In case the some topics have to be recored the following lines can be added
	<!-- rosbag record -->
	 <node pkg="rosbag" type="record" name="record" args="-e 'topics/to/record' -o path/where/the/bagfile/will/be/saved/fileName.bag"/>
			


Hardware
--------

The PF-MPE makes use of casual LEDs mounted on the target object and a monocular camera . The details of the LEDs and camera that were used in our evaluation of the package are outlined below. The system will be able to work with any kind of LEDs and camera, provided that the LEDs appear bright in the image and can consequently be segmented from the image using simple thresholding operations. Also the emission pattern of the LEDs should be as wide as possible, so that they still appear bright in the image even when viewed from shallow viewing angles.


### LEDs

The LEDs that were used were NeoPixel LEDs. We used single and ring PCBs.


#### LED Configuration

The placement of the LED on the UAV can be more or less arbitary. Configurations which should be avoided contain symmetries or more than two LEDs on the same line. To improve the accuracy it is benefitial if the LEDs span a big volume and are not collinear. In the initialsation phase the PF-MPE requires at least four LEDs to be visible. Afterwards the UAV can be tracked if at least one LED is visible, but the less LEDs are visible, the less accurate is the estimated pose. 


### Camera

The camera used was a [MatrixVision](http://www.matrix-vision.com/) mvBlueFOX cameras.


Important hints
---------------

#### Tracking multiple UAVs
In the case when multiple UAVs are tracked the marker position file and the UAV specifications in the launchfile have to be changed. The position of the markers have to be defined as one UAV with more LEDs would be used.

    e.g.marker_positions:

    - x: 0.000	# start  of the first UAV
      y: 0.000
      z: 0.000
    - x: 0.123
      y: 0.000
      z: 0.000
    - x: 0.251
      y: 0.000
      z: 0.000
    - x: 0.12644
      y: 0.200454
      z: 0.000
    - x: 0.20644  
      y: 0.128304
      z: 0.130
    - x: 0.000	# start of the second UAV
      y: 0.000
      z: 0.000
    - x: 0.245
      y: 0.000
      z: 0.000
    - x: 0.245
      y: 0.250
      z: 0.000
    - x: 0.495
      y: 0.250
      z: 0.000
    - x: 0.245  
      y: 0.000
      z: 0.245

#### Exposure time
When new video-bagfiles are recorded or the pose estimator is used for an online application, the exposure time has to be adjusted according to the environment, high in dark environments and low in very bright ones (e.g. sunny day outdoors). Consider that a too small exposure time may lead to tracking losses in bigger distances, while a too big one can lead to a very noisy environment and to tracking losses at closer distances.


#### Parameter changes
Some parameter change can have a big impact on the accuracy and the needed estimation time:

* Number of particles

* min/max noise in the motion model

* min/max cluster size (detection stage)

Especially when changing the applied noise in the motion model, be aware that using high values will reduce the influence of the linear prediction and let the noise dominate the pose prediction.


