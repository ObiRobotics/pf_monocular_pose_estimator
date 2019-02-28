// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.
/*
 * monocular_pose_estimator_node.h
 *
 *  Created on: Mar 26, 2014
 *      Author: Matthias Fässler
 *
 * Adapted till: April 30 2016
 * Author: Marco Moos
 */

/** \file monocular_pose_estimator_node.h
 * \brief File containing the definition of the Monocular Pose Estimator Node class
 *
 */

#ifndef MONOCULAR_POSE_ESTIMATOR_NODE_H_
#define MONOCULAR_POSE_ESTIMATOR_NODE_H_

#include "ros/ros.h"
#include "ros/console.h"

#include "std_msgs/Duration.h"
//#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <tf/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>
#include <pf_mpe/PFMonocularPoseEstimatorConfig.h>

#include "pf_mpe_lib/pose_estimator.h"

namespace monocular_pose_estimator
{

class MPENode
{
public:
	int numObjects; // number of objects due to total number of markers

private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  int numUAV; // number of used UAV's
  int numberOfMarkersUAV1;
  int numberOfMarkersUAV2;
  int numberOfMarkersUAV3;
  int numberOfMarkersUAV4;

  std::string ref_tf_UAV1;

  // defines publisher
  image_transport::Publisher image_pub_; //!< The ROS image publisher that publishes the visualisation image
  ros::Publisher pose_pub_1; //!< The ROS publisher that publishes the estimated pose of the first UAV.
  ros::Publisher pose_pub_2; //!< The ROS publisher that publishes the estimated pose of the second UAV.
  ros::Publisher duration_pub_; // displays the time needed to estimate the pose
  ros::Publisher duration_pub_1; // displays the time needed to initialise the pose
  ros::Publisher particle_pub_1; // displays the estimated particles
  ros::Publisher particle_pub_2; // displays the estimated particles
  ros::Publisher resampled_particle_pub_1; // displays the resampled particles
  ros::Publisher resampled_particle_pub_2; // displays the resampled particles
  //TIMO
  ros::Publisher image_pos_pub;
  
  //ros::Publisher bEstimation_pub_; // tells if an estimation could have been calculated or not
  //ros::Publisher bPrediction_pub_; // tells if the prediction or the brute force algorithm was used
  ros::Publisher statistics_pub_;
  ros::Publisher FailFlag_pub_;
  tf::TransformBroadcaster br_;
  
  // define subscribers
  ros::Subscriber image_sub_; //!< The ROS subscriber to the raw camera image
  ros::Subscriber camera_info_sub_; //!< The ROS subscriber to the camera info
  ros::Subscriber state_obsUAV_sub_; //!< The ROS subscriber to the pose from the observer UAV
  ros::Subscriber Vicon_sub_; //!< The ros subscriber to the estimated UAV pose from the camera

  dynamic_reconfigure::Server<pf_mpe::PFMonocularPoseEstimatorConfig> dr_server_; //!< The dynamic reconfigure server
  //dynamic_reconfigure::Server<pf_mpe::PFMonocularPoseEstimatorConfig>::CallbackType cb_; //!< The dynamic reconfigure callback type

  // define messages
  geometry_msgs::PoseWithCovarianceStamped predicted_pose_; //!< The ROS message variable for the estimated pose and covariance of the treated UAV
  
  //TIMO
  geometry_msgs::PointStamped UAV_pixel_pose_;
  
  std_msgs::Duration timePoseEst; // time needed to do the pose estimation
  std_msgs::Duration timeInitEst; // time needed to initialise the pose
  //std_msgs::Bool bEstimation; // boolean which tells if an estimation could be calculated or not
  //std_msgs::Bool bPrediction; // tells if the prediction or the brute force algorithm was used (1: Prediction;  0: Brute force)
  geometry_msgs::PoseArray ArrayOfPoses; // ROS message variable for the array of predicted poses
  geometry_msgs::PoseArray ArrayOfResampledPoses; // ROS message variable for the array of resampled poses
  std_msgs::Float64MultiArray statistics; // test for multiple boolean array
  std_msgs::Float32MultiArray FailFlag;
	  // 0:	 the initialisation succeeded
	  // 1:  the particle filter succeded
	  // 2:  the reinitialisation (short P3P) succeeded
	  // 3:  not enough LED could be found for the initialisation
	  // 4:  no estimated particle was reasonable
	  // 5:  the uncertainty reached the threshold level --> estimation can not be trusted
	  // 6:  correspondence vector at the beginning of the checkCorr fcn has not enough entries (less than 4 --> no P3P possible)
	  // 7:  not enough valid correspondences could be calculated (checkCorr)
	  // 8:  all estimations failed at the certainty threshold (checkCorr)
	  // 9:	 the P3P algorithm failed (checkCorr)
	  // 10: not engough markers could be found for the initialisation (init)
	  // 11: no correspondence from the histogram could be optained (init)
	  // 12: histogram failed completely, all entries are zero (init)
	  // 13: not engough markers could be found for the initialisation (Reinit; short P3P)
	  // 14: no correspondence from the histogram could be optained (Reinit; short P3P)
	  // 15: histogram failed completely, all entries are zero (Reinit; short P3P)
	  // 16: P3P algorithm failed (reinit; short P3P)

  
  bool have_camera_info_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

  PoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated


  int image_threshold; // threshold value for the displed image (image with detections)

  bool bUseParticleFilter; // tells if the particle filter is used or not

public:

  MPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  MPENode() : MPENode( ros::NodeHandle(), ros::NodeHandle("~") ){}
  ~MPENode();

  void obsUAVStateCallback(const nav_msgs::Odometry::ConstPtr& pose_msg);

  void ViconPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose_msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

  void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg);

  void dynamicParametersCallback(pf_mpe::PFMonocularPoseEstimatorConfig &config, uint32_t level);
};

} // monocular_pose_estimator namespace

#endif /* MONOCULAR_POSE_ESTIMATOR_NODE_H_ */
