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
 * monocular_pose_estimator.cpp
 *
 * Created on: Jul 29, 2013
 * Author: Karl Schwabe
 *
 * Adapted till: April 30 2016
 * Author: Marco Moos
 */

/** \file monocular_pose_estimator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */

#include "pf_mpe/monocular_pose_estimator.h"


namespace monocular_pose_estimator
{

/**
 * Constructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::MPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private), have_camera_info_(false)
{
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  dynamic_reconfigure::Server<pf_mpe::PFMonocularPoseEstimatorConfig>::CallbackType cb_;
  cb_ = boost::bind(&MPENode::dynamicParametersCallback, this, _1, _2);
  dr_server_.setCallback(cb_);

  // Initialize subscribers
  state_obsUAV_sub_ = nh_.subscribe("own_odometry", 1, &MPENode::obsUAVStateCallback, this); //read the state from the observer UAV
  image_sub_ = nh_.subscribe("src_image_raw", 10, &MPENode::imageCallback, this); //read the image data from the camera
  camera_info_sub_ = nh_.subscribe("src_camera_info", 1, &MPENode::cameraInfoCallback, this); //read the camera info
  Vicon_sub_ = nh_.subscribe("ground_truth_transform", 1, &MPENode::ViconPoseCallback, this); // read the Vicon data (optional, only for accuracy testing)


  // Initialize pose publisher
  pose_pub_1 = nh_private_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose_UAV1", 1);
  pose_pub_2 = nh_private_.advertise<geometry_msgs::PoseWithCovarianceStamped>("estimated_pose_UAV2", 1);
  duration_pub_ = nh_private_.advertise<std_msgs::Duration>("timePoseEst", 1);
  duration_pub_1 = nh_private_.advertise<std_msgs::Duration>("timeInitEst", 1);
  particle_pub_1 = nh_private_.advertise<geometry_msgs::PoseArray>("PoseParticles1", 1);
  particle_pub_2 = nh_private_.advertise<geometry_msgs::PoseArray>("PoseParticles2", 1);
  resampled_particle_pub_1 = nh_private_.advertise<geometry_msgs::PoseArray>("ResampledParticles1", 1);
  resampled_particle_pub_2 = nh_private_.advertise<geometry_msgs::PoseArray>("ResampledParticles2", 1);
  FailFlag_pub_ = nh_private_.advertise<std_msgs::Float32MultiArray>("FailFlag", 1);
  
  /*------TIMO------*/
  
  image_pos_pub = nh_private_.advertise<geometry_msgs::PointStamped>("UAV_pixel_pose",1);
  
  /*----------------*/
  
  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_private_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);

  // Create the marker positions from the test points
  List4DPoints positions_of_markers_on_object;
  //List4DPoints positions_of_markers_on_object2; // introduced for multiple UAV's
  std::vector<List4DPoints> positions_of_markers_on_object_vector; // introduced for multiple UAV's; contains the points of the different objects in different list of points


  // Read in the marker positions from the YAML parameter file
  XmlRpc::XmlRpcValue points_list;
  if (!nh_private_.getParam("marker_positions", points_list))
  {
    ROS_ERROR(
        "%s: No reference file containing the marker positions, or the file is improperly formatted. Use the 'marker_positions_file' parameter in the launch file.",
        ros::this_node::getName().c_str());
    ros::shutdown();
  }
  else
  {
      nh_private_.getParam("numUAV",numUAV);
      nh_private_.getParam("numberOfMarkersUAV1",numberOfMarkersUAV1);
      nh_private_.getParam("numberOfMarkersUAV2",numberOfMarkersUAV2);
      nh_private_.getParam("numberOfMarkersUAV3",numberOfMarkersUAV3);
      nh_private_.getParam("numberOfMarkersUAV4",numberOfMarkersUAV4);

      nh_private_.getParam("ref_tf_UAV1",ref_tf_UAV1);


      std::vector<int> numberOfMarkers_vector;
      numberOfMarkers_vector.push_back(numberOfMarkersUAV1);
      numberOfMarkers_vector.push_back(numberOfMarkersUAV2);
      numberOfMarkers_vector.push_back(numberOfMarkersUAV3);
      numberOfMarkers_vector.push_back(numberOfMarkersUAV4);


    int numberOfMarkers_total = 0;
    for (int UAV_nr = 0; UAV_nr<numUAV; UAV_nr++)
    {
      int numberOfMarkers = numberOfMarkers_vector[UAV_nr];
      positions_of_markers_on_object.resize(numberOfMarkers);

      for (int i = 0; i < numberOfMarkers; i++)
      {
	Eigen::Matrix<double, 4, 1> temp_point;
	temp_point(0) = points_list[i+numberOfMarkers_total]["x"];
	temp_point(1) = points_list[i+numberOfMarkers_total]["y"];
	temp_point(2) = points_list[i+numberOfMarkers_total]["z"];
	temp_point(3) = 1;
	positions_of_markers_on_object(i) = temp_point;
      }
      positions_of_markers_on_object_vector.push_back(positions_of_markers_on_object); // introduced for multiple UAV's
      numberOfMarkers_total += numberOfMarkers;
    }

  }
  numObjects = positions_of_markers_on_object_vector.size();
  trackable_object_.setMarkerPositions(positions_of_markers_on_object, positions_of_markers_on_object_vector);
  ROS_INFO("The number of markers on the object are: %d", (int )positions_of_markers_on_object.size());
}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::~MPENode()
{

}

/**
 * The callback function that retrieves the state (pose) from the observer UAV
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MPENode::obsUAVStateCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
  nav_msgs::Odometry P_obsUAV_temp = *pose_msg;

  double x = P_obsUAV_temp.pose.pose.position.x;
  double y = P_obsUAV_temp.pose.pose.position.y;
  double z = P_obsUAV_temp.pose.pose.position.z;
  double qw = P_obsUAV_temp.pose.pose.orientation.w;
  double qx = P_obsUAV_temp.pose.pose.orientation.x;
  double qy = P_obsUAV_temp.pose.pose.orientation.y;
  double qz = P_obsUAV_temp.pose.pose.orientation.z;



  Eigen::Quaterniond q(qw,qx,qy,qz);
  Eigen::Matrix3d R = q.toRotationMatrix();

  trackable_object_.P_obsUAV.block<3,3>(0, 0) = R;
  trackable_object_.P_obsUAV(0,3) = x;
  trackable_object_.P_obsUAV(1,3) = y;
  trackable_object_.P_obsUAV(2,3) = z;
  trackable_object_.P_obsUAV(3,0) = 0;
  trackable_object_.P_obsUAV(3,1) = 0;
  trackable_object_.P_obsUAV(3,2) = 0;
  trackable_object_.P_obsUAV(3,3) = 1;

  trackable_object_.time_obsUAV = pose_msg->header.stamp.toSec();

  //Matrix6d cov = P_obsUAV_temp.pose.covariance;

}



/**
 * callback function which reads and display the pose from the Vicon system (optional, not necessarcy for the algorithm to work; introduced for accuracy check)
 */

void MPENode::ViconPoseCallback(const geometry_msgs::TransformStamped::ConstPtr& pose_msg)
{
  geometry_msgs::TransformStamped pose_info = *pose_msg;
  double x = pose_info.transform.translation.x;
  double y = pose_info.transform.translation.y;
  double z = pose_info.transform.translation.z;
  double qw = pose_info.transform.rotation.w;
  double qx = pose_info.transform.rotation.x;
  double qy = pose_info.transform.rotation.y;
  double qz = pose_info.transform.rotation.z;

  Eigen::Quaterniond q(qw,qx,qy,qz);
  Eigen::Matrix3d R = q.toRotationMatrix();

  Eigen::Matrix4d Pose_Vicon;
  Pose_Vicon.block<3,3>(0, 0) = R;
  Pose_Vicon(0,3) = x;  Pose_Vicon(2,3) = y;  Pose_Vicon(1,3) = z;
  Pose_Vicon(3,0) = 0;  Pose_Vicon(3,1) = 0;  Pose_Vicon(3,2) = 0;  Pose_Vicon(3,3) = 1;

}




/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MPENode::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (!have_camera_info_)
  {
    cam_info_ = *msg;

    // Calibrated camera
    trackable_object_.camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
    trackable_object_.camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
    trackable_object_.camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
    trackable_object_.camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
    trackable_object_.camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
    trackable_object_.camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
    trackable_object_.camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
    trackable_object_.camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
    trackable_object_.camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
    trackable_object_.camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
    trackable_object_.camera_distortion_coeffs_ = cam_info_.D;

    have_camera_info_ = true;
    ROS_INFO("Camera calibration information obtained.");
  }

}

/**
 * The callback function that is executed every time an image is received. It runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void MPENode::imageCallback(const sensor_msgs::CompressedImage::ConstPtr& image_msg)
{
	 // calculate the time needed for the image callback function
	  ros::Time startPE = ros::Time::now();

  // Check whether already received the camera calibration data
  if (!have_camera_info_)
  {
    ROS_WARN("No camera info yet...");
    return;
  }


  // Import the image from ROS message to OpenCV mat
  cv_bridge::CvImagePtr cv_ptr;

  cv::Mat image;
  try
  {
    //cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
 // if (sensor_msgs::image_encodings::isColor(image_msg->encoding))
        {  
     cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
     cv::extractChannel(cv_ptr->image,image,2);//extract red channel      
}
//else
  //     {
    //      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
      //  cv::Mat image = cv_ptr->image;
        //}
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  

  // Get time at which the image was taken. This time is used to stamp the estimated pose and also calculate the position of where to search for the makers in the image
  double time_to_predict = image_msg->header.stamp.toSec();

/*  static double time_to_predict_old = 0; // skip frames. Take only frames which are more than 0.03 sec appart from eachother (decrease frame rate for testing)

  if (time_to_predict_old !=0)
    {
      if (time_to_predict-time_to_predict_old<0.03)
	return;
    }
  time_to_predict_old = time_to_predict;*/
  cv::Point3i ROI_center;
  const std::vector<bool> found_body_pose = trackable_object_.estimateBodyPose(image, time_to_predict, timeInitEst, ROI_center); // TIMO
  
  

  for (int ObjectNumber = 0; ObjectNumber<numObjects; ObjectNumber++)
  {
    if (found_body_pose[ObjectNumber]) // Only output the pose, if the pose was updated (i.e. a valid pose was found).
    {
      //Eigen::Matrix4d transform = trackable_object.getPredictedPose();
      Matrix6d cov = trackable_object_.getPoseCovariance(ObjectNumber);
      Eigen::Matrix4d transform = trackable_object_.getPredictedPose(ObjectNumber).inverse();

      ROS_DEBUG_STREAM("The transform: \n" << transform);
      ROS_DEBUG_STREAM("The covariance: \n" << cov);
      ROS_INFO_STREAM("x pos : "  << ROI_center.x << " y pos : " << ROI_center.y << "\n");
      // Convert transform to PoseWithCovarianceStamped message
      predicted_pose_.header.stamp = image_msg->header.stamp;
      //predicted_pose_.header.frame_id = ref_tf_UAV1;
      
      /* ------ TIMO ------*/
      UAV_pixel_pose_.header.stamp = image_msg->header.stamp;
      UAV_pixel_pose_.point.x = ROI_center.x;
      UAV_pixel_pose_.point.y = ROI_center.y;
      
      image_pos_pub.publish(UAV_pixel_pose_);
      /*___________________*/
      
      
      predicted_pose_.header.frame_id = "world";


      predicted_pose_.pose.pose.position.x = transform(0, 3);
      predicted_pose_.pose.pose.position.y = transform(1, 3);
      predicted_pose_.pose.pose.position.z = transform(2, 3);
      Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.block<3, 3>(0, 0));
      predicted_pose_.pose.pose.orientation.x = orientation.x();
      predicted_pose_.pose.pose.orientation.y = orientation.y();
      predicted_pose_.pose.pose.orientation.z = orientation.z();
      predicted_pose_.pose.pose.orientation.w = orientation.w();

        

      // Add covariance to PoseWithCovarianceStamped message
      for (unsigned i = 0; i < 6; ++i)
      {
	for (unsigned j = 0; j < 6; ++j)
	{
	      predicted_pose_.pose.covariance.elems[j + 6 * i] = cov(i, j);
	}
      }
// Don't publish the particles, they need to much time
      if (bUseParticleFilter)
	{
	  // get the paricles for a visualisation and publish them
	  std::vector<Eigen::Matrix4d> PoseParticles = trackable_object_.getPoseParticles(ObjectNumber);
	  std::vector<Eigen::Matrix4d> ResampledParticles = trackable_object_.getResampledParticles(ObjectNumber);
	  //Eigen::Matrix4d PoseParticles1 = trackable_object_.getPredictedPose();
	  //std::vector<Eigen::Matrix4d> PoseParticles;
	  //PoseParticles.push_back(PoseParticles1);
	  ArrayOfPoses.header.stamp = image_msg->header.stamp;
	  ArrayOfResampledPoses.header.stamp = image_msg->header.stamp;
	  // publish the pose as late as possible --> end o the if clause

	  geometry_msgs::Pose pose1;
	  geometry_msgs::Pose pose2;
	  geometry_msgs::PoseArray dummyArray1;
	  geometry_msgs::PoseArray dummyArray2;
     
	  if (PoseParticles.size() != 0)
	  {
	    for (int i = 0; i<PoseParticles.size(); i++)
	      {
	       // Convert PoseParticles to PoseArray message
		pose1.position.x = PoseParticles[i](0, 3);
		pose1.position.y = PoseParticles[i](1, 3);
		pose1.position.z = PoseParticles[i](2, 3);
		Eigen::Quaterniond orientation1 = Eigen::Quaterniond(PoseParticles[i].block<3, 3>(0, 0));
		pose1.orientation.x = orientation1.x();
		pose1.orientation.y = orientation1.y();
		pose1.orientation.z = orientation1.z();
		pose1.orientation.w = orientation1.w();

	       dummyArray1.poses.push_back(pose1);

	       pose2.position.x = ResampledParticles[i](0, 3);
	       pose2.position.y = ResampledParticles[i](1, 3);
	       pose2.position.z = ResampledParticles[i](2, 3);
	       Eigen::Quaterniond orientation2 = Eigen::Quaterniond(ResampledParticles[i].block<3, 3>(0, 0));
	       pose2.orientation.x = orientation2.x();
	       pose2.orientation.y = orientation2.y();
	       pose2.orientation.z = orientation2.z();
	       pose2.orientation.w = orientation2.w();

	       dummyArray2.poses.push_back(pose2);
	      }

	    ArrayOfPoses.poses = dummyArray1.poses;
	    //particle_pub_.publish(ArrayOfPoses);

	    ArrayOfResampledPoses.poses = dummyArray2.poses;
	    //resampled_particle_pub_.publish(ArrayOfResampledPoses);

	    /*// publish particles and resamples
	    if (ObjectNumber == 0)
		    {particle_pub_1.publish(ArrayOfPoses); resampled_particle_pub_1.publish(ArrayOfResampledPoses);}
	    else if (ObjectNumber == 1)
		    {particle_pub_2.publish(ArrayOfPoses); resampled_particle_pub_2.publish(ArrayOfResampledPoses);}*/
	  }

	}
ROS_INFO_STREAM("pre The transform: e\n" );
	  // Publish the pose (publish it as late as possible, close to the publishing of the FailFlag and the EstimationTime)
	  if (ObjectNumber == 0)
      {

		  pose_pub_1.publish(predicted_pose_);
          tf::Transform predicted_tf;
          tf::poseMsgToTF(predicted_pose_.pose.pose,predicted_tf);
          br_.sendTransform(tf::StampedTransform(predicted_tf, predicted_pose_.header.stamp, "world", "icarus"));
          ROS_INFO_STREAM("The transform: icarus\n" );
      }
	  else if (ObjectNumber == 1)
		  pose_pub_2.publish(predicted_pose_);



    }
    else
    {

    }
  }
  
  PoseEstimator::PubData_struct PubData = trackable_object_.getPublisherData();

  
  FailFlag.data = {PubData.Flag_Fail[0], PubData.Flag_Fail[1]};
  FailFlag_pub_.publish(FailFlag);

  
  // publish visualization image
  if (image_pub_.getNumSubscribers() > 0)
  {
    // uncomment if bw image desired
/*
    cv::Mat bw_image;
    cv::threshold(image, bw_image, image_threshold, 255, cv::THRESH_BINARY_INV); //threshold_value = 100;
    cv::Mat gaussian_image;
    cv::Size ksize; // Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
    ksize.width = 0;
    ksize.height = 0;
    //GaussianBlur(bw_image.clone(), gaussian_image, ksize, 0.6, 0.6, cv::BORDER_DEFAULT); // sigma = 0.6
    std::vector<std::vector<cv::Point> > contours;
    //cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::findContours(bw_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //cv::Mat visualized_image = gaussian_image.clone();
    cv::Mat visualized_image = bw_image.clone();
*/


    // uncomment if real image desired
    cv::Mat visualized_image = image.clone();

    cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);

    // diplay the image
    trackable_object_.augmentImage(visualized_image,found_body_pose);

    // Publish image for visualization
    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.header = image_msg->header;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = visualized_image;

    image_pub_.publish(visualized_image_msg.toImageMsg());



  }

  // calculate the time needed for the image callback function AND the visualisation (this will be excluded in the final tests)
  ros::Time endPE = ros::Time::now();
  timePoseEst.data = endPE-startPE;

  // publish the duration of the function
  duration_pub_.publish(timePoseEst);
  // publish the duration of the initialisation
  duration_pub_1.publish(timeInitEst);


}

/**
 * The dynamic reconfigure callback function. This function updates the variable within the program whenever they are changed using dynamic reconfigure.
 */
void MPENode::dynamicParametersCallback(pf_mpe::PFMonocularPoseEstimatorConfig &config, uint32_t level)
{
  trackable_object_.detection_threshold_value_ = config.threshold_value;
  trackable_object_.gaussian_sigma_ = config.gaussian_sigma;
  trackable_object_.min_blob_area_ = config.min_blob_area;
  trackable_object_.max_blob_area_ = config.max_blob_area;
  trackable_object_.max_width_height_distortion_ = config.max_width_height_distortion;
  trackable_object_.max_circular_distortion_ = config.max_circular_distortion;
  trackable_object_.roi_border_thickness_ = config.roi_border_thickness;

  trackable_object_.setBackProjectionPixelTolerance(config.back_projection_pixel_tolerance);
  trackable_object_.setNearestNeighbourPixelTolerance(config.nearest_neighbour_pixel_tolerance);
  trackable_object_.setCertaintyThreshold(config.certainty_threshold);
  trackable_object_.setValidCorrespondenceThreshold(config.valid_correspondence_threshold);
  
  trackable_object_.number_of_false_detections = config.number_of_false_detections;
  trackable_object_.number_of_occlusions = config.number_of_occlusions;
  trackable_object_.bUseParticleFilter = config.bUseParticleFilter;
  trackable_object_.N_Particle = config.N_Particle;
  trackable_object_.maxAngularNoise = config.maxAngularNoise;
  trackable_object_.minAngularNoise = config.minAngularNoise;
  trackable_object_.maxTransitionNoise = config.maxTransitionNoise;
  trackable_object_.minTransitionNoise = config.minTransitionNoise;
  trackable_object_.back_projection_pixel_tolerance_PF = config.back_projection_pixel_tolerance_PF;
  trackable_object_.active_markers = config.active_markers;
  trackable_object_.useOnlineExposeTimeControl = config.useOnlineExposeTimeControl;
  trackable_object_.expose_time_base = config.expose_time_base;
  trackable_object_.bUseCamPos = config.bUseCamPos;

  // Define vector which contains the downgraded markers
  std::vector<bool> bMarkerDowngrade;
  bMarkerDowngrade.push_back(config.bMarkerNr1);
  bMarkerDowngrade.push_back(config.bMarkerNr2);
  bMarkerDowngrade.push_back(config.bMarkerNr3);
  bMarkerDowngrade.push_back(config.bMarkerNr4);
  bMarkerDowngrade.push_back(config.bMarkerNr5);

  trackable_object_.bMarkerDowngrade = bMarkerDowngrade; // trackable_object_.variablename = variablename  is not working, therefore MarkerDowngradeGen was introduced above

  // Parameter for the displayed image (image with detections)
  image_threshold = config.threshold_value;

  // Boolean which tells if the particle filter is used or not
  bUseParticleFilter = config.bUseParticleFilter;


  ROS_INFO("Parameters changed");
}

} // namespace monocular_pose_estimator
