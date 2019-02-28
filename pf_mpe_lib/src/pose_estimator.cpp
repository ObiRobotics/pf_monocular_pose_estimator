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
 double  * PoseEstimator.cpp
 *
 *  Created on: July 29, 2013
 *      Author: Karl Schwabe
 *
 *  Adapted till: April 30 2016
 *  	Author: Marco Moos
 */

/**
 * \file pose_estimator.cpp
 * \brief File containing the definitions of the functions for the PoseEstimator class
 *
 */

#include "pf_mpe_lib/pose_estimator.h"
#include "ros/ros.h"

namespace monocular_pose_estimator
{

PoseEstimator::PoseEstimator()		// is not called
{
  back_projection_pixel_tolerance_ = 3;
  nearest_neighbour_pixel_tolerance_ = 5;
  certainty_threshold_ = 1;//.75;
  valid_correspondence_threshold_ = 0.7;

  it_since_initialized_ = 0;
}

void PoseEstimator::augmentImage(cv::Mat &image, std::vector<bool> found_body_pose)
{
  Visualization::createVisualizationImage(image, predicted_pose_Vec, camera_matrix_K_, camera_distortion_coeffs_,
                                          region_of_interest_Vec, distorted_detection_centers_Vec, number_of_false_detections,
					  number_of_occlusions, found_body_pose, N_Particle, PoseParticle_Vec, probPart_Vec,
					  bUseParticleFilter, object_points_Vec, region_of_interest_ellipse);
}

void PoseEstimator::setMarkerPositions(List4DPoints positions_of_markers_on_object, std::vector<List4DPoints> positions_of_markers_on_object_vector)
{
  object_points_Vec = positions_of_markers_on_object_vector;
  object_points_ = positions_of_markers_on_object;
  predicted_pixel_positions_.resize(object_points_.size());
  histogram_threshold_ = Combinations::numCombinations(object_points_.size(), 3); // no more used
}

List4DPoints PoseEstimator::getMarkerPositions()
{
  return object_points_;
}

std::vector<bool> PoseEstimator::estimateBodyPose(cv::Mat image, double time_to_predict, std_msgs::Duration &timeInitEst, cv::Point3i &ROI_center)
{

  if (bUseParticleFilter) // use particle filter for pose estimation
  {
	  // Set up pixel positions list
	  List2DPoints detected_led_positions;

	  static int min_blob_area_adapt; // define adaptive blob size threshold in marker detection
	  static int max_blob_area_adapt;
	  if (predicted_pose_Vec.size() == 0)
	    { min_blob_area_adapt = 0; max_blob_area_adapt = max_blob_area_;}


	  static std::vector<int> uncertainty;		// describes the number of how many times in a row the PF has not found an 'good' solution or hte initialisation has failed
	  static int ROI_dyn; 				// initialise the variable which tells the ROI if it should take its own movement into consideration or not:
	  Eigen::Matrix4d ident;
	  ident.setIdentity();
	  static Eigen::Matrix4d P_obsCam_old = ident;		// initialise the old camera pose

	  for (int objectNumber = 0; objectNumber < object_points_Vec.size(); objectNumber++)
	  {
		  bPose_updated = false; // initialise the output with false --> important as it is a 'global' private variable and therefore keeps the value true after sucessfully applying PF or initialisation
		  PubData.Flag_Fail[objectNumber] = -1; // initialise the Fail Flag

		  // define and initialise the storage vectors
		  if (predicted_pose_Vec.size() == objectNumber)
		  {
		    uncertainty.push_back(0);

		    predicted_pose_Vec.push_back(predicted_pose_);
		    pose_covariance_Vec.push_back(pose_covariance_);
		    current_pose_Vec.push_back(current_pose_);
		    previous_pose_Vec.push_back(previous_pose_);

		    if (objectNumber == 0)
		    {
			for(int i = 0; i<N_Particle; i++)
			{
			  PoseParticle.push_back(current_pose_);
			}
		    }

		    newPoseEstimation = PoseParticle;
		    PoseParticle_Vec.push_back(PoseParticle);
		    newPoseEstimation_Vec.push_back(newPoseEstimation);  ROS_INFO("object number %d is initialised",(int) predicted_pose_Vec.size());
		    it_since_initialized_Vec.push_back(0);
		    region_of_interest_Vec.push_back(cv::Rect(0, 0, image.cols, image.rows));
		    bPose_updated_Vec.push_back(bPose_updated);
		    probPart_Vec.push_back(probPart);
		    distorted_detection_centers_Vec.push_back(distorted_detection_centers_);
		  }

		  // set the object points and initialisation status corresponding to the treated object
		  object_points_ = object_points_Vec[objectNumber];
		  predicted_pixel_positions_.resize(object_points_.size()*(N_Particle+1)); // resize the number of predicted points for ROI!!! important (prediced positions of all resamples AND the predicted_pose_)
		  it_since_initialized_ = it_since_initialized_Vec[objectNumber];


		  if (it_since_initialized_ < 1)  // initialisation is needed, at the beginning or when track was completely lost
		  {
/*
 *    DETECTION STAGE (INITIALISATION)
 */
		      ros::Time startInit = ros::Time::now(); //record the time needed for the initalisation

			  setPredictedTime(time_to_predict);

			  // define the area for the LED search in the image, increase the area if no pose could be estimated last time
			  region_of_interest_ =	       region_of_interest_Vec[objectNumber];
			  region_of_interest_.x =      std::max(region_of_interest_.x-7*(1+(int)std::floor(uncertainty[objectNumber]/3)),0);
			  region_of_interest_.y =      std::max(region_of_interest_.y-7*(1+(int)std::floor(uncertainty[objectNumber]/3)),0);
			  region_of_interest_.width =  std::min(region_of_interest_.width+14*(1+(int)std::floor(uncertainty[objectNumber]/3)),(int)(image.size().width-region_of_interest_.x));
			  region_of_interest_.height = std::min(region_of_interest_.height+14*(1+(int)std::floor(uncertainty[objectNumber]/3)),(int)(image.size().height-region_of_interest_.y));
			  region_of_interest_Vec[objectNumber] = region_of_interest_;


			  // Do detection of LEDs in image
			  LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_,
					  	  	  	    max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
								    detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
								    camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections, active_markers,
								    useOnlineExposeTimeControl, expose_time_base);

			  // Do detection of LEDs in image with the min_blob_area used by the particle filter (only in case not enough LEDs were found)
			  if (detected_led_positions.size() < object_points_.size() && min_blob_area_adapt != 0)
				  LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt,
						  	  	  	    max_blob_area_adapt, max_width_height_distortion_, max_circular_distortion_,
									    detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
									    camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections, active_markers,
									    useOnlineExposeTimeControl, expose_time_base);

			  // store the detected LEDs in the storage vector
			  distorted_detection_centers_Vec[objectNumber] = distorted_detection_centers_;

			  PubData.numDetLED = detected_led_positions.size();
			  PubData.bPred = 0; // brute force approach was used

/*
*    INITIALISATION STAGE
*/
			  if (detected_led_positions.size() >= object_points_.size()) // If found enough LEDs, initialise; (initialisation requires all markers to be detected)
			  {
			      // set the image points --> use image_pts_ for further calculations, instad of the 'global variable': detected_led_positions
			      setImagePoints(detected_led_positions);

			      if (initialise(objectNumber) == 1) // initialisation succeeded
			      {
				  ROI_dyn = 0; // the ROI takes only its own movement into consideration; at the moment this variable is not used

				  // trying to hack an error
				  current_pose_ = predicted_pose_; // the initialisation does not have an 'old' pose, therefore use the (not-)optimised pose as the current (current is the one at the beginning of the optimisation)

				  optimiseAndUpdatePose(time_to_predict);


				  // write the results to the corresponding storage vectors (each UAV has one)
				  predicted_pose_Vec[objectNumber] = predicted_pose_;
				  pose_covariance_Vec[objectNumber] = pose_covariance_;
				  current_pose_Vec[objectNumber] = current_pose_;
				  previous_pose_Vec[objectNumber] = previous_pose_;
				  PoseParticle_Vec[objectNumber] = PoseParticle;
				  newPoseEstimation_Vec[objectNumber] = PoseParticle; // reasampling was not done, therefore use PoseParticle
				  bPose_updated_Vec[objectNumber] = bPose_updated;

				  it_since_initialized_Vec[objectNumber] = it_since_initialized_;
				  PubData.Flag_Fail[objectNumber] = 0; // initialisation succeeded

			      }
			      else
			      { // initialisation failed (fail flags are determined in the initialise function)
				  bPose_updated_Vec[objectNumber] = bPose_updated;
				  uncertainty[objectNumber]++; // increase the uncertainty by one
				  //ROS_INFO("Initialisation failed. PubData.Flag_Fail[objectNumber] = %f",PubData.Flag_Fail[objectNumber]);
			      }

			  }
			  else
			  { // Too few LEDs found
				  PubData.Flag_Fail[objectNumber] = 3;
				  uncertainty[objectNumber] += 2; // +2 for a fast increase of the ROI
				  bPose_updated_Vec[objectNumber] = bPose_updated;
			  }
			  ros::Time endInit = ros::Time::now();
			  timeInitEst.data = endInit-startInit; // time needed for the initialisation
		  }
		  else
		  {   // if already initialised use the particle filter
		      // no initialisation, so set the initialisation time to zero
		    ros::Time endInit = ros::Time::now();
		    timeInitEst.data = endInit-endInit;


		    // ---> set the corresponding poseEstimations from the last calculation
		    predicted_pose_ = 		predicted_pose_Vec[objectNumber];
		    current_pose_ = 		current_pose_Vec[objectNumber];
		    previous_pose_ = 		previous_pose_Vec[objectNumber];
		    PoseParticle = 		PoseParticle_Vec[objectNumber];
		    newPoseEstimation = 	newPoseEstimation_Vec[objectNumber];

/*
*    DETECTION STAGE (PARTICLE FILTER)
*/

		    // ---> predict the new pose for the ROI determination
		    Eigen::Matrix4d predictionMatrix = predictPose(time_to_predict); // movement from old to the predicted pose
		    Eigen::Matrix4d predictionMatrixInclCam = predictionMatrix;
		    //ROS_INFO("prediction matrix");
		    //displayMatrix(predictionMatrix);

		    // consider the predicted camera movement, given by the pose P_obsUAV

		    static Eigen::Matrix4d camMoveInv;
		    //camMoveInv.setIdentity();

		    if (bUseCamPos)
		      { // take movement of the camera into account (made for provision by ROVIO)
			    //static double time_obsUAV_new;
			    static double time_obsUAV_act; // Time stamp of the previous pose from the observer UAV
			    //static double time_obsUAV_old; // Time stamp of the pose from the observer UAV two poses ago
			    //static double time_obsUAV_temp;

			   // if (time_obsUAV > 10000) // prevent updating value when not initialised (fix me: set 10000 to not initialised)
			     // time_obsUAV_new = time_obsUAV;

			    if (P_obsUAV.determinant() == 0) // if no information about the cam pose is available, set it to the origin
			      P_obsUAV.setIdentity();


			    Eigen::Matrix4d rotCam; // has to be changed due to the experimental setup

			    rotCam <<   0,  1,  0,  0,  //-90 degrees in z,  then -90 degrees in x
		    		        0,  0,  1,  0,
				        1,  0,  0,  0,
					0,  0,  0,  1;

/*			    rotCam <<    0,       1,  0,       0,  //-90 degrees in z,  then -66 degrees in x
		    		        -0.4067,  0,  0.9135,  0,
					 0.9135,  0,  0.4067,  0,
					 0,       0,  0,       1;*/

/*			    rotCam <<    0.5,     0.8660,  0,  0,  //-60 degrees in z,  then -90 degrees in x
		    		         0,       0,       1,  0,
					 0.8660, -0.5,     0,  0,
					 0,       0,       0,  1;*/

/*			    rotCam <<    0.5,     0.8660,  0,       0,  //-60 degrees in z,  then -66 degrees in x
		    		        -0.3522,  0.2034,  0.9135,  0,
					 0.7912, -0.4568,  0.4067,  0,
					 0,       0,       0,       1;*/

			    Eigen::Matrix4d P_obsCam = P_obsUAV * rotCam;
			    static Eigen::Matrix4d changeCamPose;

			    //if (it_since_initialized_ < 2) // less than 3 consecutive camera poses are available
			      //P_obsCam_old = P_obsCam;  // P_obsCam_old.setIdentity();



			    Eigen::Matrix4d camMoveExtraPol;
			    double timeRatioExtrapolate;
			    static double cameraTimeShift;

			    if (time_to_predict == time_obsUAV) // t_UAV == t_camera
			      camMoveExtraPol.setIdentity();
			    else if (time_to_predict > time_obsUAV)
			      {
				if (time_obsUAV > time_obsUAV_act) // new camera pose obtained, calculate camera motion
				  {
				    changeCamPose = P_obsCam_old.inverse() * P_obsCam;
				    P_obsCam_old = P_obsCam;
				    cameraTimeShift = time_obsUAV - time_obsUAV_act;
				    time_obsUAV_act = time_obsUAV;
				    ROS_INFO("new cam pose available");
				  }
				    Vector6d deltaCamMove = logarithmMap(changeCamPose); 						// get the movement in twis coordinates
				    timeRatioExtrapolate = ((time_to_predict - current_time_)/cameraTimeShift); 	// extrapolation factor
				    Vector6d deltaCamMoveExtraPol = deltaCamMove * timeRatioExtrapolate; 				// extrapolate the camera movement
				    camMoveExtraPol = exponentialMap(deltaCamMoveExtraPol);				// get the extrapolated pose change
				    ROS_INFO("use camera extrapolation");


				    ROS_INFO("Diff1: %f",time_to_predict - time_obsUAV);
				    ROS_INFO("Diff2: %f",cameraTimeShift);
				    ROS_INFO("Ratio: %f",timeRatioExtrapolate);
			      }
			    else
			      ROS_INFO("camera time is bigger than UAV time!!!");

			    camMoveInv = camMoveExtraPol.inverse();
			    displayMatrix(changeCamPose);
			    ROS_INFO(" ");
			    displayMatrix(camMoveExtraPol);
			    ROS_INFO(" ");
			    displayMatrix(camMoveInv);



			    //camMoveExtraPol = camMove * ((time_to_predict - time_obsUAV)/(time_obsUAV - time_obsUAV_old));


			/*if (time_obsUAV > time_obsUAV_temp && time_obsUAV_act < 1000)
			  time_obsUAV_act = time_obsUAV_new;
			else if (time_obsUAV > time_obsUAV_temp) // do only update the camera movement if the there was a change in the camera pose
			  {
			    double timeRatio = (time_obsUAV - time_obsUAV_act)/(time_obsUAV_act - time_obsUAV_old);

			    double deltaTimeRovio = time_obsUAV - time_obsUAV_act; // time passed since the last rovio update

			    if (it_since_initialized_ > 1 && timeRatio < 1.001 && timeRatio > 0.999) // use rovio only for two consecutive frames
			      {
				Eigen::Matrix4d changeCamPose = P_obsCam_old.inverse() * P_obsCam;
				double velCam = sqrt(pow(changeCamPose(0,3),2)+pow(changeCamPose(1,3),2)+pow(changeCamPose(2,3),2))/deltaTimeRovio; // [m/s]
				double traceRot = changeCamPose.block<3, 3>(0, 0).trace();

				if (velCam < 4  && traceRot > 2.5)
				  {
				    // get movement, then its twist coordinates
				    Vector6d delta = logarithmMap(changeCamPose);

				    // extrapolate movement
				    double timeRatioExtrapolate = (predicted_time_ - current_time_) / deltaTimeRovio; // use ratio from the two camera timestamps to extrapolate the rovio camera movement
				    Vector6d delta_hat = delta * timeRatioExtrapolate;

				    Eigen::Matrix4d camMove = exponentialMap(delta_hat);
				    camMoveInv = camMove.inverse();
				    //camMoveInv = changeCamPose.inverse();
				    //ROS_INFO("predict cam pose");
				    //ROS_INFO("Cam Pose change:");
				    //displayMatrix(P_obsCam_old.inverse() * P_obsCam);
				    //ROS_INFO("time: %f", timeRatio); //(time_obsUAV - time_obsUAV_act)/(time_obsUAV_act - time_obsUAV_old));
				    //ROS_INFO("camMoveInv:");
				    //displayMatrix(camMoveInv);
				    ROS_INFO("use camMoveInv");
				  }
				else
				  {
				    camMoveInv.setIdentity();
				    ROS_WARN("Movement of Rovio pose is to big (>4m/s or trace(R)> 2.5), do not use Rovio");
				    ROS_INFO("time delta rovio: %f sec",deltaTimeRovio);
				    displayMatrix(changeCamPose);
				  }
			      }
			    else
			      {
				camMoveInv.setIdentity();
				ROS_WARN("Not enough consecutive frames available, dont use Rovio");
				ROS_INFO("ratio of time between last three rovio frames: %f", timeRatio);
				ROS_INFO("it_since_initialized_: %d",it_since_initialized_);
			      }

			    time_obsUAV_old = time_obsUAV_act;
			    time_obsUAV_act = time_obsUAV_new;
			    //ROS_INFO("update camera time");
			  }
			else if (it_since_initialized_ < 2)
			  camMoveInv.setIdentity();

			  time_obsUAV_temp = time_obsUAV;

			    if (it_since_initialized_ > 1)
			    	P_obsCam_old = P_obsCam;*/
		      }
		    else
		      camMoveInv.setIdentity();

		    predicted_pose_ = camMoveInv * predicted_pose_; // = predicted_pose_ * camMoveInv; // adapt the camera motion to the predicted pose
		    predictMarkerPositionsInImage(camMoveInv, predictionMatrix);  // predictMarkerPositionsInImage(predictionMatrix * camMoveInv);




		    // ---> Define region of interest (ROI)
		    if (ROI_dyn == 0) // this if clause is only needed if the ellipse case is used (but, findLED is not working with cv::RotatedRect) --> not used yet
		    {
		      region_of_interest_ = LEDDetector::determineROI(getPredictedPixelPositions(), image.size(), roi_border_thickness_,
							              camera_matrix_K_, camera_distortion_coeffs_, region_of_interest_ellipse, ROI_dyn);
		      ROI_dyn = 1;
		    }
		    else
		    {
		      region_of_interest_ = LEDDetector::determineROI(getPredictedPixelPositions(), image.size(), roi_border_thickness_,
								      camera_matrix_K_, camera_distortion_coeffs_, region_of_interest_ellipse, ROI_dyn);
		    }





		    // get the movement of the ROI
		    double cx = region_of_interest_.x + region_of_interest_.width/2;
		    double cy = region_of_interest_.y + region_of_interest_.height/2;
		    double cx_old = region_of_interest_Vec[objectNumber].x + region_of_interest_Vec[objectNumber].width/2;
		    double cy_old = region_of_interest_Vec[objectNumber].y + region_of_interest_Vec[objectNumber].height/2;

		    // ---> enlarge the ROI dependent on its calculated movement and a value depending on die distance to the camera
		    int ROI_distVal = 20/current_pose_(2,3);
		    int ROI_dyn_x = 0; //(cx-cx_old)/8; // could be changed so that it depends on the effective velocity and not only on the division by 10
		    int ROI_dyn_y = 0; //(cy-cy_old)/8;

		    region_of_interest_.x =      std::max(region_of_interest_.x-ROI_distVal+ROI_dyn_x,0);
		    region_of_interest_.y =      std::max(region_of_interest_.y-ROI_distVal+ROI_dyn_y,0);
		    region_of_interest_.width =  std::min(region_of_interest_.width+2*ROI_distVal+std::abs(ROI_dyn_x),(int)(image.size().width-region_of_interest_.x));
		    region_of_interest_.height = std::min(region_of_interest_.height+2*ROI_distVal+std::abs(ROI_dyn_y),(int)(image.size().height-region_of_interest_.y));

		    // adapt the min_blob_area_ depending on the predicted distance of the UAV
		    double pred_dist = sqrt(pow(predicted_pose_(0,3),2)+pow(predicted_pose_(1,3),2)+pow(predicted_pose_(2,3),2));
		    int abs_min_blob_area = 5;
		    min_blob_area_adapt = std::max(abs_min_blob_area,std::min((int)min_blob_area_,(int)(min_blob_area_-10*(pred_dist-1)))); // change the hard coded stuff
		    int abs_max_blob_area = 20;
		    int max_blob_area_adapt = std::max(abs_max_blob_area,std::min((int)max_blob_area_,(int)(max_blob_area_-10*(pred_dist-1)))); // change the hard coded stuff

		    // ---> Detect the LEDs in the image
		    LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt,
								    max_blob_area_adapt, max_width_height_distortion_, max_circular_distortion_,
								    detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
								    camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections,active_markers,
								    useOnlineExposeTimeControl, expose_time_base);

		    // ---> number of detected LEDs
		    int numLED = detected_led_positions.size();

		    // ---> if not enough LEDs (less than 4) were detected, enlarge the ROI
		    if (numLED < min_num_leds_detected_) //object_points_.size()
		    {
			    region_of_interest_.x =      std::max(region_of_interest_.x-20+5*ROI_dyn_x,0);
			    region_of_interest_.y =      std::max(region_of_interest_.y-20+5*ROI_dyn_y,0);
			    region_of_interest_.width =  std::min(region_of_interest_.width+40+10*std::abs(ROI_dyn_x),(int)(image.size().width-region_of_interest_.x));
			    region_of_interest_.height = std::min(region_of_interest_.height+40+10*std::abs(ROI_dyn_y),(int)(image.size().height-region_of_interest_.y));

			    LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt, max_blob_area_adapt,
									  max_width_height_distortion_, max_circular_distortion_, detected_led_positions, distorted_detection_centers_,
									  camera_matrix_K_, camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections,active_markers,
									  useOnlineExposeTimeControl, expose_time_base);
		    }
		    // ---> store the region of interest and the detectios found (incl. occluded and false detections) for the individual object
		    region_of_interest_Vec[objectNumber] = region_of_interest_;  
		    /* ----- TIMO ------- */
		    cv::Point3i ROItest;
		    
		    ROItest.x = region_of_interest_Vec[objectNumber].x + region_of_interest_Vec[objectNumber].width / 2 ;
		    ROItest.y = region_of_interest_Vec[objectNumber].y + region_of_interest_Vec[objectNumber].height / 2 ;
		    ROI_center.x = ROItest.x;
		    ROI_center.y = ROItest.y;
		    /* -------------------*/
		    
		    distorted_detection_centers_Vec[objectNumber] = distorted_detection_centers_;

		    // ---> get the coordinates of the detected points
		    setImagePoints(detected_led_positions);

/*
*    PARTICLE FILTER STAGE
*/

		    // ---> parameter initialisation for the particle filter
		    std::random_device rd;					// random number used for seeding the random generator below
		    std::default_random_engine generator (rd());		// random number generator used for the motion noise
		    Eigen::Matrix4d rotX;					// rotation matrix (x-axis)
		    Eigen::Matrix4d rotY;					// rotation matrix (y-axis)
		    Eigen::Matrix4d rotZ;					// rotation matrix (z-axis)
		    double facTransX;						// factor used to reduce the translation uncertainty after initialising the particle filter
		    double facTransY;
		    double facTransZ;
		    double facRotX;
		    double facRotY;
		    double facRotZ;

		    if (it_since_initialized_ == 1) // reduce the particle distribution after  the initialisaton
		      {
			    facTransX = 1; //std::max(std::min(0.2,std::abs(predictionMatrix(0,3))/4),0.1)*10;
			    facTransY = 1; //std::max(std::min(0.2,std::abs(predictionMatrix(1,3))/4),0.1)*10;
			    facTransZ = 1; //std::max(std::min(0.2,std::abs(predictionMatrix(2,3))/4),0.1)*10;
			    facRotX = 1;
			    facRotY = 1;
			    facRotZ = 1;
		      }
		    else
		      {
			double timeDiffFrames = predicted_time_ - current_time_;
			    facTransX = std::min(std::max(0.2,std::abs(predictionMatrix(0,3))/timeDiffFrames),1.0)/4; // 0.2;
			    facTransY = std::min(std::max(0.2,std::abs(predictionMatrix(0,3))/timeDiffFrames),1.0)/4; // 0.2;
			    facTransZ = std::min(std::max(0.2,std::abs(predictionMatrix(0,3))/timeDiffFrames),1.0)/4; // 0.2;
			    facRotX = 0.2;
			    facRotY = 0.2;
			    facRotZ = 0.2;
		      }



		    std::uniform_real_distribution<double> randTransX(minTransitionNoise*facTransX,maxTransitionNoise*facTransX);		// random number used for generating the translation   (m)
		    std::uniform_real_distribution<double> randTransY(minTransitionNoise*facTransY,maxTransitionNoise*facTransY);		// random number used for generating the translation   (m)
		    std::uniform_real_distribution<double> randTransZ(minTransitionNoise*facTransZ,maxTransitionNoise*facTransZ);		// random number used for generating the translation   (m)
		    //std::uniform_real_distribution<double> randAngle(minAngularNoise,maxAngularNoise); 			// random number used for generating the rotation      (radian)
		    std::uniform_real_distribution<double> randAngleX(minAngularNoise*facRotX,maxAngularNoise*facRotX);
		    std::uniform_real_distribution<double> randAngleY(minAngularNoise*facRotY,maxAngularNoise*facRotY);
		    std::uniform_real_distribution<double> randAngleZ(minAngularNoise*facRotZ,maxAngularNoise*facRotZ);
		    int Particle_index;						// index number of the individual particle
		    double probPartSum = 0;					// (accumulated) sum of the probability of each single particle (probability == weigth)
		    double highestProb = 0;					// highest calculated probability of a particle
		    probPart.setZero(1,N_Particle);				// initalise the vector containing all particle probabilities (weights)
		    int N_Resamples = N_Particle;				// number of chosen resamplings
		    double randVar;						// random variable which is used in the resampling process
		    std::uniform_real_distribution<double> randResample(0,1);	// random number used for generating the random variable in the resampling process
		    VectorXu counterMeas(N_Particle,1);				// vector which counts how many times a praticle is resampled
		    counterMeas.setZero();
		    std::vector<VectorXuPairs> correspondencesVec; 		// vector containing the correspondences of the valid particles
		    int iter = 0;						// number of iterations for generating the prediction and measurement update
		    std::vector<Eigen::Matrix4d> PoseParticle_likely;		// storage vector for the most likely particle vector in the while loop
		    std::vector<VectorXuPairs> correspondencesVec_likely;	// storage vector for the most likely correspondence vector in the while loop
		    RowXd probPart_likely = probPart;				// storage vector for the most likely probability vector in the while loop
		    int mostLikelyParticleIdx;					// index of the most likely particle in the while loop
		    Eigen::Matrix4d PoseParticle_temp;				// temporary pose of the particle


		    do
		    {
		     correspondencesVec.clear(); // clear the correspondence vector, otherwise the size would increas to bigger values than possible (is not set to zero at the end of the loop)


  /* ---------- get the prediction states (x_p)----------*/

		      // motion model; as motion is unknown, use a small disturbance to cover the motion
		      for (int numParticle = 0; numParticle<N_Particle; numParticle++)
		      {
			if (numParticle == 0) // use optimized pose for the first particle
			{
			      PoseParticle[numParticle] = current_pose_;
			}
			else if (numParticle == 1) // use predicted pose for the second particle (simple velocity prediction)
			{
			      PoseParticle[numParticle] = predicted_pose_;
			}
			else	// apply the predicted movement and then small motions to the other particles
			{
			  if (it_since_initialized_ > 1 && (iter%10) != 0) // use the prediction only if last time was NOT the initialisation (there would not be a 'good' prediction)
			    PoseParticle_temp = camMoveInv * newPoseEstimation[numParticle] * predictionMatrix; // apply the predicted movement only to 90% of remaining the particles
			  else if (it_since_initialized_ > 1)
			    PoseParticle_temp = camMoveInv * newPoseEstimation[numParticle];
			  else
			    PoseParticle_temp = newPoseEstimation[numParticle];

			  //rotation
			  double a = randAngleX(generator)*(1+0.025*std::floor(iter/10));
			  double b = randAngleY(generator)*(1+0.025*std::floor(iter/10));
			  double c = randAngleZ(generator)*(1+0.025*std::floor(iter/10));

			  rotX <<    	     1,        0,      0,      0,
					     0,      cos(a), -sin(a),  0,
					     0,      sin(a),  cos(a),  0,
					     0,        0,      0,      1;

			  rotY <<    	   cos(b),     0,     sin(b),  0,
					     0,        1,      0,      0,
					   -sin(b),    0,     cos(b),  0,
					     0,        0,      0,      1;

			  rotZ <<    	    cos(c), -sin(c),   0,      0,
					    sin(c),  cos(c),   0,      0,
					     0,       0,       1,      0,
					     0,       0,       0,      1;

			  PoseParticle[numParticle] = PoseParticle_temp*rotZ*rotY*rotX;

			  // translation (add translation seperately to guarantee independency to the rotation)
			  PoseParticle[numParticle](0, 3) = PoseParticle_temp(0, 3) + randTransX(generator)*(1+0.025*std::floor(iter/10));
			  PoseParticle[numParticle](1, 3) = PoseParticle_temp(1, 3) + randTransY(generator)*(1+0.025*std::floor(iter/10));
			  PoseParticle[numParticle](2, 3) = PoseParticle_temp(2, 3) + randTransZ(generator)*(1+0.025*std::floor(iter/10));
		        }


  /* ---------- get the measurement states (x_m)----------*/

		         // initialise the predicted object points
			 List2DPoints projected_ObjectPoints(object_points_.size());

			 for (int numObjectPoint = 0; numObjectPoint<object_points_.size(); numObjectPoint++)
			 {
			    projected_ObjectPoints(numObjectPoint) = project2d((Eigen::Vector4d)object_points_(numObjectPoint), PoseParticle[numParticle]);
			 }

			 //calculate the probability of the particle corresponding to the measurements (detections) --> f(z(k)|x^n_p(k))
			 probPart(0,numParticle) = calculateEstimationProbability(image_points_, projected_ObjectPoints, correspondencesVec);  // not normalized!!!! (max value is #objectPoint*(#objectPoint+1))


		      } // end iteration through all particles
		      iter++;

		      if (probPart.maxCoeff() > highestProb)
		      {
			  highestProb = probPart.maxCoeff(&mostLikelyParticleIdx);
			  correspondencesVec_likely = correspondencesVec;
			  PoseParticle_likely = PoseParticle;
			  probPart_likely = probPart;
		      }

		    } while (iter<80 && probPart.maxCoeff()<object_points_.size()*std::min(5,numLED));
		    // exit the loop if max number of iterations is reached OR at least one particle is 'enough' likely to be the correct one (true pose)

		    if (PoseParticle_likely.size() != 0)
		    {
			  PoseParticle = PoseParticle_likely;
			  correspondencesVec = correspondencesVec_likely;
			  probPart = probPart_likely;
		    }


		    probPartSum = probPart.sum();
		    if (probPartSum != 0)
			    probPart = probPart/probPartSum;	//normalize the probabilities of the particle --> beta_n


		    // ---> check if at least one particle is reasonable, and if so, resample all particles, otherwise reinitialise (reasonable --> at least 3 LEDs match with detections)
		    if (probPartSum != 0 && highestProb > object_points_.size()*std::min(3,numLED))
		    {
			PubData.Flag_Fail[objectNumber] = 1; // estimation was made; PF succeeded

			if (highestProb < object_points_.size()*std::min(3,numLED)+2/3*numLED) // check if markers are missing or self-occlusion happend
			{
			    if (uncertainty[objectNumber] < 200)
			    {
				uncertainty[objectNumber]++;
				ROS_INFO("correspondence size: %d", (int) correspondencesVec[mostLikelyParticleIdx].rows());
				if (correspondencesVec[mostLikelyParticleIdx].rows() == 3) // in case only 3 correspondeces could be found, Reinitialise with short P3P
				{
				   if (P3P_short(correspondencesVec[mostLikelyParticleIdx], objectNumber))
				     PubData.Flag_Fail[objectNumber] = 2;
				   else
				     it_since_initialized_ = 0;

				   ROS_INFO("Do short P3P");
				}

			    }
			    else
			    {
				it_since_initialized_ = 0;    // too big uncertainty; set it_since_initialized_ to 0 to force a new initialisation
				uncertainty[objectNumber] = 1;
				PubData.Flag_Fail[objectNumber] = 5;
				ROS_WARN(" einitialise due to too big uncertainty");
			    }
			}
			else  // do a resampling of the particle
			  uncertainty[objectNumber] = 1;


			if (it_since_initialized_ > 0) // do only resample if a 'reasonable' pose could be found
			  {
/*---- Resampling ----*/     for (int numResamples = 0; numResamples<N_Resamples; numResamples++)
			     {
			       //randVar = randResample(generator); 				// get a random number for the resampling		--> Multinomial resampling
			       randVar = (numResamples+randResample(generator))/N_Resamples; 	// get a random number for the resampling		--> Stratified resampling
			       probPartSum = 0; // is used to get the accumulated sum of the first 'idxParticle' particle

			       for(int idxParticle = 0; idxParticle<N_Particle; idxParticle++) // pick a new sample
			       {
				       probPartSum += probPart(idxParticle);
				       if (probPartSum>=randVar)
					      {Particle_index = idxParticle; counterMeas(idxParticle)++; break;}
			       }

			       newPoseEstimation[numResamples] = PoseParticle[Particle_index]; // this particle is resampled
			     }

			     // take the most likely (most resampled) particle and optimise it due to the detections
			     int mostLikelyPoseIdx;
			     double maxCoeff = counterMeas.maxCoeff(&mostLikelyPoseIdx);
			     predicted_pose_ = PoseParticle[mostLikelyPoseIdx];
			     correspondences_ = correspondencesVec[mostLikelyPoseIdx];
			     Eigen::Matrix4d predicted_pose_temp = predicted_pose_;
			     optimiseAndUpdatePose(time_to_predict);

			     // check if the optimisation made a big jump in the pose
			     if (std::abs(predicted_pose_temp(0,0)-predicted_pose_(0,0)) >= 0.3 || std::abs(predicted_pose_temp(0,1)-predicted_pose_(0,1)) >= 0.3 || std::abs(predicted_pose_temp(0,2)-predicted_pose_(0,2)) >= 0.3
			      || std::abs(predicted_pose_temp(1,0)-predicted_pose_(1,0)) >= 0.3 || std::abs(predicted_pose_temp(1,1)-predicted_pose_(1,1)) >= 0.3 || std::abs(predicted_pose_temp(1,2)-predicted_pose_(1,2)) >= 0.3
			      || std::abs(predicted_pose_temp(2,0)-predicted_pose_(2,0)) >= 0.3 || std::abs(predicted_pose_temp(2,1)-predicted_pose_(2,1)) >= 0.3 || std::abs(predicted_pose_temp(2,2)-predicted_pose_(2,2)) >= 0.3)
			       {
				 //ROS_INFO("new matrix:");
				 //displayMatrix(predicted_pose_temp-predicted_pose_);
				 PubData.Flag_Fail[objectNumber] = 1.5;
				 //predicted_pose_ = predicted_pose_temp;
			       }

			     predicted_pose_Vec[objectNumber] = predicted_pose_;
			  }

		    }
		    else  // no particle was reasonable (less than 3 detection matches) --> reinitialise
		    {
		      uncertainty[objectNumber]++;
		      it_since_initialized_ = 0;
		      PubData.Flag_Fail[objectNumber] = 4;
		      ROS_INFO("Reinitialise due to lack of detection machtes.  highestProb = %f; numLED = %d",highestProb, numLED);

		      highestProb = probPart.maxCoeff(&mostLikelyParticleIdx);
		      //correspondencesVec_likely = correspondencesVec;
		      predicted_pose_ = PoseParticle[mostLikelyParticleIdx];
		      //probPart_likely = probPart;

		    }

		    // write the results to the corresponding storage vector
		    //predicted_pose_Vec[objectNumber] = predicted_pose_; if resampling was done do not use the optimised pose for the visualisation
		    pose_covariance_Vec[objectNumber] = pose_covariance_;
		    current_pose_Vec[objectNumber] = current_pose_; // after pose update current_pose contains the optimsed pose prediction
		    previous_pose_Vec[objectNumber] = previous_pose_;
		    PoseParticle_Vec[objectNumber] = PoseParticle;
		    newPoseEstimation_Vec[objectNumber] = newPoseEstimation;

		    it_since_initialized_Vec[objectNumber] = it_since_initialized_;

		    bPose_updated_Vec[objectNumber] = bPose_updated;

		    probPart_Vec[objectNumber] = probPart;

		  } // end if clause if initialised or not
	  } // end for loop, all different objects
  } // end if PF
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------  END PF
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------  END PF
else // if the PF was not used, use the algorithm of faessler (functions which are also used in the PF case, were modified)
  {
    for (int objectNumber = 0; objectNumber < object_points_Vec.size(); objectNumber++)
    {

	  bPose_updated = false;
	  if (bPose_updated_Vec.size() == objectNumber)
	    {
	      bPose_updated_Vec.push_back(bPose_updated);
	      predicted_pose_Vec.push_back(predicted_pose_);
	      pose_covariance_Vec.push_back(pose_covariance_);
	      current_pose_Vec.push_back(current_pose_);
	      previous_pose_Vec.push_back(previous_pose_);
	      it_since_initialized_Vec.push_back(0);
	      region_of_interest_Vec.push_back(cv::Rect(0, 0, image.cols, image.rows)); // ROI equals the whole image
	      distorted_detection_centers_Vec.push_back(distorted_detection_centers_);
	    }

	  object_points_ = object_points_Vec[objectNumber];
	  it_since_initialized_ = it_since_initialized_Vec[objectNumber];
	  PubData.Flag_Fail[objectNumber] = -1;


	  // Set up pixel positions list
	  List2DPoints detected_led_positions;

	  static int min_blob_area_adapt = min_blob_area_;


	  if (it_since_initialized_ < 1)  // If not yet initialised, search the whole image for the points
	    {
	      // set the time which is needed for the initialisation
	      ros::Time startInit = ros::Time::now();

	      setPredictedTime(time_to_predict);

	      region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

	      // Do detection of LEDs in image
	      LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt,
	                            max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
	                            detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
	                            camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections, active_markers,
				    useOnlineExposeTimeControl, expose_time_base);

	      PubData.numDetLED = detected_led_positions.size();
	      PubData.bPred = 0; // brute force approach was used

	      if (detected_led_positions.size() >= min_num_leds_detected_) // If found enough LEDs, Reinitialise
	      {
	        // Reinitialise
	  //      ROS_WARN("Initialising using brute-force correspondence search.");

	        setImagePoints(detected_led_positions);

	        if (initialise(objectNumber) == 1)
	        {
	          optimiseAndUpdatePose(time_to_predict);
	          PubData.Flag_Fail[objectNumber] = 0;
	        }
	        else
	        {
	      	  PubData.Flag_Fail[objectNumber] = 7; // initialisation failed, no valid correspondences
	        }
	      }
	      else
	      { // Too few LEDs found
	      	PubData.Flag_Fail[objectNumber] = 10; // not enough LED while initialising
	      }
	      // calculate the time which was needed for the initialisation
	      ros::Time endInit = ros::Time::now();
	      timeInitEst.data = endInit-startInit;
	    }
	    else
	    { // If initialised
	      // as the system is already initialised, set the time for the publisher to zero
	      ros::Time endInit = ros::Time::now();
	      timeInitEst.data = endInit-endInit;

	      predicted_pose_ = 	predicted_pose_Vec[objectNumber];
	      current_pose_ = 		current_pose_Vec[objectNumber];
	      previous_pose_ = 		previous_pose_Vec[objectNumber];

	      double pred_dist = sqrt(pow(predicted_pose_(0,3),2)+pow(predicted_pose_(1,3),2)+pow(predicted_pose_(2,3),2));
	      int abs_min_blob_area = 5;
	      min_blob_area_adapt = std::max(abs_min_blob_area,std::min((int)min_blob_area_,(int)(min_blob_area_-10*(pred_dist-1)))); // change the hard coded stuff


	      predictWithROI(time_to_predict, image);
	      // Search image within ROI
	      LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt,
	                            max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
	                            detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
	                            camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections, active_markers,
				    useOnlineExposeTimeControl, expose_time_base);
	      PubData.numDetLED = detected_led_positions.size();

	      bool repeat_check = true;
	      unsigned num_loops = 0;
	      do
	      {
	        num_loops++;
	        if (detected_led_positions.size() >= min_num_leds_detected_) // If enough LEDs detected. If ROI was wrong one LED can be missed but the algorithm continues!!!!
	        {
	          setImagePoints(detected_led_positions);
	          findCorrespondencesAndPredictPose(time_to_predict, objectNumber, detected_led_positions);
	          repeat_check = false;


	        }
	        else
	        { // If too few LEDS detected
	          if (num_loops < 2)
	          { // If haven't searched image yet, search image

	  //          ROS_WARN("Too few LEDs detected in ROI. Searching whole image. Num LEDs detected: %d.", (int)detected_led_positions.size());

	            // Search whole image
	            region_of_interest_ = cv::Rect(0, 0, image.cols, image.rows);

	            // Do detection of LEDs in image
	            LEDDetector::findLeds(image, region_of_interest_, detection_threshold_value_, gaussian_sigma_, min_blob_area_adapt,
	                                  max_blob_area_, max_width_height_distortion_, max_circular_distortion_,
	                                  detected_led_positions, distorted_detection_centers_, camera_matrix_K_,
	                                  camera_distortion_coeffs_,number_of_occlusions,number_of_false_detections,active_markers,
					  useOnlineExposeTimeControl, expose_time_base);
	            PubData.numDetLED = detected_led_positions.size();
	            //ROS_INFO("The number of detected Leds is: %d", (int )detected_led_positions.size());
	          }
	          else
	          { // If already searched image continue
	            repeat_check = false;
	            PubData.numDetLED = detected_led_positions.size();
	            PubData.Flag_Fail[objectNumber] = 10; //2; // not enough LED detected, after init_since_initialized_itialisation
	  //          ROS_WARN("Too few LEDs detected. Num LEDs detected: %d.", (int)detected_led_positions.size());

	          }
	        }
	      } while (repeat_check);
	    }

	  bPose_updated_Vec[objectNumber] = bPose_updated;
	  predicted_pose_Vec[objectNumber] = predicted_pose_;
	  pose_covariance_Vec[objectNumber] = pose_covariance_;
	  current_pose_Vec[objectNumber] = current_pose_;
	  previous_pose_Vec[objectNumber] = previous_pose_;
	  region_of_interest_Vec[objectNumber] = region_of_interest_;
	  distorted_detection_centers_Vec[objectNumber] = distorted_detection_centers_;
	  it_since_initialized_Vec[objectNumber] = it_since_initialized_;

    }
  }


  
  return bPose_updated_Vec;
}

void PoseEstimator::setPredictedPose(const Eigen::Matrix4d & pose, double time)
{
  predicted_pose_ = pose;
  predicted_time_ = time;

}

Eigen::Matrix4d PoseEstimator::getPredictedPose(int objectNumber)
{
  return predicted_pose_Vec[objectNumber];
  //return predicted_pose_;
}

Matrix6d PoseEstimator::getPoseCovariance(int objectNumber)
{
  return pose_covariance_Vec[objectNumber];
  //return pose_covariance_;
}

std::vector<Eigen::Matrix4d> PoseEstimator::getPoseParticles(int objectNumber)
{
  return PoseParticle_Vec[objectNumber];
  //return PoseParticle;
}

std::vector<Eigen::Matrix4d> PoseEstimator::getResampledParticles(int objectNumber)
{
  return newPoseEstimation_Vec[objectNumber];
  //return newPoseEstimation;
}

void PoseEstimator::setImagePoints(List2DPoints points)
{
  image_points_ = points;
  PoseEstimator::calculateImageVectors();
}

void PoseEstimator::setPredictedPixels(List2DPoints points)
{
  predicted_pixel_positions_ = points;
}

List2DPoints PoseEstimator::getPredictedPixelPositions()
{
  return predicted_pixel_positions_;
}

void PoseEstimator::setBackProjectionPixelTolerance(double tolerance)
{
  back_projection_pixel_tolerance_ = tolerance;
}

double PoseEstimator::getBackProjectionPixelTolerance()
{
  return back_projection_pixel_tolerance_;
}

void PoseEstimator::setNearestNeighbourPixelTolerance(double tolerance)
{
  nearest_neighbour_pixel_tolerance_ = tolerance;
}

double PoseEstimator::getNearestNeighbourPixelTolerance()
{
  return nearest_neighbour_pixel_tolerance_;
}

void PoseEstimator::setCertaintyThreshold(double threshold)
{
  certainty_threshold_ = threshold;
}

double PoseEstimator::getCertaintyThreshold()
{
  return certainty_threshold_;
}

void PoseEstimator::setValidCorrespondenceThreshold(double threshold)
{
  valid_correspondence_threshold_ = threshold;
}

double PoseEstimator::getValidCorrespondenceThreshold()
{
  return valid_correspondence_threshold_;
}

void PoseEstimator::setHistogramThreshold(unsigned threshold)
{
  histogram_threshold_ = threshold;
}

unsigned PoseEstimator::getHistogramThreshold()
{
  return histogram_threshold_;
}

Eigen::Matrix4d PoseEstimator::predictPose(double time_to_predict)
{
  predicted_time_ = time_to_predict;

  // get movement, then its twist coordinates
  Vector6d delta = logarithmMap(previous_pose_.inverse() * current_pose_);

  // extrapolate
  Vector6d delta_hat = delta / (current_time_ - previous_time_) * (predicted_time_ - current_time_);

   // predict new pose
  Eigen::Matrix4d predictionMatrix = exponentialMap(delta_hat);
  predicted_pose_ = current_pose_ * predictionMatrix;

  return predictionMatrix;
}

List2DPoints PoseEstimator::getImagePoints()
{
  return image_points_;
}

inline Eigen::Vector2d PoseEstimator::project2d(Eigen::Vector4d point, Eigen::Matrix4d transform)
{
  // convert matrix interna
  Matrix3x4d camera_matrix;
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      camera_matrix(i, j) = camera_matrix_K_.at<double>(i, j);
    }
    camera_matrix(i, 3) = 0.0;
  }

  Eigen::Vector3d temp;
  temp = camera_matrix * transform * point;
  temp = temp / temp(2);
  return temp.head<2>();
}

void PoseEstimator::predictMarkerPositionsInImage(Eigen::Matrix4d camMoveInv, Eigen::Matrix4d predictionMatrix)
{
 if (bUseParticleFilter)
 {
	  for (unsigned i = 0; i < object_points_.size(); ++i) // get the pixel predictions from the resamplings
	  {
		for (int j = 0; j<N_Particle; j++)
		{
			predicted_pixel_positions_(i*N_Particle+j) = project2d((Eigen::Vector4d)object_points_(i), camMoveInv * newPoseEstimation[j] * predictionMatrix);
		}
	  }

	 for (unsigned k = 0; k < object_points_.size(); ++k) // get the pixel predictions from the prediction
	   {
		 predicted_pixel_positions_(k+object_points_.size()*N_Particle) = project2d((Eigen::Vector4d)object_points_(k), predicted_pose_);
	   }
 }
 else
 {
	 for (unsigned i = 0; i < object_points_.size(); ++i)
	   {
	     predicted_pixel_positions_(i) = project2d((Eigen::Vector4d)object_points_(i), predicted_pose_);
	   }
 }
}

void PoseEstimator::setCorrespondences(VectorXuPairs corrs)
{
  correspondences_ = corrs;
}

VectorXuPairs PoseEstimator::getCorrespondences()
{
  return correspondences_;
}

void PoseEstimator::calculateImageVectors()
{
  unsigned num_image_points = image_points_.size();
  image_vectors_.resize(num_image_points);
  Eigen::Vector3d single_vector;

  for (unsigned i = 0; i < num_image_points; ++i)
  {
    single_vector(0) = (image_points_(i)(0) - camera_matrix_K_.at<double>(0, 2)) / camera_matrix_K_.at<double>(0, 0);
    single_vector(1) = (image_points_(i)(1) - camera_matrix_K_.at<double>(1, 2)) / camera_matrix_K_.at<double>(1, 1);
    single_vector(2) = 1;
    image_vectors_(i) = single_vector / single_vector.norm();
  }
}

double PoseEstimator::calculateSquaredReprojectionErrorAndCertainty(const List2DPoints & image_pts,
                                                                    const List2DPoints & object_pts, double & certainty)
{

  double squared_error = 0;
  unsigned num_correspondences = 0;

  //Declare distance matrix
  //MatrixXYd distances(image_pts.size(), object_pts.size());
  MatrixXYd distances(image_pts.size(), 1);

  if (image_pts.size() != object_pts.size())
    ROS_INFO("Sth is wrong");

  // Build distance matrix
  for (unsigned i = 0; i < image_pts.size(); ++i)
  {
      distances(i, 0) = squareDist((Eigen::Vector2d)image_pts(i), (Eigen::Vector2d)object_pts(i));
  }
  //distances = distances.cwiseSqrt(); // Square root to get distances

  double min_value;
  unsigned row_idx, col_idx;
  double back_projection_pixel_tolerance_squared = pow(back_projection_pixel_tolerance_,2);

  for (unsigned j = 1; j <= std::min(image_pts.size(), object_pts.size()); ++j)
  {
    min_value = distances.minCoeff(&row_idx, &col_idx);
    if (min_value <= back_projection_pixel_tolerance_squared)
    {
      //squared_error += distances(row_idx, col_idx); //pow((double)distances(row_idx, col_idx), 2);
      squared_error += min_value;
      num_correspondences++;
      distances(row_idx,0) = INFINITY;
      //distances.row(row_idx).setConstant(INFINITY); // set it to infinity: initialisation requires all LED to be detected separate (no occlusions)
      //distances.col(col_idx).setConstant(INFINITY);
    }
    else
      break;
  }

  //certainty = (double)num_correspondences / object_pts.size();
  certainty = (double)num_correspondences / std::min(image_pts.size(), object_pts.size());

  return squared_error;
}

std::vector<VectorXuPairs> PoseEstimator::correspondencesFromHistogram(MatrixXYu histogram, bool bInitialisation) //VectorXuPairs
{

// Take all possible cominations for the correspondences


  /* --- CALCULATE THE PROBABILITY OF EACH POSSIBLE LED/DETECTION (CORRESPONDENCE) --- */
  unsigned numRows = histogram.rows(); // number of detections
  unsigned numCols = histogram.cols(); // number of LEDs
  double prob_threshold = (1.3*1.0)/(numRows*numCols); // = (2.0*1.0)/(numRows*numCols); // minimum probability which has to be achived, for considering the LED&detection correspondence
  //double prob_threshold = (2.0)/(numRows);
  MatrixXYd histogram_prob = histogram.cast <double> ();

  //if (histogram_prob.sum() == 0)
    //ROS_WARN("histogram sum is zero (before thresholding");

  for(int cols = 0; cols<numCols; ++cols)
  	{
	  unsigned colSum = histogram.col(cols).sum();

	  if (colSum == 0)
	    continue; // there is no detection close enough for this LED

	  for (int rows = 0; rows<numRows; ++rows)
	  {
	    histogram_prob(rows,cols) = std::max(0.0,pow(histogram_prob(rows,cols),2)/(colSum*(histogram.row(rows).sum())));
	    //histogram_prob(rows,cols) = histogram_prob(rows,cols)/colSum;

	    if (histogram_prob(rows,cols) < prob_threshold)
	      histogram_prob(rows,cols) = 0;
	  }
  	}

  //if (histogram_prob.sum() == 0)
    //ROS_WARN("histogram sum is zero (after thresholding");

/*
  ROS_INFO("Hist:");
  for (int kk = 0; kk<numRows; kk++)
    ROS_INFO("%d %d %d %d %d", histogram(kk,0), histogram(kk,1), histogram(kk,2), histogram(kk,3), histogram(kk,4));

  ROS_INFO("NormedHist:");
  for (int kk = 0; kk<numRows; kk++)
    ROS_INFO("%f %f %f %f %f", histogram_prob(kk,0), histogram_prob(kk,1), histogram_prob(kk,2), histogram_prob(kk,3), histogram_prob(kk,4));
*/



 /* --- CALCULATE THE PROBABILITY OF ALL POSSIBLE CORRESPONDENCE VECTORS (COMBINATIONS) --- */
  std::vector<double> v_prob; 			// containing possibilities of one LED to all detections
  std::vector<int> v_num;			// containing the corresponding detections
  std::vector<std::vector<double>> u_prob;	// containing possibilities of every LED to the all detections
  std::vector<std::vector<int>> u_num;		// containing the combinations of the specific LED and the possible detections (LED1: 1 2 3 4; LED2: 3 5 6; LED3: 1 3 4 5; ...)
  int N = 1;					// number of possible combinations (N_v[0]*N_v[1]*N_v[2]*...
  std::vector<int> N_v;				// vector containing the number of non zero elements per column (number of correspondences for each LED)
  std::vector<int> comb;			// chosen combination/correspondence (number of the element is the LED, value of the element is the detection)
  std::vector<std::vector<int>> v_comb;		// vector containing the chosen combinations
  std::vector<double> v_prob_comb;		// vector containing the probability of all combinations

  // store all corespondence combination with its probability
  for (int a = 0; a<numCols; a++) // a: LED index
  {
      for (int b = 0; b<numRows; b++) // b: detection index
      {
	  if (histogram_prob(b,a) != 0)
	  {
	      v_prob.push_back(histogram_prob(b,a));
	      v_num.push_back(b+1);
	  }
      }
      u_prob.push_back(v_prob); v_prob.clear();
      u_num.push_back(v_num); v_num.clear();
  }

  for (int k = 0; k<(int)u_prob.size(); k++)
  {
	  N = N * std::max(1,(int)u_prob[k].size()); 	// number of total combinations
	  N_v.push_back(u_num[k].size());		// number of combinations: specific LED(LED nr.k) and detections
  }

  // get the probability of each possible combination (of correspondences) (eg. L1D2 & L2D4 & L3D1 ... )
  for (int i = 0; i<N; i++)
  {
	  double prob = 1;
	  int n = 1;
	  for (int idxLED = u_prob.size()-1; idxLED>-1; idxLED--)
	  {
		if (N_v[idxLED] > 0)
		{
		    int idxDet = (i/n)%N_v[idxLED]; 		// calculates the index for the detection
		    prob = prob * u_prob[idxLED][idxDet]; 	// u_prob[LED][Det] (e.g. prob = u_prob[0][0]*u_prob[1][0]*u_prob[2][0]; prob = u_prob[0][0]*u_prob[1][0]*u_prob[2][1]; prob = u_prob[0][0]*u_prob[1][0]*u_prob[2][2]; ...)
		    comb.push_back(u_num[idxLED][idxDet]); 	// store the used detection
		    n = n*std::max(1,N_v[idxLED]);
		}
		else
			comb.push_back(0);
	  }

	  v_prob_comb.push_back(prob);
	  std::reverse(comb.begin(),comb.end()); // flip the vector, as the calculation began with the last LED
	  v_comb.push_back(comb);
	  comb.clear(); // each vector contains a set of detections, the index numbers are the corresponding LEDs
  }


 // normalise the probability of the specific correspondences
  double v_prob_comb_sum = 0;
  for (unsigned i = 0; i<v_prob_comb.size(); i++)
	  v_prob_comb_sum += v_prob_comb[i];

  for (unsigned ii = 0; ii<v_prob_comb.size(); ii++)
	  v_prob_comb[ii] = v_prob_comb[ii]/v_prob_comb_sum;


  // sort the correspondence vectors that the most likely one is the first
   std::vector<double>::iterator v_row_idx;
   int row_idx;
   std::vector<int> correspondingDetections;		// contains the most likely correspondence vector (LED is the number and det the value of the elements)
   VectorXuPairs correspondences(numCols, 2); 		// vector which contains the corresponding LEDs&detections
   std::vector<VectorXuPairs> correspondencesVec; 	// vector containing all combinations (of correspondences) sorted by their probability
   int corrCounter = 0;

   for (int bb = 0; bb<v_prob_comb.size(); ++bb)
   {
     v_row_idx = std::max_element(v_prob_comb.begin(), v_prob_comb.end());
     row_idx = std::distance(v_prob_comb.begin(), v_row_idx);
     v_prob_comb[row_idx] = 0;
     correspondingDetections = v_comb[row_idx];

     if (bInitialisation)
       {
	 if (checkAmbiguity(correspondingDetections)) 	// if not all detections are distinc don't use this correspondence for the initialisation
	   continue;					// in case of the reinitialisation occlusions are allowed
       }

     // fill up the correspondence vector
     corrCounter = 0;
     for (int correspondingLED = 0; correspondingLED<numCols; ++correspondingLED)
     {
	     if (correspondingDetections[correspondingLED] != 0)
	     {
		     correspondences.col(0)[corrCounter] = correspondingLED + 1; 			// LED number (index is zero based, therefore add 1 to obtain the true LED number
		     correspondences.col(1)[corrCounter] = correspondingDetections[correspondingLED]; 	// write values from vector into a matrix
		     corrCounter++; // counts the number of correspondences
	     }
     }
     VectorXuPairs correspondencesDummy = correspondences;
     correspondencesDummy.conservativeResize(corrCounter,2);
     correspondencesVec.push_back(correspondencesDummy);
   }
   if (v_comb.size() == 1 && corrCounter < 5)
     ROS_INFO("only %d correspondences could be found, numcols = %u",corrCounter,numCols);

  return correspondencesVec; //correspondences_out;
}

void PoseEstimator::findCorrespondences() // not used in the particle filter
{
  Eigen::VectorXd min_distances;
  VectorXuPairs pairs = calculateMinDistancesAndPairs(predicted_pixel_positions_, image_points_, min_distances);
  VectorXuPairs temp_corrs(pairs.rows(), 2);
  unsigned num_corrs = 0;

  for (unsigned i = 0; i < pairs.rows(); ++i)
  {
    if (min_distances(i) <= nearest_neighbour_pixel_tolerance_)
    {
      temp_corrs(num_corrs, 0) = pairs(i, 0);
      temp_corrs(num_corrs, 1) = pairs(i, 1);
      num_corrs++;
    }
  }

  temp_corrs.conservativeResize(num_corrs, 2);

  correspondences_ = temp_corrs;
}

unsigned PoseEstimator::checkCorrespondences(int & NumberOfP3PEstimation, int objectNumber, int minNumCorr)
{
  bool valid_correspondences = 0;
  unsigned num_valid_correspondences = 0;

  if (correspondences_.rows() < minNumCorr)
  {
	PubData.Flag_Fail[objectNumber] = 6; // not enough valid correspondences
    return valid_correspondences;
  }
  else
  {
    MatrixXYd mean_reprojected_object_points(4, object_points_.size());
    mean_reprojected_object_points.setZero();

    MatrixXYu combinations = Combinations::combinationsNoReplacement(correspondences_.rows(), 3);
    unsigned N = combinations.rows();
    unsigned min_squared_error_overall = INFINITY;

    for (unsigned i = 0; i < N; ++i)
    {
      //Declare and populate the feature vectors and world points required for the P3P algorithm
      Eigen::Matrix3d feature_vectors, world_points;
      Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;


      Eigen::Vector4d temp_point;
      temp_point = object_points_(correspondences_(combinations(i, 0) - 1, 0) - 1);
      world_points.col(0) = temp_point.head<3>();
      temp_point = object_points_(correspondences_(combinations(i, 1) - 1, 0) - 1);
      world_points.col(1) = temp_point.head<3>();
      temp_point = object_points_(correspondences_(combinations(i, 2) - 1, 0) - 1);
      world_points.col(2) = temp_point.head<3>();

      feature_vectors.col(0) = image_vectors_(correspondences_(combinations(i, 0) - 1, 1) - 1);
      feature_vectors.col(1) = image_vectors_(correspondences_(combinations(i, 1) - 1, 1) - 1);
      feature_vectors.col(2) = image_vectors_(correspondences_(combinations(i, 2) - 1, 1) - 1);

      // Find the unused image and object points
      unsigned total_unused_correspondences = correspondences_.rows() - 3;
      List2DPoints unused_im_points(total_unused_correspondences); 	// Vector to hold the indices of the unused image points that have correspondences
      List4DPoints unused_object_points(total_unused_correspondences); 	// Vector to hold the indices of the unused object points that have correspondences
      unsigned num_unused_points = 0;
      bool already_used_correspondence = 0;

      // Search for the unused object points and image points
      for (unsigned l = 0; l < correspondences_.rows(); ++l)
      {
        already_used_correspondence = 0;
        for (unsigned n = 0; n < 3; ++n)
        {
          if (combinations(i, n) - 1 == l)
            already_used_correspondence = 1;
        }
        if (!already_used_correspondence)
        {
          unused_object_points(num_unused_points) = object_points_(correspondences_(l, 0) - 1);
          unused_im_points(num_unused_points) = image_points_(correspondences_(l, 1) - 1);
          num_unused_points++;
        }
        //Break if all the unused object points are found
        if (num_unused_points == total_unused_correspondences)
          break;
      }

      // Compute the poses using the P3P algorithm
      int executed_correctly = P3P::computePoses(feature_vectors, world_points, solutions);

      // If the P3P algorithm found a solution (i.e. the world points were not aligned), then continue
      if (executed_correctly == 0)
      {
        double squared_error;
        double min_squared_error = INFINITY;
        unsigned smallest_error_idx;
        bool valid_correspondence_found = 0;

        Eigen::Matrix4d H_o_c_current;

        // Loop through the four solutions provided by the P3P algorithm
        for (unsigned j = 0; j < 4; ++j)
        {
          H_o_c_current.setIdentity();
          H_o_c_current.block<3, 4>(0, 0) = solutions(j);

          // Consider only the real poses. NaN poses are excluded/ignored
          if (isFinite(H_o_c_current))
          {
            //Back-project the unused object points onto the image plane
            List2DPoints unused_back_projected_object_points(total_unused_correspondences);
            Eigen::Vector3d temp;
            for (unsigned ii = 0; ii < total_unused_correspondences; ++ii)
            {
              unused_back_projected_object_points(ii) = project2d(unused_object_points(ii), H_o_c_current.inverse());
            }

            double certainty;  // calculate the reproduction error of the not in P3P used points (still correspondences)
            squared_error = calculateSquaredReprojectionErrorAndCertainty(unused_im_points,
                                                                          unused_back_projected_object_points,
                                                                          certainty);
            if (certainty >= certainty_threshold_)
            {
              valid_correspondence_found = 1;			// if at least one P3P solution is 'good' enough

              if (squared_error < min_squared_error)
              {
                min_squared_error = squared_error;
                smallest_error_idx = j;
              }
            }
          }
        }

        // take the mean of the estimated valid P3P solutions
        if (valid_correspondence_found)
        {
          num_valid_correspondences++; // number of matches while picking different combinations of detections

          if (N_Particle>=NumberOfP3PEstimation && bUseParticleFilter)
          {
              // store all 'resonable' P3P solutions for the particle filter
    	      Eigen::Matrix4d temp_solution1;
    	      temp_solution1.setIdentity();
    	      temp_solution1.block<3, 4>(0, 0) = solutions(smallest_error_idx);
              PoseParticle[N_Particle - NumberOfP3PEstimation] = temp_solution1.inverse();
              NumberOfP3PEstimation++;
          }

          // get mean reprojected object points
          Eigen::Matrix4d temp_solution;
          temp_solution.setIdentity();
          temp_solution.block<3, 4>(0, 0) = solutions(smallest_error_idx); // if two estimations are euqal 'reasonable' then take the first calculated

          for (unsigned jj = 0; jj < object_points_.size(); ++jj)
          {
            mean_reprojected_object_points.col(jj) = mean_reprojected_object_points.col(jj)
                + temp_solution.inverse() * object_points_(jj);
          }
        }
        else
        {
        	//PubData.Flag_Fail[objectNumber] = 4.2; // certainty treshold criteria failed
        }

/*

      // take the best P3P solution
		if (min_squared_error < min_squared_error_overall)
		{
			min_squared_error_overall = min_squared_error;
	        Eigen::Matrix4d temp_solution;
	        temp_solution.setIdentity();
	        temp_solution.block<3, 4>(0, 0) = solutions(smallest_error_idx); // if two estimations are euqal 'good' then take the first calculated
			predicted_pose_ = temp_solution; // if two estimations are euqal 'good' then take the first calculated
		}
		valid_correspondences = 1; // tells that a reasonable P3P solution is found
*/

      }
      else
      {
    	  PubData.Flag_Fail[objectNumber] = 9; // P3P failed
      }

    }

    if ((double)num_valid_correspondences / N >= valid_correspondence_threshold_)
    {
      valid_correspondences = 1;
      mean_reprojected_object_points = mean_reprojected_object_points / num_valid_correspondences;
      MatrixXYd object_points_matrix(4, object_points_.size());
      for (unsigned kk = 0; kk < object_points_.size(); ++kk)
      {
        object_points_matrix.col(kk) = object_points_(kk);
      }
      object_points_matrix.conservativeResize(3, object_points_matrix.cols());
      mean_reprojected_object_points.conservativeResize(3, mean_reprojected_object_points.cols());
      predicted_pose_ = computeTransformation(object_points_matrix, mean_reprojected_object_points);
    }
    else
    { //valid_correspondences = 1; // test, as all 'reasonable' P3P solutions are used, try to initialise with them
    	if (num_valid_correspondences>0)
    		PubData.Flag_Fail[objectNumber] = 7; // not enough valid correspondences
    	else
    		PubData.Flag_Fail[objectNumber] = 8; // certainty treshold criteria failed for all estimations
    }

  }

  return valid_correspondences;
}

unsigned PoseEstimator::initialise(int objectNumber)
{
  if (bUseParticleFilter == 1)
    {
      if (image_points_.size()<object_points_.size())
      {
          ROS_INFO("not enough markers for a initialisation could be detected");
          PubData.Flag_Fail[objectNumber] = 10; // not enough markers for the initialisaion could be detected
          return 0;
      }
    }
  else
    {
      if (image_points_.size()<min_num_leds_detected_)
      {
	  ROS_INFO("not enough markers for a initialisation could be detected");
	  PubData.Flag_Fail[objectNumber] = 10; // not enough markers for the initialisaion could be detected
	  return 0;
      }
    }



  // Combinations of seen points
  RowXu seen_points_working_vector;
  seen_points_working_vector.setLinSpaced(image_points_.size(), 1, image_points_.size());
  MatrixXYu seen_points_combinations = Combinations::combinationsNoReplacement(seen_points_working_vector, 3);
  unsigned num_seen_points_combinations = seen_points_combinations.rows();

  // Permutations of object points
  MatrixXYu object_points_permutations = Combinations::permutationsNoReplacement(object_points_.size(), 3);
  unsigned num_object_points_permutations = object_points_permutations.rows();

  MatrixXYu hist_corr;
  hist_corr.setZero(image_points_.size(), object_points_.size());

  Eigen::Matrix3d feature_vectors, world_points;

  // for every combination of 3 seen points, we have to iterate through all
  // the possible permutations of 3 object points.
  for (unsigned i = 0; i < num_seen_points_combinations; ++i)
  {

    Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;

    // Build up matrix of unit feature vectors
    feature_vectors.col(0) = image_vectors_(seen_points_combinations(i, 0) - 1);
    feature_vectors.col(1) = image_vectors_(seen_points_combinations(i, 1) - 1);
    feature_vectors.col(2) = image_vectors_(seen_points_combinations(i, 2) - 1);


    Eigen::Vector2d d1 = image_points_(seen_points_combinations(i, 0) - 1);
    Eigen::Vector2d d2 = image_points_(seen_points_combinations(i, 1) - 1);
    Eigen::Vector2d d3 = image_points_(seen_points_combinations(i, 2) - 1);
    double threshDist = 10000*100; // 100*100
    double threshDist2 = 10000*100; // 100*100

    if (squareDist(d1,d2)>threshDist)
	continue;
    else if (squareDist(d1,d3)>threshDist)
	continue;
    else if (squareDist(d2,d3)>threshDist)
	continue;

    Eigen::Vector2d dm = Eigen::Vector2d((d1(0)+d2(0)+d3(0))/3,(d1(1)+d2(1)+d3(1))/3);
    int cd = 0;
    bool noEst = true;
    for (unsigned kk = 0; kk<image_points_.size(); kk++)
      {
	if (squareDist(dm,image_points_(kk))<threshDist2)
	  cd++;

	//if (cd == 5) // check if at least 5 detections are cloe enough to be a UAV
	  //break;

      }

    if (cd<5) // check if at least 5 detections are cloe enough to be a UAV
      continue;


    // Find unused image points
    unsigned total_unused_im_points = image_points_.size() - 3;
    //List2DPoints unused_im_points(total_unused_im_points); // Vector to hold the indices of the unused image points
    //VectorXu unused_im_points_idx(total_unused_im_points);
    List2DPoints unused_im_points(cd-3); // Vector to hold the indices of the unused image points
    VectorXu unused_im_points_idx(cd-3);
    unsigned num_unused_im_points = 0;
    bool already_used_im_point = 0;
    for (unsigned kk = 0; kk < image_points_.size(); ++kk)
    {
      already_used_im_point = 0;
      for (unsigned ii = 0; ii < 3; ++ii)
      {
        if (seen_points_combinations(i, ii) - 1 == kk)
          already_used_im_point = 1;
      }
      if (!already_used_im_point && (squareDist(dm,image_points_(kk))<threshDist2)) // do only consider points which are close enough together
      {
        unused_im_points(num_unused_im_points) = image_points_(kk);
        unused_im_points_idx(num_unused_im_points) = kk;
        num_unused_im_points++;
      }
      if (num_unused_im_points == total_unused_im_points)
        break;
    }
    //unused_im_points.conservativeResize(num_unused_im_points, 1);
    //unused_im_points_idx.conservativeResize(num_unused_im_points, 1);
    for (unsigned j = 0; j < num_object_points_permutations; ++j)
    {

      // Build up matrix of world points
      Eigen::Vector4d temp_point;
      temp_point = object_points_(object_points_permutations(j, 0) - 1);
      world_points.col(0) = temp_point.head<3>();
      temp_point = object_points_(object_points_permutations(j, 1) - 1);
      world_points.col(1) = temp_point.head<3>();
      temp_point = object_points_(object_points_permutations(j, 2) - 1);
      world_points.col(2) = temp_point.head<3>();

      // Compute the poses using the P3P algorithm
      int executed_correctly = P3P::computePoses(feature_vectors, world_points, solutions);

      // If the P3P algorithm found a solution (i.e. the world points were not aligned), then continue
      if (executed_correctly == 0)
      {

        Eigen::Matrix4d H_o_c_current;

        // Find the unused image and object points
        unsigned total_unused_object_points = object_points_.size() - 3;

        List4DPoints unused_object_points(total_unused_object_points); // Vector to hold the indexes of the unused object points
        VectorXu unused_object_points_idx(total_unused_object_points);
        unsigned num_unused_object_points = 0;
        bool already_used_object_point = 0;
        for (unsigned ll = 0; ll < object_points_.size(); ++ll)
        {
          already_used_object_point = 0;
          for (unsigned jj = 0; jj < 3; ++jj)
          {
            if (object_points_permutations(j, jj) - 1 == ll)
              already_used_object_point = 1;
          }
          if (!already_used_object_point)
          {
            unused_object_points(num_unused_object_points) = object_points_(ll);
            unused_object_points_idx(num_unused_object_points) = ll;
            num_unused_object_points++;
          }
          //Break if found all the unused object points
          if (num_unused_object_points == total_unused_object_points)
            break;
        }

        // Loop through the four solutions provided by the P3P algorithm
        for (unsigned k = 0; k < 4; ++k)
        {
	  if (k>0) // if a solution appears the second time (imaginary part was neglected) then neclegt the calculation
	  {
	    if ((solutions(k)-solutions(k-1)).all() == 0)
		continue;
	  }

          H_o_c_current.setIdentity();
          H_o_c_current.block<3, 4>(0, 0) = solutions(k);

          if (isFinite(H_o_c_current))
          {
            //Back-project the unused object points onto the image plane
            List2DPoints unused_back_projected_object_points(total_unused_object_points);
            Eigen::Vector3d temp;
            for (unsigned m = 0; m < total_unused_object_points; ++m)
            {
              unused_back_projected_object_points(m) = project2d(unused_object_points(m), H_o_c_current.inverse());
            }
            // calculate closet image/object point pairs
            Eigen::VectorXd min_distances;
            MatrixXYu pairs = calculateMinDistancesAndPairs(unused_im_points, unused_back_projected_object_points,
                                                            min_distances);
            // Check that at least one of the points was within the threshold
            unsigned count_within_pixel_threshold = 0;
            for (unsigned ll = 0; ll < min_distances.size(); ++ll)
            {
              if (min_distances(ll) < back_projection_pixel_tolerance_)
              {
                count_within_pixel_threshold++;
              }
            }
            if (count_within_pixel_threshold > 0)
            {
              unsigned im_idx;
              unsigned obj_idx;
              for (unsigned mm = 0; mm < 3; ++mm)
              {
                im_idx = seen_points_combinations(i, mm) - 1; // image point index
                obj_idx = object_points_permutations(j, mm) - 1; // object point index
                hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
              }
              for (unsigned nn = 0; nn < min_distances.size(); ++nn)
              {
                if (min_distances(nn) < back_projection_pixel_tolerance_)
                {
                  im_idx = unused_im_points_idx(pairs(nn, 0) - 1); // image point index
                  obj_idx = unused_object_points_idx(pairs(nn, 1) - 1); // object point index
                  hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
                }
              }
            }
          }
        }
      }
    }
  }
  if (!(hist_corr.array() == 0).all())
  {
      // returns a vector filled with correspondences sorted that the best one is in the first entry
      std::vector<VectorXuPairs> correspondencesVector = correspondencesFromHistogram(hist_corr, true);

      if (correspondencesVector.size() == 0)
	PubData.Flag_Fail[objectNumber] = 11; // no correspondences from hisotgramm


      int foundSol = 0;
      int NumberOfP3PEstimation = 1; // number of 'reasonable' correspondeces calculated with the P3P algorithm (use =1 as it will be subtracted from N_Particle
      int firstMatch = 0;
      int numNeedCorr;
      VectorXuPairs correspondences_optimisation;
      Eigen::Matrix4d predicted_pose_optimisation;

      for (int possibleCorrespondences = 0; possibleCorrespondences<correspondencesVector.size(); possibleCorrespondences++)
      {
	correspondences_ = correspondencesVector[possibleCorrespondences];

	if (bUseParticleFilter) // define number of needed correspondences for acceptence of initialisation
	  numNeedCorr = object_points_.size();
	else
	  numNeedCorr = min_num_leds_detected_;

	if (checkCorrespondences(NumberOfP3PEstimation, objectNumber, numNeedCorr) == 1 && NumberOfP3PEstimation<N_Particle) // store the P3P pose estimation as long as enough 'free' particles are available
	{
	 foundSol = 1; // Found a solution

	 if (firstMatch == 0)
	 {
	   firstMatch++;
	   correspondences_optimisation = correspondences_; // store the first matching correspondences
	   predicted_pose_optimisation = predicted_pose_; // store the first matching pose prediction
	 }
	}
      }

      int NumOfReasonableEstimations = NumberOfP3PEstimation - 1; // use -1 as last index of PoseParticle.size() is N_Particle-1 (first element has index 0)
      while(NumberOfP3PEstimation<N_Particle && foundSol==1 && bUseParticleFilter) // if still 'free' PoseParticles are available, set them to the ones already calculated (only if at least one 'reasonable' particle could be found)
      {
	  PoseParticle[N_Particle - NumberOfP3PEstimation] = PoseParticle[N_Particle - NumberOfP3PEstimation + NumOfReasonableEstimations];
	  NumberOfP3PEstimation++;
      }

      //ROS_DEBUG_STREAM("The transform: \n" << NumberOfP3PEstimation);
      correspondences_ = correspondences_optimisation; 	// use the correct correspondences for the optimisation
      predicted_pose_ = predicted_pose_optimisation;	// use the best prediction for the optimisation

      return foundSol;

    /*correspondences_ = (hist_corr);

    if (checkCorrespondences() == 1)
    {
      return 1; // Found a solution
    }
    else
    {
      //PubData.Flag_Fail = 4; // correspondences failed detailed error in correspondences
      return 0; // Failed to find a solution
    }*/  // comment out till here
  }
  else
  {
	PubData.Flag_Fail[objectNumber] = 12; // histogram failed, all entries are zero
    return 0; // Failed to find a solution
  }

}

void PoseEstimator::setPredictedTime(double time)
{
  predicted_time_ = time;
}

double PoseEstimator::getPredictedTime()
{
  return predicted_time_;
}

PoseEstimator::PubData_struct PoseEstimator::getPublisherData()
{
	return PubData;
}



void PoseEstimator::optimisePose()
{
  // Using a Gauss-Newton Optimisation

  const double converged = 1e-13;
  const unsigned max_itr = 500;

  Eigen::Matrix4d T_new;
  Eigen::Matrix4d T_old = predicted_pose_;
  Eigen::Matrix4d predicted_pose_init = predicted_pose_;
  Matrix6d A;
  Vector6d b;
  Eigen::Matrix2d R; // Covariance matrix of the image points. Assume the image points points are independent
  R.setIdentity(); // Assume the variance is one pixel in u and v.
  Matrix2x6d J;
  Eigen::Vector2d focal_lengths;
  focal_lengths(0) = camera_matrix_K_.at<double>(0, 0);
  focal_lengths(1) = camera_matrix_K_.at<double>(1, 1);
  double e_init = 0; // initial error
  double e_end = 0; // error when max number of iterations is reached
  double e_new = 0;
  double e_old = 0;
  double lambda = 0.1;
  double lambdaIncr = 2;
  double lambdaDecr = 4;
  double counter = 0;

  //Eigen::Matrix4d predicted_pose_1 = predicted_pose_;


  Vector6d dT;

  for (unsigned i = 0; i < max_itr; ++i)
  {
    A.setZero();
    b.setZero();

    double e_new = 0;

    // Compute the initial errors/residual
    for (unsigned j = 0; j < correspondences_.rows(); ++j)
    {
      if (correspondences_(j, 1) == 0)
        continue;

      // Project point into image plane
      Eigen::Vector2d p_image_plane = project2d((Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1),
                                                predicted_pose_);


      // Calculate the error
      Eigen::Vector2d e = image_points_(correspondences_(j, 1) - 1) - p_image_plane;

      if (i == 0)
	{e_init =+ e.norm();} // store the initial error for an evaluation of the algorithm at the end, if it does NOT converge
      else if (i+1 == max_itr)
	{e_end =+ e.norm();} // store the error at the last iteration for the evaluation at the end

     // e_new += e.norm();

      // Compute Jacobian
      J = computeJacobian(predicted_pose_, (Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1), focal_lengths);

      A.noalias() += J.transpose() * R.inverse() * J;
      b.noalias() += J.transpose() * R.inverse() * e;
    }

    dT = A.ldlt().solve(b);


    // Update the model
    T_new = exponentialMap(dT) * predicted_pose_;
    T_old = predicted_pose_;
    predicted_pose_ = T_new;

    // Stop when converged
    if (norm_max(dT) <= converged)
    {
      PubData.numIter = i;
      break;
    }
    if (i+1 == max_itr)
    {
	if (e_init<e_end)
	  { // the error at the beginning of the iteration is smaller than in the end --> the algorithm did diverege --> use the estimation from the beginning
	    predicted_pose_ = predicted_pose_init;
	    ROS_INFO("max iteration in pose optimisation is reached. The optimisation did diverge --> use the initial estimation from the PF");
	  }
	else
	  ROS_INFO("The iteration limit of the pose optimisation is reached. Use last guess for the pose");
    }

  }
/* LEVENBERG-MARQUART

  unsigned i = 0;

  A.setZero();
  b.setZero();
  e_new = 0;

  // Compute the initial errors/residual
  for (unsigned j = 0; j < correspondences_.rows(); ++j)
  {
    if (correspondences_(j, 1) == 0)
      continue;

    // Project point into image plane
    Eigen::Vector2d p_image_plane = project2d((Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1),
					      predicted_pose_1);


    // Calculate the error
    Eigen::Vector2d e = image_points_(correspondences_(j, 1) - 1) - p_image_plane;
    e_new =+ e.norm();

    // Compute Jacobian
    J = computeJacobian(predicted_pose_1, (Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1), focal_lengths);

    A.noalias() += J.transpose() * R.inverse() * J;
    b.noalias() += J.transpose() * R.inverse() * e;
  }


  while (i < max_itr)
    {

      A(0,0) = (1+lambda)*A(0,0);
      A(1,1) = (1+lambda)*A(1,1);
      A(2,2) = (1+lambda)*A(2,2);
      A(3,3) = (1+lambda)*A(3,3);
      A(4,4) = (1+lambda)*A(4,4);
      A(5,5) = (1+lambda)*A(5,5);

      dT = A.ldlt().solve(b);


      // Update the model
      T_new = exponentialMap(dT) * predicted_pose_1;
      T_old = predicted_pose_1;
      predicted_pose_1 = T_new;
      e_old = e_new;

      // Stop when converged
      if (norm_max(dT) <= converged)
      {
        PubData.numIter = i;
        break;
      }

      A.setZero();
      b.setZero();
      e_new = 0;

      // Compute the initial errors/residual
      for (unsigned j = 0; j < correspondences_.rows(); ++j)
      {
        if (correspondences_(j, 1) == 0)
          continue;

        // Project point into image plane
        Eigen::Vector2d p_image_plane = project2d((Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1),
                                                  predicted_pose_1);


        // Calculate the error
        Eigen::Vector2d e = image_points_(correspondences_(j, 1) - 1) - p_image_plane;
        e_new =+ e.norm();

        // Compute Jacobian
        J = computeJacobian(predicted_pose_1, (Eigen::Vector4d)object_points_(correspondences_(j, 0) - 1), focal_lengths);

        A.noalias() += J.transpose() * R.inverse() * J;
        b.noalias() += J.transpose() * R.inverse() * e;
      }

      if (e_new <= e_old)
	{
	  lambda = lambda/lambdaDecr;
	  e_old = e_new;
	  i++;
	  counter = 0;
	}
      else
	{
	  lambda = lambda*lambdaIncr;
	  counter++;
	  if (counter>100)
	    {
	      ROS_INFO("LM optimiser failed");
	      break;
	    }
	}

    } // end while
*/

  //ROS_INFO("total of %d iteration were needed",iter);

  pose_covariance_ = A.inverse();
  //predicted_pose_ = predicted_pose_1;
  //displayMatrix(predicted_pose_-predicted_pose_1);
  //ROS_INFO(" ");

}

void PoseEstimator::updatePose()
{
  previous_pose_ = current_pose_;
  current_pose_ = predicted_pose_;

  if (predicted_time_-current_time_>0.001 || predicted_time_ < current_time_) 	// update time only if more than 1ms has passed since the last call, or the video has started from the beginning
  {										// this is made to prevent issues when more than one UAV is calculated
	  previous_time_ = current_time_;	   // use the time when the image was made, not when the calculation started --> assume image is NOT updated between the calculations of different UAVs
	  current_time_ = predicted_time_;
  }
}

void PoseEstimator::optimiseAndUpdatePose(double & time_to_predict)
{
  optimisePose();

  if (it_since_initialized_ < 2)
  {
    it_since_initialized_++;
  }

  updatePose();
  bPose_updated = true;
  //PubData.Flag_Fail[objectNumber] = 0; // estimation succeeded
}

void PoseEstimator::predictWithROI(double & time_to_predict, const cv::Mat & image)
{
  if (it_since_initialized_ >= 2)
  { // Predict the pose if initialised. If not, the pose will remain the same as the previous step (constant velocity model)
    predictPose(time_to_predict);
  }
  else
  { // If not yet fully initialised (Only one pose calculated so far)
    setPredictedTime(time_to_predict);
  }
  Eigen::Matrix4d dummyIdentity;
  dummyIdentity.setIdentity();
  predictMarkerPositionsInImage(dummyIdentity, current_pose_); // this is wrong, do not use that input, it should be the exponential map

  // Define region of interest (ROI)
  region_of_interest_ = LEDDetector::determineROI(getPredictedPixelPositions(), image.size(), roi_border_thickness_,
                                                  camera_matrix_K_, camera_distortion_coeffs_, region_of_interest_ellipse, 0);
}

void PoseEstimator::findCorrespondencesAndPredictPose(double & time_to_predict, int objectNumber, List2DPoints detected_led_positions) // not used in the PF case
{
  findCorrespondences();
  int NumberOfP3PEstimation = 1;
  if (checkCorrespondences(NumberOfP3PEstimation, objectNumber, min_num_leds_detected_) == 1)
  { // If the correspondences were correct, update the pose
    optimiseAndUpdatePose(time_to_predict);
    PubData.Flag_Fail[objectNumber] = 1;
  }
  else
  { // Reinitialise if the correspondences weren't correct
	PubData.bPred = 0; // brute force approach was used, prediction failed
	
    if (initialise(objectNumber) == 1)
    { // Only update pose if the initialisation found a valid pose.
      optimiseAndUpdatePose(time_to_predict);
      PubData.Flag_Fail[objectNumber] = 0;
    }
    else
    {

    }
  }
}

template<typename DerivedA, typename DerivedB>
  double PoseEstimator::squareDist(const Eigen::MatrixBase<DerivedA>& p1, const Eigen::MatrixBase<DerivedB>& p2)
  {
	return (p1 - p2).squaredNorm();
  }

template<typename Derived>
  bool PoseEstimator::isFinite(const Eigen::MatrixBase<Derived>& x)
  {
    return ((x - x).array() == (x - x).array()).all();
  }

VectorXuPairs PoseEstimator::calculateMinDistancesAndPairs(const List2DPoints & points_a, const List2DPoints & points_b,
                                                           Eigen::VectorXd & min_distances)
{
  unsigned num_points_a = points_a.size();
  unsigned num_points_b = points_b.size();
  VectorXuPairs pairs(num_points_a, 2);

  // Work around since pairs.col(0).setLinSpaced(num_points_a,1,num_points_a) throws a floating point error when num_points_a = 1
  if (num_points_a == 1)
  {
    pairs(0) = 1;
  }
  else
  {
    pairs.col(0).setLinSpaced((int)num_points_a, 1, num_points_a);
  }
  pairs.col(1).setZero();

  double min_dist_squared = INFINITY;
  double dist_squared;
  Eigen::VectorXd min_dists(num_points_a);

  for (unsigned i = 0; i < num_points_a; ++i)
  {
    min_dist_squared = INFINITY;

    for (unsigned j = 0; j < num_points_b; ++j)
    {
      dist_squared = squareDist((Eigen::Vector2d)points_a(i), (Eigen::Vector2d)points_b(j));

      if (dist_squared < min_dist_squared)
      {
        min_dist_squared = dist_squared;
        pairs(i, 1) = j + 1; // Storing values as base 1 indexing
      }
    }

    min_dists(i) = min_dist_squared;

  }

  min_distances = min_dists.cwiseSqrt();

  return pairs;
}

Eigen::Matrix4d PoseEstimator::computeTransformation(const MatrixXYd & object_points,
                                                     const MatrixXYd & reprojected_points)
{
  Eigen::Vector3d mean_object_points = object_points.rowwise().sum() / object_points.cols();
  Eigen::Vector3d mean_reprojected_points = reprojected_points.rowwise().sum() / reprojected_points.cols();
  MatrixXYd object_points_bar = object_points.colwise() - mean_object_points; // object points with zero mean
  MatrixXYd reprojected_points_bar = reprojected_points.colwise() - mean_reprojected_points;

  Eigen::JacobiSVD<MatrixXYd> svd_of_points(object_points_bar * reprojected_points_bar.transpose(),
                                            Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Matrix3d U = svd_of_points.matrixU();
  Eigen::Matrix3d V = svd_of_points.matrixV();

  Eigen::Matrix3d R = V * U.transpose();
  Eigen::Vector3d t = mean_reprojected_points - R * mean_object_points;

  Eigen::Matrix4d transform;
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = t;
  return transform;
}

Matrix2x6d PoseEstimator::computeJacobian(const Eigen::Matrix4d & T_c_o, const Eigen::Vector4d & world_points,
                                          const Eigen::Vector2d & focal_lengths)
{
  // This Jacobian is calculated according to equation A.14 from the the PhD thesis of Ethan Eade
  // See http://ethaneade.com/
  // See http://ethaneade.com/thesis_revised.pdf

  const Eigen::Vector4d point_camera_frame = T_c_o * world_points;
  double x = point_camera_frame(0);
  double y = point_camera_frame(1);
  double z = point_camera_frame(2);
  double z_2 = z * z;
  Matrix2x6d jacobian;
  jacobian(0, 0) = 1 / z * focal_lengths(0);
  jacobian(0, 1) = 0;
  jacobian(0, 2) = -x / z_2 * focal_lengths(0);
  jacobian(0, 3) = -x * y / z_2 * focal_lengths(0);
  jacobian(0, 4) = (1 + (x * x / z_2)) * focal_lengths(0);
  jacobian(0, 5) = -y / z * focal_lengths(0);

  jacobian(1, 0) = 0;
  jacobian(1, 1) = 1 / z * focal_lengths(1);
  jacobian(1, 2) = -y / z_2 * focal_lengths(1);
  jacobian(1, 3) = -(1 + y * y / z_2) * focal_lengths(1);
  jacobian(1, 4) = x * y / z_2 * focal_lengths(1);
  jacobian(1, 5) = x / z * focal_lengths(1);


  return jacobian;
}

Eigen::Matrix4d PoseEstimator::exponentialMap(const Vector6d & twist)
{
  Eigen::Vector3d upsilon = twist.head<3>();
  Eigen::Vector3d omega = twist.tail<3>();

  double theta = omega.norm();
  double theta_squared = theta * theta;

  Eigen::Matrix3d Omega = skewSymmetricMatrix(omega);
  Eigen::Matrix3d Omega_squared = Omega * Omega;
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d V;

  if (theta == 0)
  {
    rotation = Eigen::Matrix3d::Identity();
    V.setIdentity();
  }
  else
  {
    rotation = Eigen::Matrix3d::Identity() + Omega / theta * sin(theta)
        + Omega_squared / theta_squared * (1 - cos(theta));
    V = (Eigen::Matrix3d::Identity() + (1 - cos(theta)) / (theta_squared) * Omega
        + (theta - sin(theta)) / (theta_squared * theta) * Omega_squared);
  }

  Eigen::Matrix4d transform;
  transform.setIdentity();
  transform.block<3, 3>(0, 0) = rotation;
  transform.block<3, 1>(0, 3) = V * upsilon;

  return transform;
}

Vector6d PoseEstimator::logarithmMap(const Eigen::Matrix4d & trans)
{
  Vector6d xi;
  Eigen::Matrix3d R = trans.block<3, 3>(0, 0);
  Eigen::Vector3d t = trans.block<3, 1>(0, 3);
  Eigen::Vector3d w, upsilon;
  Eigen::Matrix3d w_hat;
  Eigen::Matrix3d A_inv;
  double phi = 0;
  double w_norm;

  // Calculate w_hat
  if (R.isApprox(Eigen::Matrix3d::Identity(), 1e-10) == 1)
  {
    // phi has already been set to 0;
    w_hat.setZero();
  }
  else
  {
    double temp = (R.trace() - 1) / 2;
    // Force phi to be either 1 or -1 if necessary. Floating point errors can cause problems resulting in this not happening
    if (temp > 1)
    {
      temp = 1;
    }
    else if (temp < -1)
    {
      temp = -1;
    }

    phi = acos(temp);
    if (phi == 0)
    {
      w_hat.setZero();
    }
    else
    {
      w_hat = (R - R.transpose()) / (2 * sin(phi)) * phi;
    }
  }

  // Extract w from skew symmetrix matrix of w
  w << w_hat(2, 1), w_hat(0, 2), w_hat(1, 0);

  w_norm = w.norm();

  // Calculate upsilon
  if (t.isApproxToConstant(0, 1e-10) == 1)
  {
    A_inv.setZero();
  }
  else if (w_norm == 0 || sin(w_norm) == 0)
  {
    A_inv.setIdentity();
  }
  else
  {
    A_inv = Eigen::Matrix3d::Identity() - w_hat / 2
        + (2 * sin(w_norm) - w_norm * (1 + cos(w_norm))) / (2 * w_norm * w_norm * sin(w_norm)) * w_hat * w_hat;
  }

  upsilon = A_inv * t;

  // Compose twist coordinates vector
  xi.head<3>() = upsilon;
  xi.tail<3>() = w;

  return xi;
}

Eigen::Matrix3d PoseEstimator::skewSymmetricMatrix(const Eigen::Vector3d w)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;
  return Omega;
}

double PoseEstimator::norm_max(const Eigen::VectorXd & v)
{
  double max = -1;
  for (int i = 0; i < v.size(); i++)
  {
    double abs_val = std::abs((double)v(i));
    if (abs_val > max)
    {
      max = abs_val;
    }
  }
  return max;
}

float * PoseEstimator::setP3PEstProb(float error_vec[4]) // , std::vector<float>  Prob_vec) // no more used
{
	// get the total errors (of all estimations)
	float sum = 0;
	int numEstimations = 0; // number of valid P3P estimations
	static float Prob_vec[4];
	//std::vector<float>  Prob_vec;
	for (unsigned i = 0; i<4; i++)
	{
		if (error_vec[i] != -1)
		{
			sum = sum + error_vec[i];
			numEstimations++;
		}
	}
	
	// probability = (TotalError - EstimationError)/(TotalError*(#SuccessfulEstimation-1))
	for (unsigned j = 0; j<4; j++)
	{
		if (numEstimations > 1 && error_vec[j] != -1)
		{
			Prob_vec[j] = (sum - error_vec[j])/(sum*(numEstimations-1));
		}
		else if (error_vec[j] != -1)
		{
			Prob_vec[j] = 1; // only one valid P3P estimation
		}
		else
		{
			Prob_vec[j] = 0;
		}
	}
	return Prob_vec;
}


std::vector<int> PoseEstimator::getCorrespondingDetections(double number, unsigned Base , unsigned numLED)  // no more used
// Base := Numeral System Base (#detections); numLED: number of LED --> length of the correspondence vector
{

	std::vector <int> correspondingDetections;
	//VectorXu correspondingDetections;
	//ROS_INFO("%f", number);
	//ROS_INFO("%u", (unsigned)number);

	for (int i = numLED-1; i>-1; i--) // gives the number: (numLED-1)+(numLED-2)*Base+(numLED-3)*Base²+... = number
	{	//ROS_INFO("%f , %u", number, Base);
		unsigned remainer = unsigned(number)%Base;
		//ROS_INFO("remainer = %u; number = %f; Base = %u",remainer, number, Base);
		number = number/Base; //ROS_INFO("                               %u", remainer);
		correspondingDetections.push_back(remainer); // ROS_INFO("  ");
	}

	std::reverse(correspondingDetections.begin(),correspondingDetections.end());
	return correspondingDetections;

}

void PoseEstimator::displayMatrix(Eigen::Matrix4d inputMatrix) // dummy function for debugging
{
	ROS_INFO("%f %f %f %f", inputMatrix(0,0), inputMatrix(0,1), inputMatrix(0,2), inputMatrix(0,3));
	ROS_INFO("%f %f %f %f", inputMatrix(1,0), inputMatrix(1,1), inputMatrix(1,2), inputMatrix(1,3));
	ROS_INFO("%f %f %f %f", inputMatrix(2,0), inputMatrix(2,1), inputMatrix(2,2), inputMatrix(2,3));
	ROS_INFO("%f %f %f %f", inputMatrix(3,0), inputMatrix(3,1), inputMatrix(3,2), inputMatrix(3,3));
}

double PoseEstimator::calculateEstimationProbability(const List2DPoints & image_pts,
                        const List2DPoints & object_pts, std::vector<VectorXuPairs> & correspondencesVec)
{
  double Probability = 0;
  VectorXuPairs pairs(object_pts.size(),2);
  pairs.setZero();

  //Declare distance matrix
  MatrixXYd distances(image_pts.size(), object_pts.size());
  // Build distance matrix
  for (unsigned i = 0; i < image_pts.size(); ++i)
  {
    for (unsigned j = 0; j < object_pts.size(); ++j)
    {
      distances(i, j) = squareDist((Eigen::Vector2d)image_pts(i), (Eigen::Vector2d)object_pts(j));
    }
  }

  double min_value;
  unsigned row_idx, col_idx;
  int numCorrsespondences = 0;
  std::vector<unsigned> usedDetections; // stores the detections which have already be used in correspondence with another object point
  int numSelfocclusion = 1;

  for (unsigned k = 0; k < std::min(image_pts.size(), object_pts.size()); ++k)
  {
    min_value = distances.minCoeff(&row_idx, &col_idx);
    min_value = sqrt(min_value);

    if (min_value <= back_projection_pixel_tolerance_PF)
    {
	Probability += object_points_.size() + pow((back_projection_pixel_tolerance_ - min_value)/back_projection_pixel_tolerance_,2);
    	pairs(k,0) = col_idx + 1; // set the LED						+1 is needed as the optimization is 1 based
    	pairs(k,1) = row_idx + 1; // set the corresponding detection	+1 is needed as the optimization is 1 based
    	++numCorrsespondences;

    	if (std::any_of(usedDetections.begin(),usedDetections.end(), [&row_idx](unsigned i){return i==row_idx;}))
    		{
    		Probability = Probability-numSelfocclusion*3; // subtract a constant if the estimated pose would have occlusions --> weight the poses without occlusions more
    		numSelfocclusion++;
    		}

    	usedDetections.push_back(row_idx); // store used detections

    	// downgrade correspondences with certain object points
    	//if (!bMarkerDowngrade(col_idx))
    	if (bMarkerDowngrade[col_idx])
    		Probability = Probability-2; // -1 may be adjusted, keep in mind that decision if PF looses track depends on the probability

      //distances.row(row_idx).setConstant(INFINITY); // only set the object points to zero, the detection can be used by different object points (occlusion)
      distances.col(col_idx).setConstant(INFINITY);
    }
    else
      break; // leave the for loop when the pixel tolerance is reached
  }

  pairs.conservativeResize(numCorrsespondences,2);
  correspondencesVec.push_back(pairs);

  return Probability;
}

bool PoseEstimator::checkAmbiguity(std::vector<int> correspondingDetections)
{
	for (int i = 0; i<correspondingDetections.size(); i++)
	{
		for (int j = correspondingDetections.size(); j>i; j--)
		{
			if (correspondingDetections[i] == correspondingDetections[j])
				return true;
		}
	}
	return false;
}

int PoseEstimator::detectionGroups() // not finished, cluster detections
{  // size of the image 480x752 (pixel)


	double distanceThreshold = 240; // 480/2
	int NumOfGroups;
	std::vector<int> dummyVec;

	std::vector<std::vector<int>> clusters; //contains different point clusters which contain the specific detection points

	for (int ii = 0; ii<image_points_.size(); ii++)
		{
			for (int jj = 0; jj<image_points_.size(); jj++)
			{
				dummyVec.clear();
				Eigen::Vector2d dist = image_points_(ii)-image_points_(jj);
				if (dist.norm()<distanceThreshold)
				{
					dummyVec.push_back(std::min(ii,jj));
				}
			}
			clusters.push_back(dummyVec);
		}

	MatrixXYd distOk(clusters.size(),clusters.size());
	distOk.setZero();

	// erase multiple clusters
	for (int k = 0; k<clusters.size(); k++)
	{
		for (int m = k+1; m<clusters.size(); m++)
		{
			if (clusters[k] == clusters[m])
			{
				clusters.erase(clusters.begin() + m);
				m = m-1;
			}
		}
	}

	NumOfGroups = clusters.size();


	return NumOfGroups;
}

unsigned PoseEstimator::P3P_short(VectorXuPairs correspondences_given, int objectNumber)
{ // assumed 3 correspondences are given
	if (image_points_.size()<min_num_leds_detected_)
	{ // not enough image points are detected
		PubData.Flag_Fail[objectNumber] = 13;
		return 0;
	}

  Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;
  MatrixXYu hist_corr;
  hist_corr.setZero(image_points_.size(), object_points_.size());
  Eigen::Matrix3d feature_vectors, world_points;


  // Combinations of correspondences (take only two and release the others the rest of the detection and markers)
  RowXu idxVectorCorrespondences;
  RowXu imgIdxAvl(image_points_.size()-2);  // vector containing the index of the not in the correspondence vector used image points
  RowXu objIdxAvl(object_points_.size()-2); // vector containing the index of the not in the correspondence vector used object points
  idxVectorCorrespondences.setLinSpaced(correspondences_given.rows(), 0, correspondences_given.rows());
  MatrixXYu correspondencesCombinations = Combinations::combinationsNoReplacement(idxVectorCorrespondences, 2); // contains the idx for the used correspondences
  unsigned numCombinations = correspondencesCombinations.rows();


  for (int corrCombIdx = 0; corrCombIdx<numCombinations; corrCombIdx++) // iterate through the different chosen correspondence combinations
  {

      unsigned idx_img = 0;
      unsigned idx_obj = 0;

    // Build up matrix of unit feature vectors (image points) of the chosen correspondences
    feature_vectors.col(0) = image_vectors_(correspondences_given(correspondencesCombinations(corrCombIdx,0), 1) - 1);
    feature_vectors.col(1) = image_vectors_(correspondences_given(correspondencesCombinations(corrCombIdx,1), 1) - 1);

    // Build up matrix of world points
    Eigen::Vector4d temp_point;
    temp_point = object_points_(correspondences_given(correspondencesCombinations(corrCombIdx,0),0) - 1);
    world_points.col(0) = temp_point.head<3>();
    temp_point = object_points_(correspondences_given(correspondencesCombinations(corrCombIdx,1),0) - 1);
    world_points.col(1) = temp_point.head<3>();


    // get the index of the available image and object points (all points except the ones given due to the correspondence)
    for (int i = 0; i<image_points_.size(); i++)
    {
      if (i != (correspondences_given(correspondencesCombinations(corrCombIdx,0),1)-1) && i != (correspondences_given(correspondencesCombinations(corrCombIdx,1),1)-1))
	     {imgIdxAvl(0,idx_img) = i; idx_img++;}
    }
    for (int i = 0; i<object_points_.size(); i++)
    {
      if (i != (correspondences_given(correspondencesCombinations(corrCombIdx,0),0)-1) && i != (correspondences_given(correspondencesCombinations(corrCombIdx,1),0)-1))
	     {objIdxAvl(0,idx_obj) = i; idx_obj++;}
    }
    // number of remaining points (image and object), one less than in the available vector (totally image_points_.size()-3)
    unsigned numOfRemainingImgPts = imgIdxAvl.size()-1;
    unsigned numOfRemainingObjPts = objIdxAvl.size()-1;


    // iterate through all the possible combinations
    for (int i=0; i<numOfRemainingImgPts; i++)
    {
      // get the third image detection
      feature_vectors.col(2) = image_vectors_(imgIdxAvl(i));

      // unused image points
      unsigned total_unused_im_points = image_points_.size() - 3;
      List2DPoints unused_im_points(total_unused_im_points); // Vector with the unused image_points_
      VectorXu unused_im_points_idx(total_unused_im_points); // Vector containing the index of the unused points
      unsigned num_unused_im_points = 0;
      bool already_used_im_point = 0;


      for (unsigned kk = 0; kk < image_points_.size(); ++kk) // get the not used image points
      {
	if ((correspondences_given(correspondencesCombinations(corrCombIdx,0),1)-1) != kk && (correspondences_given(correspondencesCombinations(corrCombIdx,1),1)-1) != kk && imgIdxAvl(i) != kk)
	{ // point with index kk is not used in the correspondences and is not the third image point which was chosen
	  unused_im_points(num_unused_im_points) = image_points_(kk);
	  unused_im_points_idx(num_unused_im_points) = kk;
	  num_unused_im_points++;
	}
	if (num_unused_im_points == total_unused_im_points)
	    break;
      }

      for (int j = 0; j < numOfRemainingObjPts; j++)
      {
	 // reinitialise the solution matrix
	 Eigen::Matrix<Eigen::Matrix<double, 3, 4>, 4, 1> solutions;

	// get the third object point
	temp_point = object_points_(objIdxAvl(j));
	world_points.col(2) = temp_point.head<3>();

	// unused object points
	unsigned total_unused_object_points = object_points_.size() - 3;
	List4DPoints unused_object_points(total_unused_object_points); // Vector to hold the indexes of the unused object points that have correspondences
	VectorXu unused_object_points_idx(total_unused_object_points);
	unsigned num_unused_object_points = 0;
	bool already_used_object_point = 0;
	for (unsigned ll = 0; ll < object_points_.size(); ++ll)
	{
	  if ((correspondences_given(correspondencesCombinations(corrCombIdx,0),0)-1) != ll && (correspondences_given(correspondencesCombinations(corrCombIdx,1),0)-1) != ll && objIdxAvl(j) != ll)
	    { // point with index kk is not used in the correspondences and is not the third object point which was choosen
	      unused_object_points(num_unused_object_points) = object_points_(ll);
	      unused_object_points_idx(num_unused_object_points) = ll;
	      num_unused_object_points++;
	    }
	  //Break if already found all the unused object points
	  if (num_unused_object_points == total_unused_object_points)
		break;
	}
	/* --- Compute the poses using the P3P algorithm --- */
	  int executed_correctly = P3P::computePoses(feature_vectors, world_points, solutions);

	    // If the P3P algorithm found a solution (i.e. the world points were not aligned), then continue
	  if (executed_correctly == 0)
	  {
	    Eigen::Matrix4d H_o_c_current;

	    // Loop through the four solutions provided by the P3P algorithm
	    for (unsigned k = 0; k < 4; ++k)
	    {
	      if (k>0) // if two consecutive solutions (imaginary part was neglected) then neclegt the calculation
	      {
		    if ((solutions(k)-solutions(k-1)).all() == 0)
			    continue;
	      }

	      H_o_c_current.setIdentity();
	      H_o_c_current.block<3, 4>(0, 0) = solutions(k);

	      if (isFinite(H_o_c_current))
	      {
		//Back-project the unused object points onto the image plane
		List2DPoints unused_back_projected_object_points(total_unused_object_points);
		Eigen::Vector3d temp;
		for (unsigned m = 0; m < total_unused_object_points; ++m)
		{
		  unused_back_projected_object_points(m) = project2d(unused_object_points(m), H_o_c_current.inverse());
		}

		Eigen::VectorXd min_distances;
		MatrixXYu pairs = calculateMinDistancesAndPairs(unused_im_points, unused_back_projected_object_points, min_distances); // it is fine that pairs have size of detections

		// Check if at least one of the points was within the threshold
		if (min_distances.minCoeff()<back_projection_pixel_tolerance_) // all object points have to be back projected correctly
		{
		  unsigned im_idx;
		  unsigned obj_idx;
		  for (unsigned mm = 0; mm < 3; ++mm)
		  {
		    im_idx = correspondences_given(mm, 1) - 1; // image point index
		    obj_idx = correspondences_given(mm, 0) - 1; // object point index
		    hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
		  }

		  for (unsigned nn = 0; nn < min_distances.size(); ++nn)
		  {
		    if (min_distances(nn) < back_projection_pixel_tolerance_)
		    {
		      im_idx = unused_im_points_idx(pairs(nn, 0) - 1); // image point index
		      obj_idx = unused_object_points_idx(pairs(nn, 1) - 1); // object point index
		      hist_corr(im_idx, obj_idx) = hist_corr(im_idx, obj_idx) + 1;
		    }
		  }
		}
	      }
	    }
	  }
	  else
	    PubData.Flag_Fail[objectNumber] = 16;

      } // end for; permutations of the detections

    } // end for; combinations of the detections

  } // end for; combinations of the correspondences


  if (!(hist_corr.array() == 0).all())
  {
	  // returns a vector filled with correspondences sorted with the best ones as the first entry
	  std::vector<VectorXuPairs> correspondencesVector = correspondencesFromHistogram(hist_corr, false);

	  if (correspondencesVector.size() == 0)
	    PubData.Flag_Fail[objectNumber] = 14; // no correspondences from hisotgramm


	  int foundSol = 0;
	  int NumberOfP3PEstimation = 1; // is used to count the number of 'reasonable' correspondeces calculated with the P3P algorithm (use =1 as it will be subtracted from N_Particle
	  int firstMatch = 0;
	  VectorXuPairs correspondences_optimisation;
	  Eigen::Matrix4d predicted_pose_optimisation;
	  std::vector<Eigen::Matrix4d> PoseParticle_dummy = PoseParticle; // if the correspondences are too bad, do not use the new calculated particles, use the old ones

	  for (int possibleCorrespondences = 0; possibleCorrespondences<correspondencesVector.size(); possibleCorrespondences++)
	  {
		  correspondences_ = correspondencesVector[possibleCorrespondences];

		  if (checkCorrespondences(NumberOfP3PEstimation, objectNumber, min_num_leds_detected_) == 1 && NumberOfP3PEstimation<N_Particle) // store the P3P pose estimation as long as enough 'free' particles are available
		      {
		       foundSol = 1; // Found a solution

		       if (firstMatch == 0)
		       {
		    	   firstMatch++;
		    	   correspondences_optimisation = correspondences_; // store the first matching correspondences
		    	   predicted_pose_optimisation = predicted_pose_; // store the first matching pose prediction
		       }

		      }
	  }

	  int NumOfReasonableEstimations = NumberOfP3PEstimation - 1; // use -1 as last index of PoseParticle.size() is N_Particle-1 (first element has index 0)
	  while(NumberOfP3PEstimation<N_Particle && foundSol==1) // if still 'free' PoseParticles are available, set them to the ones already calculated (only if at least one 'reasonable' particle could be found)
	  {
		  PoseParticle[N_Particle - NumberOfP3PEstimation] = PoseParticle[N_Particle - NumberOfP3PEstimation + NumOfReasonableEstimations];
		  NumberOfP3PEstimation++;
	  }

	  if (!foundSol)
		  {PoseParticle = PoseParticle_dummy;}// PubData.Flag_Fail[objectNumber] = 6.22;} // do not use the new calculated particles if no correspondence was good enough

	  correspondences_ = correspondences_optimisation; 	// use the correct correspondences for the optimisation
	  predicted_pose_ = predicted_pose_optimisation;	// use the best prediction for the optimisation


	  return foundSol;
  }
  else
  {
	if (PubData.Flag_Fail[objectNumber] != 16)
		PubData.Flag_Fail[objectNumber] = 15; // histogramm failed

    return 0; // Failed to find a solution
  }
}


} // namespace

