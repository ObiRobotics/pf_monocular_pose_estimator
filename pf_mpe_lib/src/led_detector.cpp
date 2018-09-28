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
 * led_detector.cpp
 *
 * Created on: July 29, 2013
 * Author: Karl Schwabe
 *
 * Adapted till: April 30 2016
 * Author: Marco Moos
 */

/**
 * \file led_detector.cpp
 * \brief File containing the function definitions required for detecting LEDs and visualising their detections and the pose of the tracked object.
 *
 */

#include <pf_mpe_lib/led_detector.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <stdlib.h>

#include <opencv2/imgproc/imgproc.hpp>

namespace monocular_pose_estimator
{

// LED detector
void LEDDetector::findLeds(const cv::Mat &image, cv::Rect ROI, const int &threshold_value, const double &gaussian_sigma,
                           const double &min_blob_area, const double &max_blob_area,
                           const double &max_width_height_distortion, const double &max_circular_distortion,
                           List2DPoints &pixel_positions, std::vector<cv::Point2f> &distorted_detection_centers,
                           const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
			   unsigned & number_of_occlusions, unsigned & number_of_false_detections, bool active_markers,
			   bool useOnlineExposeTimeControl, int expose_time_base)
{
  // Threshold the image
  cv::Mat bw_image;
  //cv::threshold(image, bwImage, threshold_value, 255, cv::THRESH_BINARY);
  if (active_markers)
	  cv::threshold(image(ROI), bw_image, threshold_value, 255, cv::THRESH_TOZERO);
  else
	  cv::threshold(image(ROI), bw_image, threshold_value, 255, cv::THRESH_BINARY_INV);

  // Gaussian blur the image
  cv::Mat gaussian_image;
  cv::Size ksize; // Gaussian kernel size. If equal to zero, then the kerenl size is computed from the sigma
  ksize.width = 0;
  ksize.height = 0;
  GaussianBlur(bw_image.clone(), gaussian_image, ksize, gaussian_sigma, gaussian_sigma, cv::BORDER_DEFAULT);

 // cv::imshow( "Gaussian", gaussian_image );

  // Find all contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(gaussian_image.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
  //ROS_INFO("num of contours: %d",(int)contours.size());

  unsigned numPoints = 0; // Counter for the number of detected LEDs

  // Vector for containing the detected points that will be undistorted later
  std::vector<cv::Point2f> distorted_points;

  // Vector containng the size of the detected  distorted_points
  std::vector<double> area_distPts_Vec;

  // Identify the blobs in the image
  for (unsigned i = 0; i < contours.size(); i++)
  {
    double area = cv::contourArea(contours[i]); // Blob area
    cv::Rect rect = cv::boundingRect(contours[i]); // Bounding box
    double radius = (rect.width + rect.height) / 4; // Average radius

    cv::Moments mu;
    mu = cv::moments(contours[i], false);
    cv::Point2f mc;
    mc = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00) + cv::Point2f(ROI.x, ROI.y);


    // Look for round shaped blobs of the correct size
    if (area >= min_blob_area && area <= max_blob_area
        && std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))
            <= max_width_height_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) <= max_circular_distortion
        && std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) <= max_circular_distortion)
    {
      distorted_points.push_back(mc);
      ++numPoints;
      area_distPts_Vec.push_back(area);
    }
    else
    {
   	/*if (area >= max_blob_area || area <= min_blob_area)
    		ROS_INFO("Blob Area, Area: %f; coordinates: x: %f  y: %f", area, mc.x, mc.y);
    	
    	if (std::abs(1 - std::min((double)rect.width / (double)rect.height, (double)rect.height / (double)rect.width))>= max_width_height_distortion)
    		ROS_INFO("Width High distortion, coordinates: x: %f  y: %f", mc.x, mc.y);
    	
    	if (std::abs(1 - (area / (CV_PI * std::pow(rect.width / 2, 2)))) >= max_circular_distortion)
    		ROS_INFO("Circular Distortion Width, coordinates: x: %f  y: %f", mc.x, mc.y);  // causes problems --> hinders LED detection.... fix this issue
    		
    	if (std::abs(1 - (area / (CV_PI * std::pow(rect.height / 2, 2)))) >= max_circular_distortion)
    		ROS_INFO("Circular Distortion Height, coordinates: x: %f  y: %f", mc.x, mc.y);*/
    }
  }

  double area_ROI = ROI.area();
  if (area_distPts_Vec.size() != 0)
    {
      double sum_area_distPts = 0;
      for (int i = 0; i<area_distPts_Vec.size(); i++)
	sum_area_distPts += area_distPts_Vec[i];

      double frac = sum_area_distPts/area_ROI;
      //std::cout << "frac: " << frac << std::endl;

      static int counterIncrExpTime = 0;
      static int counterDecrExpTime = 0;
      bool increase = false;
      bool decrease = false;

      if (frac < 0.013)
      	{
      	  counterIncrExpTime ++;
      	  if (counterIncrExpTime > 500)
      	    {
      	      increase = true;
      	      counterIncrExpTime = 0;
      	      counterDecrExpTime = 0;
      	    }
      	}
      else if (frac > 0.037)
      	{
      	  counterDecrExpTime ++;
      	  if (counterDecrExpTime > 500)
      	    {
      	      decrease = true;
      	      counterIncrExpTime = 0;
      	      counterDecrExpTime = 0;
      	    }
      	}



      if (useOnlineExposeTimeControl && (increase || decrease))
	ExposeTimeControl(increase, decrease, expose_time_base); // bool, bool, int

    }

  // These will be used for the visualization
  distorted_detection_centers = distorted_points;

  if (numPoints > 0) // number of true detections
  {
	  std::vector<cv::Point2f> occludedLEDs = distorted_points; // need to have full size
	  if (number_of_false_detections>0 && distorted_detection_centers.size()>0)
	  {
		  insertFalseDetections(distorted_points, distorted_detection_centers, number_of_false_detections);
	  }
	  if (number_of_occlusions>0)
	  {
		  occludeDetections(distorted_points, distorted_detection_centers, occludedLEDs, number_of_occlusions);
	  }

	 	    
	// add the changes (occlusion, false detection) also to the distored_points
	distorted_points.resize(distorted_detection_centers.size());
	distorted_points = distorted_detection_centers;
	numPoints = distorted_points.size();
	//distorted_detection_centers.resize(numPoints);

	
	 	    
	  
	if (numPoints>0) // number of point used by the algorithm (occluded and fake points included)
	{
			// Vector that will contain the undistorted points
		    std::vector<cv::Point2f> undistorted_points;

		    // Undistort the points
		    cv::undistortPoints(distorted_points, undistorted_points, camera_matrix_K, camera_distortion_coeffs, cv::noArray(),
		                        camera_matrix_K);

		    // Resize the vector to hold all the possible LED points
		    pixel_positions.resize(numPoints);

		    // Populate the output vector of points
		    for (unsigned j = 0; j < numPoints; ++j)
		    {
		      Eigen::Vector2d point;
		      point(0) = undistorted_points[j].x;
		      point(1) = undistorted_points[j].y;
		      pixel_positions(j) = point;
		    }
	}
    
  }
}

cv::Rect LEDDetector::determineROI(List2DPoints pixel_positions, cv::Size image_size, const int border_size,
                                   const cv::Mat &camera_matrix_K, const std::vector<double> &camera_distortion_coeffs,
				   cv::RotatedRect &region_of_interest_ellipse, int ROI_dyn)
{
  double x_min = INFINITY;
  double x_max = 0;
  double y_min = INFINITY;
  double y_max = 0;

  for (unsigned i = 0; i < pixel_positions.size(); ++i)
  {
    if (pixel_positions(i)(0) < x_min)
    {
      x_min = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(0) > x_max)
    {
      x_max = pixel_positions(i)(0);
    }
    if (pixel_positions(i)(1) < y_min)
    {
      y_min = pixel_positions(i)(1);
    }
    if (pixel_positions(i)(1) > y_max)
    {
      y_max = pixel_positions(i)(1);
    }
  }

  /* ---  Calculation of the ROI defined by an ellipse --- */
 /* std::vector<cv::Point2f> Positions;
  std::vector<cv::Point2f> Positions2;
  cv::Point2f Pts;
  for (int i = 0; i < pixel_positions.size(); ++i)
  {
	  Pts.x = pixel_positions(i)(0);
	  Pts.y = pixel_positions(i)(1);
	  Positions.push_back(Pts);
  }

  Pts.x = x_min;
  Pts.y = y_min;
  Positions.push_back(Pts);
  Pts.x = x_max;
  Pts.y = y_max;
  Positions.push_back(Pts);


  distortPoints(Positions, Positions2, camera_matrix_K, camera_distortion_coeffs);// ROS_INFO("Positions2 size:", (int)Positions2.size());

  //region_of_interest_ellipse = cv::fitEllipse(Positions);
  region_of_interest_ellipse = cv::minAreaRect(Positions);  // defines the rotated rectangle for the ellipsid as the ROI
  //region_of_interest_ellipse.size += cv::Size2f(70,70);

  static cv::Point2f M_old; // value will be set in the end
  cv::Point2f M = region_of_interest_ellipse.center;
  float h = region_of_interest_ellipse.size.height;
  float w = region_of_interest_ellipse.size.width;
  float alpha = region_of_interest_ellipse.angle*(-3.14159265/180); // use minus as the angle is defined in clockwise direction

  std::vector<cv::Point2f> Positions_new;
  // 4 corners
  Positions_new.push_back(M + 1.25* cv::Point2f(0.5*(  w*cos(alpha)-h*sin(alpha) ) , 0.5*(  w*sin(alpha)+h*cos(alpha) )));
  Positions_new.push_back(M + 1.25* cv::Point2f(0.5*(  w*cos(alpha)+h*sin(alpha) ) , 0.5*(  w*sin(alpha)-h*cos(alpha) )));
  Positions_new.push_back(M + 1.25* cv::Point2f(0.5*( -w*cos(alpha)+h*sin(alpha) ) , 0.5*( -w*sin(alpha)-h*cos(alpha) )));
  Positions_new.push_back(M + 1.25* cv::Point2f(0.5*( -w*cos(alpha)-h*sin(alpha) ) , 0.5*( -w*sin(alpha)+h*cos(alpha) )));
  // 5th point for the ellipse equation
  if ( (3*((2*w)+h) - sqrt((3*(2*w)+h)*((2*w)+3*h))) > (3*(w+(2*h)) - sqrt((3*w+(2*h))*(w+3*(2*h)))) ) // enlarge the half axis which causes the smaller encirclement in the end
    Positions_new.push_back(M + 1.25* cv::Point2f(0.5*(  0*cos(alpha)-2*h*sin(alpha) ) , 0.5*(  0*sin(alpha)+2*h*cos(alpha) )));
  else
    Positions_new.push_back(M + 1.25* cv::Point2f(0.5*(  2*w*cos(alpha)-0*sin(alpha) ) , 0.5*(  2*w*sin(alpha)+0*cos(alpha) )));

  //region_of_interest_ellipse = cv::minAreaRect(Positions_new);
  region_of_interest_ellipse = cv::fitEllipse(Positions_new);

//  region_of_interest_ellipse.size.height = region_of_interest_ellipse.size.height*1.5;
//  region_of_interest_ellipse.size.width = region_of_interest_ellipse.size.width*1.5;


   * dM = M-M_old
   * height = height * (1 + dM(0) * fac)
   * width  = width  * (1 + dM(1) * fac)
   * fac ist die unsicherheit welche grösser wird je weiter der ROI sich bewegt


  if (ROI_dyn == 1)
    { // enlarge the ellipse depending on its speed
      cv::Point2f M_diff = region_of_interest_ellipse.center-M_old;
      int fac = 1;
      float dx = std::abs(M_diff.x);
      float dy = std::abs(M_diff.y);
      float dM = dx+dy;
      region_of_interest_ellipse.size.width =  region_of_interest_ellipse.size.width  * (1+fac*0.5*std::abs(dx*cos(alpha)-dy*sin(alpha))/dM);
      region_of_interest_ellipse.size.height = region_of_interest_ellipse.size.height * (1+fac*0.5*std::abs(dx*sin(alpha)+dy*cos(alpha))/dM);
    }
  else
    {
      region_of_interest_ellipse.size.height = region_of_interest_ellipse.size.height * 1.5;
      region_of_interest_ellipse.size.width = region_of_interest_ellipse.size.width * 1.5;
    }




  M_old = region_of_interest_ellipse.center;




   * http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=findcontours
   * http://docs.opencv.org/2.4/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=fitellipse#fitellipse
   * http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/bounding_rotated_ellipses/bounding_rotated_ellipses.html

*/

  std::vector<cv::Point2f> undistorted_points;

  undistorted_points.push_back(cv::Point2f(x_min, y_min));
  undistorted_points.push_back(cv::Point2f(x_max, y_max));

  std::vector<cv::Point2f> distorted_points;

  // Distort the points
  distortPoints(undistorted_points, distorted_points, camera_matrix_K, camera_distortion_coeffs);

  double x_min_dist = distorted_points[0].x;
  double y_min_dist = distorted_points[0].y;
  double x_max_dist = distorted_points[1].x;
  double y_max_dist = distorted_points[1].y;

  double x0 = std::max(0.0, std::min((double)image_size.width, x_min_dist - border_size));
  double x1 = std::max(0.0, std::min((double)image_size.width, x_max_dist + border_size));
  double y0 = std::max(0.0, std::min((double)image_size.height, y_min_dist - border_size));
  double y1 = std::max(0.0, std::min((double)image_size.height, y_max_dist + border_size));

  cv::Rect region_of_interest;

  // if region of interest is too small, use entire image
  // (this happens, e.g., if prediction is outside of the image)
  if (x1 - x0 < 1 || y1 - y0 < 1)
  {
    region_of_interest = cv::Rect(0, 0, image_size.width, image_size.height);
  }
  else
  {
    region_of_interest.x = x0;
    region_of_interest.y = y0;
    region_of_interest.width = x1 - x0;
    region_of_interest.height = y1 - y0;
  }

  return region_of_interest;
}

void LEDDetector::distortPoints(const std::vector<cv::Point2f> & src, std::vector<cv::Point2f> & dst,
                                const cv::Mat & camera_matrix_K, const std::vector<double> & distortion_matrix)
{
  dst.clear();
  double fx_K = camera_matrix_K.at<double>(0, 0);
  double fy_K = camera_matrix_K.at<double>(1, 1);
  double cx_K = camera_matrix_K.at<double>(0, 2);
  double cy_K = camera_matrix_K.at<double>(1, 2);

  double k1 = distortion_matrix[0];
  double k2 = distortion_matrix[1];
  double p1 = distortion_matrix[2];
  double p2 = distortion_matrix[3];
  double k3 = distortion_matrix[4];

  for (unsigned int i = 0; i < src.size(); i++)
  {
    // Project the points into the world
    const cv::Point2d &p = src[i];
    double x = (p.x - cx_K) / fx_K;
    double y = (p.y - cy_K) / fy_K;
    double xCorrected, yCorrected;

    // Correct distortion
    {
      double r2 = x * x + y * y;

      // Radial distortion
      xCorrected = x * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
      yCorrected = y * (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);

      // Tangential distortion
      xCorrected = xCorrected + (2. * p1 * x * y + p2 * (r2 + 2. * x * x));
      yCorrected = yCorrected + (p1 * (r2 + 2. * y * y) + 2. * p2 * x * y);
    }

    // Project coordinates onto image plane
    {
      xCorrected = xCorrected * fx_K + cx_K;
      yCorrected = yCorrected * fy_K + cy_K;
    }
    dst.push_back(cv::Point2d(xCorrected, yCorrected));
  }
}


void LEDDetector::occludeDetections(std::vector<cv::Point2f> distorted_points, 
			  std::vector<cv::Point2f> & distorted_detection_centers, std::vector<cv::Point2f> & occludedLEDs, int number_of_occlusions)
{
	// occludedLEDs are not used yet. This vector can be used to seperate them from the other detections. At the moment it is ont used than the visualisation of the occlusions also uses the distored_detection_centers


	// initialize random point/random number
	cv::Point2f rndPt;
	std:: srand(time(NULL));
	int numOccl = 0; // actual number of occlusions
	int numDetLED = distorted_points.size(); // number of true detections (no false detections included)
	int numAllDet = distorted_detection_centers.size();
	int numOccl_max = std::min(numDetLED, number_of_occlusions); // max possible occlusions
	int randomNumber = 0;
	
	for (int nn = 0; nn<numOccl_max; ++nn)
	{
		int bOcclusion = rand() % 2; // random integer between [0 1] --> 0 or 1; decide if a detection gets occluded or not
		if(bOcclusion==1)
		{
			randomNumber = rand() % (numDetLED-nn); // chose a rand number between 0 and numDetLED-nn-1
			distorted_detection_centers.push_back(-1*distorted_detection_centers[randomNumber]); // use -1* to identify as an occlusion --> will be 
																								 // back changed in visualisation.cpp
			//distorted_detection_centers.erase(randomNumber); // erase cant be used, as the functions are static!!!, use the following for loop instead
			
			int aa = 0;
			for (int mm = 0; mm<numAllDet; ++mm)
			{
				if (mm != randomNumber)
				{
					distorted_detection_centers[aa] = distorted_detection_centers[mm];
					++aa;
				}
			}
		}
	}
}

void LEDDetector::insertFalseDetections(std::vector<cv::Point2f> distorted_points, 
			  std::vector<cv::Point2f> & distorted_detection_centers, int number_of_false_detections)
{

	cv::Point2f rndPt;
	std:: srand(time(NULL));
	for (int falsedet = 0; falsedet<number_of_false_detections; ++falsedet)
	{
	   	int sumx= 0;
	   	int sumy = 0;
	   	int maxDist = 5;
	   	//int oldNumPts = numPoints;
	 	
	   	/*
	   	for (int i = 0; i<numPoints; i++)
	   	{
	   		sumx += distorted_points[i].x;
	   		sumy += distorted_points[i].y;
	  	}
	  	*/
	  	int signx = 2*(rand() % 2 -0.5);
	   	int signy = 2*(rand() % 2 -0.5);
	    int xRdn = rand() % maxDist + 1; // random number between 1 and 100
		int yRdn = rand() % maxDist + 1; // random number between 1 and 100
		int numDet = rand() % distorted_points.size();
		  //ROS_INFO("22");
		//rndPt = cv::Point2f(sumx/oldNumPts+xRdn-(maxDist/2), sumy/oldNumPts+yRdn-(maxDist/2));  // close to the center
	  	//rndPt = cv::Point2f(sumx/oldNumPts+(xRdn+maxDist/2)*signx, sumy/oldNumPts+(yRdn+maxDist/2)*signy); // away from the center
	  	//rndPt = cv::Point2f(sumx/oldNumPts+(xRdn*signx), sumy/oldNumPts+(yRdn*signy)); // somewhere
	   	rndPt = cv::Point2f(distorted_points[numDet].x+(xRdn*signx), distorted_points[numDet].y+(yRdn*signy));
	   	distorted_detection_centers.push_back(rndPt);
	   	//numPoints ++;
	}
}

void LEDDetector::ExposeTimeControl(bool increase, bool decrease, int expose_time_base)
{

  static int expose_time = expose_time_base;
  std::ostringstream cmd;

  if (increase && !decrease)  // increase the expose time
  {
    expose_time = expose_time + 0.2*expose_time_base;
    cmd <<  "rosrun dynamic_reconfigure dynparam set /mv_25001329 expose_us " << expose_time;
    int ret = system(cmd.str().c_str());
    ROS_INFO("increase expose time to %d ms",expose_time);
  }
  else if (!increase && decrease)  // decrease expose time
  {
    expose_time = expose_time - 0.2*expose_time_base;
    cmd <<  "rosrun dynamic_reconfigure dynparam set /mv_25001329 expose_us " << expose_time;
    int ret = system(cmd.str().c_str());
    ROS_INFO("decrease expose time to %d ms",expose_time);
  }


}




} // namespace
