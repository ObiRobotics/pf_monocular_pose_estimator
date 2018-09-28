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
 * visualization.cpp
 *
 *  Created on: Mar 21, 2014
 *      Author: Matthias Faessler
 *
 * Adapted till: April 30 2016
 * Author: Marco Moos
 */

/**
 * \file visualization.cpp
 * \brief File containing the function definitions for all visualization tasks
 *
 */

#include "pf_mpe_lib/visualization.h"
#include <stdio.h>
#include "ros/ros.h"

namespace monocular_pose_estimator
{

// Function that projects the RGB orientation vectors back onto the image
void Visualization::projectOrientationVectorsOnImage(cv::Mat &image, const std::vector<cv::Point3f> points_to_project,
                                                     const cv::Mat camera_matrix_K,
                                                     const std::vector<double> camera_distortion_coeffs, double weight, int colorSwitch)
{

  std::vector<cv::Point2f> projected_points;

  // 0 rotation
  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);

  // 0 translation
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);

  projectPoints(points_to_project, rvec, tvec, camera_matrix_K, camera_distortion_coeffs, projected_points);

  cv::line(image, projected_points[0], projected_points[0] + (projected_points[1]-projected_points[0]) * ((5+2*weight)/1), CV_RGB(255-colorSwitch*128, colorSwitch*128, 0), 3 * weight);
  cv::line(image, projected_points[0], projected_points[0] + (projected_points[2]-projected_points[0]) * ((5+2*weight)/1), CV_RGB(0, 255-colorSwitch*128, colorSwitch*128), 3 * weight);
  cv::line(image, projected_points[0], projected_points[0] + (projected_points[3]-projected_points[0]) * ((5+2*weight)/1), CV_RGB(colorSwitch*128, 0, 255-colorSwitch*128), 3 * weight);

}

void Visualization::createVisualizationImage(cv::Mat &image, std::vector<Eigen::Matrix4d> transform, const cv::Mat camera_matrix_K,
                                             const std::vector<double> camera_distortion_coeffs,
					     std::vector<cv::Rect> region_of_interest_Vec,
					     std::vector<std::vector<cv::Point2f>> distorted_detection_centers_Vec,
					     unsigned number_of_false_detections,
					     unsigned number_of_occlusions,
					     std::vector<bool> found_body_pose, int N_Particle,
					     std::vector<std::vector<Eigen::Matrix4d>> PoseParticle_Vec,
					     std::vector<RowXd> probPart_Vec, bool bUseParticleFilter, std::vector<List4DPoints> object_points_Vec,
					     cv::RotatedRect region_of_interest_ellipse)
{

	std::vector<cv::Point2f> distorted_detection_centers;

	for (int objectNumber = 0; objectNumber < found_body_pose.size(); objectNumber++)
	{

	  if (found_body_pose[objectNumber]) // plot the orientation trivector only if pose estimation succeeded (also ROI)
	  {
	    const double orientation_vector_length = 0.075; //!< Length of the orientation trivectors that will be projected onto the output image

	    Eigen::Matrix4d orientation_vector_points; // Matrix holding the points for the orientation trivector that will be projected onto the output image in the object body frame of reference
	    orientation_vector_points.col(0) << 0, 0, 0, 1;
	    orientation_vector_points.col(1) << orientation_vector_length, 0, 0, 1;
	    orientation_vector_points.col(2) << 0, orientation_vector_length, 0, 1;
	    orientation_vector_points.col(3) << 0, 0, orientation_vector_length, 1;

	    Eigen::Matrix4d visualisation_pts = transform[objectNumber] * orientation_vector_points;

	    std::vector<cv::Point3f> points_to_project;
	    points_to_project.resize(4);

	    points_to_project[0].x = visualisation_pts(0, 0);
	    points_to_project[0].y = visualisation_pts(1, 0);
	    points_to_project[0].z = visualisation_pts(2, 0);
	    points_to_project[1].x = visualisation_pts(0, 1);
	    points_to_project[1].y = visualisation_pts(1, 1);
	    points_to_project[1].z = visualisation_pts(2, 1);
	    points_to_project[2].x = visualisation_pts(0, 2);
	    points_to_project[2].y = visualisation_pts(1, 2);
	    points_to_project[2].z = visualisation_pts(2, 2);
	    points_to_project[3].x = visualisation_pts(0, 3);
	    points_to_project[3].y = visualisation_pts(1, 3);
	    points_to_project[3].z = visualisation_pts(2, 3);

	    projectOrientationVectorsOnImage(image, points_to_project, camera_matrix_K, camera_distortion_coeffs, 3, 1-found_body_pose[objectNumber]);



	    /*------ project predicted points ----- */
	    for (int ObjPtNum = 0; ObjPtNum<object_points_Vec[objectNumber].size(); ObjPtNum++) //object_points_Vec[objectNumber].size(); ObjPtNum++)
	    {
	      Eigen::Vector4d point = object_points_Vec[objectNumber](ObjPtNum);

	      // convert matrix interna
	      Matrix3x4d camera_matrix;
	      for (int i=0; i<3; i++)
	      {
		for (int j=0; j<3; j++)
		{
		  camera_matrix(i, j) = camera_matrix_K.at<double>(i, j);
		}
		camera_matrix(i, 3) = 0.0;
	      }

	      Eigen::Vector4d temp;
	      temp = transform[objectNumber] * point;



/*
	      // Draw the reprojected points onto the image
	      std::vector<cv::Point3f> project1;
	      project1.resize(1);
	      project1[0].x = temp[0];
	      project1[0].y = temp[1];
	      project1[0].z = temp[2];
	      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 0 rotation
	      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 0 translation
	      std::vector<cv::Point2f> projected_points1;
	      projectPoints(project1, rvec, tvec, camera_matrix_K, camera_distortion_coeffs, projected_points1);
	      cv::circle(image, projected_points1[0], (int)10/(std::max(1,(int)temp(2))), CV_RGB(255*(1-objectNumber), 255*objectNumber, 255), 2); // considering distortion
*/



/*	      // Draw the reprojected points onto the image (without distortion)
	      Eigen::Vector3d temp1;
	      temp1 = camera_matrix * transform[objectNumber] * point;
	      temp1 = temp1 / temp1(2);
	      cv::Point2f drawPoint;
	      drawPoint.x = temp1[0];
	      drawPoint.y = temp1[1];

	      cv::circle(image, drawPoint, (int)8/(std::max(1,(int)temp(2))), CV_RGB(0, 255, 255), 2); // not considering distortion
*/

	      //ROS_INFO("drawPoint.x              = %f;  drawPoint.y         = %f",(float) drawPoint.x, (float) drawPoint.y); // not considering distortion
	      //ROS_INFO("projected_points1.x      = %f;  projected_points1.y = %f",(float) projected_points1[0].x, (float) projected_points1[0].y); // considering distortion
	    }


	    if (found_body_pose[objectNumber] && bUseParticleFilter && PoseParticle_Vec[objectNumber].size() != 0  && probPart_Vec[objectNumber].sum() != 0) // make sure the PF is used, and there exist some estimated particles
	    {
	     RowXd normedPartProb = probPart_Vec[objectNumber]/probPart_Vec[objectNumber].maxCoeff();
	     for (int numParticle = 0; numParticle<N_Particle; numParticle++)
	     {
		    Eigen::Matrix4d transform_temp = PoseParticle_Vec[objectNumber][numParticle];
		    Eigen::Matrix4d visualisation_pts = transform_temp * orientation_vector_points;


		    std::vector<cv::Point3f> points_to_project;
		    points_to_project.resize(4);

		    points_to_project[0].x = visualisation_pts(0, 0);
		    points_to_project[0].y = visualisation_pts(1, 0);
		    points_to_project[0].z = visualisation_pts(2, 0);
		    points_to_project[1].x = visualisation_pts(0, 1);
		    points_to_project[1].y = visualisation_pts(1, 1);
		    points_to_project[1].z = visualisation_pts(2, 1);
		    points_to_project[2].x = visualisation_pts(0, 2);
		    points_to_project[2].y = visualisation_pts(1, 2);
		    points_to_project[2].z = visualisation_pts(2, 2);
		    points_to_project[3].x = visualisation_pts(0, 3);
		    points_to_project[3].y = visualisation_pts(1, 3);
		    points_to_project[3].z = visualisation_pts(2, 3);


/*
		    bool dummyColor;
		    if ((numParticle%10) != 0 || numParticle == 1)
		      dummyColor = 1;
		    else
		      dummyColor = 0;
*/


		    projectOrientationVectorsOnImage(image, points_to_project, camera_matrix_K, camera_distortion_coeffs, normedPartProb(0,numParticle), 1-found_body_pose[objectNumber]);
		    //projectOrientationVectorsOnImage(image, points_to_project, camera_matrix_K, camera_distortion_coeffs, normedPartProb(0,numParticle), dummyColor);

/*
		    for (int ObjPtNum = 0; ObjPtNum<object_points_Vec[objectNumber].size(); ObjPtNum++) //object_points_Vec[objectNumber].size(); ObjPtNum++)
		    {
		      Eigen::Vector4d point = object_points_Vec[objectNumber](ObjPtNum);
		      Eigen::Vector4d temp;
		      temp = transform_temp * point;


		      std::vector<cv::Point3f> project1;
		      project1.resize(1);
		      project1[0].x = temp[0];
		      project1[0].y = temp[1];
		      project1[0].z = temp[2];
		      cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 0 rotation
		      cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 0 translation
		      std::vector<cv::Point2f> projected_points1;
		      projectPoints(project1, rvec, tvec, camera_matrix_K, camera_distortion_coeffs, projected_points1);
		      cv::circle(image, projected_points1[0], (int)10/(std::max(1,(int)temp(2))), CV_RGB(0, 255, 255), 2); // considering distortion
		    }
*/


	     }
	    }

	    // Draw region of interest
	      //cv::rectangle(image, region_of_interest_Vec[objectNumber], CV_RGB(255*(1-objectNumber), 255*(1-objectNumber), 255*objectNumber), 2); draw it always

	    //cv::ellipse (image, region_of_interest_ellipse, CV_RGB(30, 200, 200), 2, 8);

	    /*cv::Point2f vertices[4];
	    region_of_interest_ellipse.points(vertices);
	    for (int i = 0; i < 4; i++)
		line(image, vertices[i], vertices[(i+1)%4], CV_RGB(0,255,0));*/
	  }

	  // Draw region of interest
	  cv::rectangle(image, region_of_interest_Vec[objectNumber], CV_RGB(255*(1-objectNumber), 255*(1-objectNumber), 255*objectNumber), 2);


	  for (int detectionNumber = 0; detectionNumber<distorted_detection_centers_Vec[objectNumber].size(); detectionNumber++)
		  distorted_detection_centers.push_back(distorted_detection_centers_Vec[objectNumber][detectionNumber]);

	}
	// check how many LEDs are occluded
	unsigned numbOcclusion = 0;
	for (int jj = 0; jj<number_of_occlusions; jj++)
	{
	  if (distorted_detection_centers[distorted_detection_centers.size() - (jj+1)].x<0)
	  {
		  numbOcclusion++;
		  distorted_detection_centers[distorted_detection_centers.size() - (jj+1)] = -1*distorted_detection_centers[distorted_detection_centers.size() - (jj+1)];
	  }
	  else
	  {
		  break;
	  }
	}

	// Draw a circle around the detected LED
	for (int i = 0; i < distorted_detection_centers.size(); i++)
	{
	 if (i<distorted_detection_centers.size()-number_of_false_detections-numbOcclusion) 	// mark the normal detected LEDs in green
	 {
		 cv::circle(image, distorted_detection_centers[i], 5, CV_RGB(0, 255, 0), 2);
	 }
	 else if (i<distorted_detection_centers.size()-numbOcclusion) 							// mark the fake LED detections
	 {
		 cv::circle(image, distorted_detection_centers[i], 8, CV_RGB(255, 255, 0), 2);
	 }
	 else 														// mark the occluded (true) LED detections
	 {
		 cv::circle(image, distorted_detection_centers[i], 8, CV_RGB(255, 0, 0), 2);
	 }

	}

	//image = image(region_of_interest_Vec[0]);

	//cv::circle(image, cv::Point2f(100,100), 5, CV_RGB(255, 0, 0), 1);
	//cv::circle(image, cv::Point2f(300,300), 5, CV_RGB(255, 0, 0), 1);
	//cv::circle(image, cv::Point2f(300,400), 5, CV_RGB(255, 0, 0), 1);
	// get the textbox which tell if the track was lost
	if (!found_body_pose[0])
	  {
	    putText(image, "Lost track", cvPoint(600,100), CV_FONT_HERSHEY_PLAIN, 1.5, CV_RGB(255,0,0), 2, false);
	  }


}

} // namespace
