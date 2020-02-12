/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: normal_3d.cpp 35361 2011-01-20 04:34:49Z rusu $
 *
 */

#include <pluginlib/class_list_macros.h>
#include "pcl_ros/features/obstacles.h"
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/don.h>
#include "pcl_ros/transforms.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/extract_indices.h>

#include "sensor_msgs/Imu.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))

void 
pcl_ros::Obstacles::emptyPublish (const PointCloudInConstPtr &cloud_in)
{
  PointCloudOut output;
  output.header = cloud_in->header;
  pub_output_.publish (output.makeShared ());
}

void 
pcl_ros::Obstacles::computePublish (const PointCloudInConstPtr &cloud_in,
                                           const PointCloudInConstPtr &surface,
                                           const IndicesPtr &indices)
{

  // // std::vector<int> indices2;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::removeNaNFromPointCloud(*cloud_in,*cloud_in_filtered);


  boost::shared_ptr<std::vector<int>> indices_nan(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*cloud_in, *indices_nan);
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(indices_nan);
  extract.setNegative(false);
  extract.filter(*cloud_in_filtered);


  ////////////////////////////////
  // GET VEHICLE ROLL AND PITCH //
  ////////////////////////////////

  tf::TransformListener listener;
  tf::StampedTransform T_world_imu;
  double imu_roll, imu_pitch, imu_yaw;
/*
  try
  {
    listener.waitForTransform("/world", "/imu_orientation", ros::Time::now(), ros::Duration(0.05) );
    listener.lookupTransform("/world", "/imu_orientation", ros::Time(0), T_world_imu);
    tf::Matrix3x3 m(T_world_imu.getRotation());
    int solution_number = 1;
    m.getEulerYPR(imu_yaw, imu_pitch, imu_roll, solution_number);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("%s",ex.what()); 
    // ros::Duration(0.1).sleep();
  }
*/

  // ROS_INFO("\nRoll   : %.3f\n"
  //          "  Pitch  : %.3f\n"
  //          "  Yaw    : %.3f\n",
  //          RADIANS_TO_DEGREES(roll), RADIANS_TO_DEGREES(pitch), RADIANS_TO_DEGREES(yaw)  );

  /////////////////////////////////
  // TRANSFORM INPUT POINT CLOUD //
  /////////////////////////////////

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  tf::Transform transform_pcl;
  tf::Quaternion q;

  // Note roll and pitch are intentionally backwards due to the image frame to boldy frame transform_pcl. The IMU transform_pcl converts from body to world frame.
  transform_pcl.setRotation( tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_) );
  pcl_ros::transformPointCloud	(	*cloud_in_filtered, *cloud_in_transformed, transform_pcl);

  ///////////////////////
  // CALCULATE NORMALS //
  ///////////////////////

  if (cloud_in_transformed->isOrganized ())
  {
    tree_xyz.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
  }
  else
  {
    // Use KDTree for non-organized data
    tree_xyz.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }

  // Set input pointcloud for search tree
  tree_xyz->setInputCloud (cloud_in_transformed);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne; // parallel compute, uses just as much CPU
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> ne;
  ne.setInputCloud (cloud_in_transformed);
  ne.setSearchMethod (tree_xyz);

  // Set viewpoint, very important so normals are all pointed in the same direction
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ne (new pcl::PointCloud<pcl::PointXYZINormal>);

  ne.setRadiusSearch (normal_radius_);
  ne.compute (*cloud_ne);


  // Assign original xyz data to normal estimate cloud (this is necessary because by default the xyz fields are empty)
  for (int i = 0; i < cloud_in_transformed->points.size(); i++)
  {
    cloud_ne->points[i].x = cloud_in_transformed->points[i].x;
    cloud_ne->points[i].y = cloud_in_transformed->points[i].y;
    cloud_ne->points[i].z = cloud_in_transformed->points[i].z;
  }

  //////////////////////////////////
  // CALCULATE VARIANCE IN HEIGHT //
  //////////////////////////////////

  // Set up KDTree
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZINormal>);
  tree->setInputCloud (cloud_ne);

  // Neighbors containers
  std::vector<int> k_indices; 
  std::vector<float> k_distances;

  float search_radius = height_variance_radius_;
  float var;
  
  // Main Loop
  for (int point_id = 0; point_id < cloud_ne->size (); ++point_id)
  {

    tree->radiusSearch (point_id, search_radius, k_indices, k_distances);

    // For each neighbor
    for (size_t n_id = 0; n_id < k_indices.size (); ++n_id)
    {
  
      float id = k_indices.at (n_id);
      var += (cloud_ne->points[id].z - cloud_ne->points[point_id].z) * (cloud_ne->points[id].z - cloud_ne->points[point_id].z);

    }

    var /= k_indices.size();
    cloud_ne->points[point_id].curvature = var;

  }

  // Create conditional object
  pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointXYZINormal> () );
    
  // Add conditional statements
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x",  pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );  
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y",  pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z",  pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );
  //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::LT, curvature_LT_threshold_)) );
  //range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::GT, curvature_GT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("intensity", pcl::ComparisonOps::LT, intensity_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("intensity", pcl::ComparisonOps::GT, intensity_GT_threshold_)) );

  // Build the filter
  pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_ne);

  // Apply filter
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_condrem (new pcl::PointCloud<pcl::PointXYZINormal>);
  condrem.filter (*cloud_condrem);

  //////////////////
  // REDUCE NOISE //
  //////////////////

  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZINormal>);
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
  // sor.setInputCloud (cloud_condrem);
  // sor.setMeanK (sor_nearest_neighbors_); // Set the number of nearest neighbors to use for mean distance estimation
  // sor.setStddevMulThresh (sor_std_dev_multiplier_); // Set the standard deviation multiplier for the distance threshold calculation
  // sor.filter (*cloud_sor);

  // build the filter
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ror (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::RadiusOutlierRemoval<pcl::PointXYZINormal> ror;
  ror.setInputCloud(cloud_condrem);
  ror.setRadiusSearch(ror_radius_);
  ror.setMinNeighborsInRadius (ror_min_neighbors_);
  ror.filter (*cloud_ror);

  //////////////////////////////////
  // PUBLISH FILTERED POINT CLOUD //
  //////////////////////////////////

  // Estimate the feature
  PointCloudOut output;
  output = *cloud_ror;

  // Publish a Boost shared ptr const data, enforce that the TF frame and the timestamp are copied
  output.header = cloud_in_transformed->header;

  pub_output_.publish (output);

}

typedef pcl_ros::Obstacles Obstacles;
PLUGINLIB_EXPORT_CLASS(Obstacles, nodelet::Nodelet)

