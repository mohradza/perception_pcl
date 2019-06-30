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
#include "pcl_ros/features/diff_normals.h"

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/features/don.h>

#include <pcl/surface/mls.h>

#include "pcl_ros/transforms.h"
#include <pcl/filters/statistical_outlier_removal.h>

void 
pcl_ros::DiffNormals::emptyPublish (const PointCloudInConstPtr &cloud_in)
{
  PointCloudOut output;
  output.header = cloud_in->header;
  pub_output_.publish (output.makeShared ());
}

void 
pcl_ros::DiffNormals::computePublish (const PointCloudInConstPtr &cloud_in,
                                           const PointCloudInConstPtr &surface,
                                           const IndicesPtr &indices)
{

  /////////////////////////////////
  // TRANSFORM INPUT POINT CLOUD //
  /////////////////////////////////

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_transformed (new pcl::PointCloud<pcl::PointXYZ>);
  tf::Transform transform;
  tf::Quaternion q;

  transform.setRotation( tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_) );
  pcl_ros::transformPointCloud	(	*cloud_in, *cloud_in_transformed, transform);

  ///////////////////////
  // FILTER BY NORMALS //
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

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne;
  ne.setInputCloud (cloud_in_transformed);
  ne.setSearchMethod (tree_xyz);

  // Set viewpoint, very important so normals are all pointed in the same direction
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_ne (new pcl::PointCloud<pcl::PointXYZINormal>);

  ne.setRadiusSearch (normal_radius_);
  ne.compute (*cloud_ne);

  float max_val = 10;
  // Assignment part
  for (int i = 0; i < cloud_in_transformed->points.size(); i++)
  {
    cloud_ne->points[i].x = cloud_in_transformed->points[i].x;
    cloud_ne->points[i].y = cloud_in_transformed->points[i].y;
    cloud_ne->points[i].z = cloud_in_transformed->points[i].z;
    if(!(std::isnan(cloud_ne->points[i].normal_x + cloud_ne->points[i].normal_y)))
    {
      cloud_ne->points[i].intensity = std::abs(cloud_ne->points[i].normal_x) + std::abs(cloud_ne->points[i].normal_y);
    }

  }

  pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointXYZINormal> () );
    
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x", pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x", pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );  
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y", pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y", pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z", pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z", pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::LT, curvature_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::GT, curvature_GT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("intensity", pcl::ComparisonOps::LT, intensity_LT_threshold_)) );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("intensity", pcl::ComparisonOps::GT, intensity_GT_threshold_)) );

  // Build the filter
  pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (cloud_ne);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_condrem (new pcl::PointCloud<pcl::PointXYZINormal>);

  // Apply filter
  condrem.filter (*cloud_condrem);

  //////////////////
  // REDUCE NOISE //
  //////////////////

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
  sor.setInputCloud (cloud_condrem);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_sor);

  //////////////////////////////////
  // PUBLISH FILTERED POINT CLOUD //
  //////////////////////////////////

  // Estimate the feature
  PointCloudOut output;
  output = *cloud_sor;

  // Publish a Boost shared ptr const data, enforce that the TF frame and the timestamp are copied
  output.header = cloud_in_transformed->header;

pub_output_.publish (output);

}

// http://docs.pointclouds.org/1.8.1/cvfh_8hpp_source.html
template<typename PointInT, typename PointNT, typename PointOutT>
void pcl_ros::DiffNormals::filterForObstacles(
  const pcl::PointCloud<PointNT> & cloud_in_transformed,
  std::vector<int> &indices_to_use,
  std::vector<int> &indices_out,
  std::vector<int> &indices_in,
  float threshold)
{
  indices_out.resize (cloud_in_transformed.points.size ());
  indices_in.resize (cloud_in_transformed.points.size ());
  
  size_t in, out;
  in = out = 0;

  for (int i = 0; i < static_cast<int> (indices_to_use.size ()); i++)
  {
    if (cloud_in_transformed.points[indices_to_use[i]].curvature > threshold)
    {
      indices_out[out] = indices_to_use[i];
      out++;
    }
    else
    {
      indices_in[in] = indices_to_use[i];
      in++;
    }
  }

  indices_out.resize (out);
  indices_in.resize (in);
}

typedef pcl_ros::DiffNormals DiffNormals;
PLUGINLIB_EXPORT_CLASS(DiffNormals, nodelet::Nodelet)

