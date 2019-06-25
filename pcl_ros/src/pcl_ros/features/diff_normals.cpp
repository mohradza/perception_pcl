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

// pcl_ros::transformPointCloud	(	const pcl::PointCloud< PointT > & 	cloud_in,
//                                 pcl::PointCloud< PointT > & 	cloud_out,
//                                 const tf::Transform & 	transform)		

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
tf::Transform transform;
tf::Quaternion q;
transform.setRotation( tf::createQuaternionFromRPY(-M_PI/2,0,0) );
pcl_ros::transformPointCloud	(	*cloud_in, *cloud, transform);

  // /////////////////////////////////////
  // // FILTER BY DIFFERENCE OF NORMALS //
  // /////////////////////////////////////

  // // Create a KD-Tree
  // if (cloud->isOrganized ())
  // {
  //   tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
  // }
  // else
  // {
  //   // Use KDTree for non-organized dataPointNormal
  //   tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  // }

  // // Set input pointcloud for search tree
  // tree->setInputCloud (cloud);

  // // Check if small scale is smaller than large scale
  // if (don_radius_1_ >= don_radius_2_)
  // {
  //   std::cerr << "Error: Large scale must be > small scale!" << std::endl;
  //   exit (EXIT_FAILURE);
  // }

  // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  // ne.setInputCloud (cloud);
  // ne.setSearchMethod (tree);

  // // Set viewpoint, very important so normals are all pointed in the same direction
  // ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // // Calculate normals with the small scale
  // std::cout << "Calculating normals for scale..." << don_radius_1_ << std::endl;
  // pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

  // ne.setRadiusSearch (don_radius_1_);
  // ne.compute (*normals_small_scale);

  // // Calculate normals with the large scale
  // std::cout << "Calculating normals for scale..." << don_radius_2_ << std::endl;
  // pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

  // ne.setRadiusSearch (don_radius_2_);
  // ne.compute (*normals_large_scale);

  // // Create output cloud for Difference of Normals (DoN) results
  // pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud (new pcl::PointCloud<pcl::PointNormal>);
  // pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *don_cloud);

  // // Create DoN operator
  // std::cout << "Calculating DoN... " << std::endl;
  // pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;

  // don.setInputCloud (cloud);
  // don.setNormalScaleLarge (normals_large_scale);
  // don.setNormalScaleSmall (normals_small_scale);

  // // Check if can initialize DoN feature operator
  // if (!don.initCompute ())
  // {
  //   std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
  //   exit (EXIT_FAILURE);
  // }

  // // Compute DoN
  // don.computeFeature (*don_cloud);

  // // Build the condition for filtering
  // std::cout << "Filtering out DoN mag < " << don_LT_threshold_ << "..." << std::endl;
  // std::cout << "Filtering out DoN mag > " << don_GT_threshold_ << "..." << std::endl;

  // pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointNormal> () );
  // range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::LT, don_LT_threshold_)) );
  // range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, don_GT_threshold_)) );
  
  // // Build the filter
  // pcl::ConditionalRemoval<pcl::PointNormal> condrem;
  // condrem.setCondition (range_cond);
  // condrem.setInputCloud (don_cloud);

  // pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud_filtered_normals (new pcl::PointCloud<pcl::PointNormal>);

  // // Apply filter
  // condrem.filter (*don_cloud_filtered_normals);

  // // Print size of filtered output
  // std::cout << "Filtered Pointcloud: " << don_cloud_filtered_normals->points.size () << " data points." << std::endl;

  ///////////////////////
  // FILTER BY NORMALS //
  ///////////////////////

  if (cloud->isOrganized ())
  {
    tree2.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
  }
  else
  {
    // Use KDTree for non-organized data
    tree2.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }

  // Set input pointcloud for search tree
  tree2->setInputCloud (cloud);

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne2;
  ne2.setInputCloud (cloud);
  ne2.setSearchMethod (tree2);

  // Set viewpoint, very important so normals are all pointed in the same direction
  ne2.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud2 (new pcl::PointCloud<pcl::PointNormal>);

  ne2.setRadiusSearch (normal_radius_);
  ne2.compute (*don_cloud2);

  // Assignment part
  for (int i = 0; i < cloud->points.size(); i++)
  {
    don_cloud2->points[i].x = cloud->points[i].x;
    don_cloud2->points[i].y = cloud->points[i].y;
    don_cloud2->points[i].z = cloud->points[i].z;
  }

  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond2 ( new pcl::ConditionOr<pcl::PointNormal> () );
    
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_x", pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_x", pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );  
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_y", pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_y", pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("normal_z", pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::LT, curvature_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, curvature_GT_threshold_)) );

  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> condrem2;
  condrem2.setCondition (range_cond2);
  condrem2.setInputCloud (don_cloud2);

  pcl::PointCloud<pcl::PointNormal>::Ptr don_cloud_filtered_normals2 (new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  condrem2.filter (*don_cloud_filtered_normals2);

  // Need to fuse together PointNormal data with PointXYZ
  // https://stackoverflow.com/questions/34400656/how-can-i-compute-a-normal-for-each-point-in-cloud

  // Line 48: http://docs.pointclouds.org/1.7.0/normal__3d_8hpp_source.html
  // Line 189: http://docs.pointclouds.org/1.7.0/feature_8hpp_source.html

  // http://www.meshlab.net/
  // http://meshlabstuff.blogspot.com/2009/09/meshing-point-clouds.html
  // https://pixinsight.com/developer/pcl/doc/html/classpcl_1_1SurfaceSpline.html

  //////////////////
  // REDUCE NOISE //
  //////////////////

  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_filtered_sor (new pcl::PointCloud<pcl::PointNormal>);
  pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
  sor.setInputCloud (don_cloud_filtered_normals2);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered_sor);

  //////////////////////////////////
  // PUBLISH FILTERED POINT CLOUD //
  //////////////////////////////////

  // Estimate the feature
  PointCloudOut output;
  output = *cloud_filtered_sor;

  // Publish a Boost shared ptr const data, enforce that the TF frame and the timestamp are copied
  output.header = cloud->header;

pub_output_.publish (output);

}

// http://docs.pointclouds.org/1.8.1/cvfh_8hpp_source.html
template<typename PointInT, typename PointNT, typename PointOutT>
void pcl_ros::DiffNormals::filterNormalsWithHighCurvature(
  const pcl::PointCloud<PointNT> & cloud,
  std::vector<int> &indices_to_use,
  std::vector<int> &indices_out,
  std::vector<int> &indices_in,
  float threshold)
{
  indices_out.resize (cloud.points.size ());
  indices_in.resize (cloud.points.size ());
  
  size_t in, out;
  in = out = 0;

  for (int i = 0; i < static_cast<int> (indices_to_use.size ()); i++)
  {
    if (cloud.points[indices_to_use[i]].curvature > threshold)
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

