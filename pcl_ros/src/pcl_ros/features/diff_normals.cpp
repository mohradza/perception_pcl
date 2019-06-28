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

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  tf::Transform transform;
  tf::Quaternion q;
  transform.setRotation( tf::createQuaternionFromRPY(transform_pcl_roll_, transform_pcl_pitch_, transform_pcl_yaw_) );
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
  //   // Use KDTree for non-organized dataPointXYZINormal
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

  // pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne;
  // ne.setInputCloud (cloud);
  // ne.setSearchMethod (tree);

  // // Set viewpoint, very important so normals are all pointed in the same direction
  // ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // // Calculate normals with the small scale
  // std::cout << "Calculating normals for scale..." << don_radius_1_ << std::endl;
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointXYZINormal>);

  // ne.setRadiusSearch (don_radius_1_);
  // ne.compute (*normals_small_scale);

  // // Calculate normals with the large scale
  // std::cout << "Calculating normals for scale..." << don_radius_2_ << std::endl;
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointXYZINormal>);

  // ne.setRadiusSearch (don_radius_2_);
  // ne.compute (*normals_large_scale);

  // // Create output cloud for Difference of Normals (DoN) results
  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr don_cloud (new pcl::PointCloud<pcl::PointXYZINormal>);
  // pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZINormal>(*cloud, *don_cloud);

  // // Create DoN operator
  // std::cout << "Calculating DoN... " << std::endl;
  // pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointXYZINormal, pcl::PointXYZINormal> don;

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

  // pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond ( new pcl::ConditionOr<pcl::PointXYZINormal> () );
  // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::LT, don_LT_threshold_)) );
  // range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::GT, don_GT_threshold_)) );
  
  // // Build the filter
  // pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem;
  // condrem.setCondition (range_cond);
  // condrem.setInputCloud (don_cloud);

  // pcl::PointCloud<pcl::PointXYZINormal>::Ptr don_cloud_filtered_normals (new pcl::PointCloud<pcl::PointXYZINormal>);

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

  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointXYZINormal> ne2;
  ne2.setInputCloud (cloud);
  ne2.setSearchMethod (tree2);

  // Set viewpoint, very important so normals are all pointed in the same direction
  ne2.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr don_cloud2 (new pcl::PointCloud<pcl::PointXYZINormal>);

  ne2.setRadiusSearch (normal_radius_);
  ne2.compute (*don_cloud2);

  float max_val = 10;
  // Assignment part
  for (int i = 0; i < cloud->points.size(); i++)
  {
    don_cloud2->points[i].x = cloud->points[i].x;
    don_cloud2->points[i].y = cloud->points[i].y;
    don_cloud2->points[i].z = cloud->points[i].z;
    if(!(std::isnan(don_cloud2->points[i].normal_x + don_cloud2->points[i].normal_y)))
    {
      don_cloud2->points[i].intensity = std::max(std::min(don_cloud2->points[i].normal_x + don_cloud2->points[i].normal_y,max_val),-max_val);
    }

  }

  pcl::ConditionOr<pcl::PointXYZINormal>::Ptr range_cond2 ( new pcl::ConditionOr<pcl::PointXYZINormal> () );
    
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x", pcl::ComparisonOps::LT, normal_x_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_x", pcl::ComparisonOps::GT, normal_x_GT_threshold_)) );  
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y", pcl::ComparisonOps::LT, normal_y_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_y", pcl::ComparisonOps::GT, normal_y_GT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z", pcl::ComparisonOps::LT, normal_z_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("normal_z", pcl::ComparisonOps::GT, normal_z_GT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::LT, curvature_LT_threshold_)) );
  range_cond2->addComparison (pcl::FieldComparison<pcl::PointXYZINormal>::ConstPtr ( new pcl::FieldComparison<pcl::PointXYZINormal> ("curvature", pcl::ComparisonOps::GT, curvature_GT_threshold_)) );

  // Build the filter
  pcl::ConditionalRemoval<pcl::PointXYZINormal> condrem2;
  condrem2.setCondition (range_cond2);
  condrem2.setInputCloud (don_cloud2);

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr don_cloud_filtered_normals2 (new pcl::PointCloud<pcl::PointXYZINormal>);

  // Apply filter
  condrem2.filter (*don_cloud_filtered_normals2);

  // Need to fuse together PointXYZINormal data with PointXYZ
  // https://stackoverflow.com/questions/34400656/how-can-i-compute-a-normal-for-each-point-in-cloud

  // Line 48: http://docs.pointclouds.org/1.7.0/normal__3d_8hpp_source.html
  // Line 189: http://docs.pointclouds.org/1.7.0/feature_8hpp_source.html

  // http://www.meshlab.net/
  // http://meshlabstuff.blogspot.com/2009/09/meshing-point-clouds.html
  // https://pixinsight.com/developer/pcl/doc/html/classpcl_1_1SurfaceSpline.html

  //////////////////
  // REDUCE NOISE //
  //////////////////

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_filtered_sor (new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
  sor.setInputCloud (don_cloud_filtered_normals2);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered_sor);

  // //////// NEW STUFF


  // // Create output cloud for Difference of Normals (DoN) results
  // pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_filtered_sor (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::copyPointCloud<pcl::PointXYZINormal, pcl::PointXYZ>(*don_cloud2, *xyz_cloud_filtered_sor);


  // float x;

  // pcl::PointCloud<pcl::PointXYZI> cost_map;
  

  // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  // kdtree.setInputCloud (xyz_cloud_filtered_sor);
  // float radius = 0.25;
  // std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
  // std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

  // for (size_t i = 0; i < 20; ++i)
  // { 

  //   for (size_t j = 0; j < 20; ++j)
  //   { 
    
  //     for (size_t k = 0; k < 20; ++k)
  //     { 

  //       pcl::PointXYZ searchPoint(-5 + i*0.5,-5 + j*0.5,-5 + k*0.5);
  //       if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  //       {
          
  //         for (size_t l = 0; l < pointIdxRadiusSearch.size (); ++l)
  //         {
          
  //           pcl::PointXYZI point;
  //           point.x = don_cloud2->points[ pointIdxRadiusSearch[l] ].x;
  //           point.y = don_cloud2->points[ pointIdxRadiusSearch[l] ].y;
  //           point.z = don_cloud2->points[ pointIdxRadiusSearch[l] ].z;
  //           point.intensity = don_cloud2->points[ pointIdxRadiusSearch[l] ].normal_y + don_cloud2->points[ pointIdxRadiusSearch[l] ].normal_y + 10 * don_cloud2->points[ pointIdxRadiusSearch[l] ].curvature;
  //           cost_map.points.push_back(point);

  //           // std::cout << "    "  <<   cloud_filtered_sor->points[ pointIdxRadiusSearch[i] ].normal_x 
  //           //           << " " << cloud_filtered_sor->points[ pointIdxRadiusSearch[i] ].normal_y 
  //           //           << " " << cloud_filtered_sor->points[ pointIdxRadiusSearch[i] ].normal_z 
  //           //           << " " << cloud_filtered_sor->points[ pointIdxRadiusSearch[i] ].curvature 
  //           //           << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
  //         }

  //       }

  //     }

  //   }

  // }

  // didnt work

  // std::vector < pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > data;
  // data = cloud->points;



  // for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
  // {
  //   don_cloud2->points[i].x = cloud->points[i].x;
  //   don_cloud2->points[i].y = cloud->points[i].y;
  //   don_cloud2->points[i].z = cloud->points[i].z;
  // }

  //////////////////////////////////
  // PUBLISH FILTERED POINT CLOUD //
  //////////////////////////////////

  // Estimate the feature
  PointCloudOut output;
  output = *don_cloud2;

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

