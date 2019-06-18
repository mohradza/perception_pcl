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

void 
pcl_ros::DiffNormals::emptyPublish (const PointCloudInConstPtr &cloud)
{
  PointCloudOut output;
  output.header = cloud->header;
  pub_output_.publish (output.makeShared ());
}

void 
pcl_ros::DiffNormals::computePublish (const PointCloudInConstPtr &cloud,
                                           const PointCloudInConstPtr &surface,
                                           const IndicesPtr &indices)
{
  // // Set the parameters
  // impl_.setKSearch (k_);
  // impl_.setRadiusSearch (search_radius_);

  // // Set the inputs
  // impl_.setInputCloud (cloud);
  // impl_.setIndices (indices);
  // impl_.setSearchSurface (surface);
  // // Estimate the feature
  // PointCloudOut output;
  // impl_.compute (output);

  // // Publish a Boost shared ptr const data
  // // Enforce that the TF frame and the timestamp are copied
  // output.header = cloud->header;
  // pub_output_.publish (output.makeShared ());

  // // Load cloud in blob format
  // pcl::PCLPointCloud2 blob;
  // pcl::io::loadPCDFile (infile.c_str (), blob);
  // pcl::PointCloud<PointXYZRGB>::Ptr cloud (new pcl::PointCloud<PointXYZRGB>);
  // pcl::fromPCLPointCloud2 (blob, *cloud);

  // Create a search tree, use KDTreee for non-organized data.
  
  if (cloud->isOrganized ())
  {
    tree.reset (new pcl::search::OrganizedNeighbor<pcl::PointXYZ> ());
  }
  else
  {
    tree.reset (new pcl::search::KdTree<pcl::PointXYZ> (false));
  }

  // Set the input pointcloud for the search tree
  tree->setInputCloud (cloud);

  if (scale1_ >= scale2_)
  {
    std::cerr << "Error: Large scale must be > small scale!" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute normals using both small and large scales at each point
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
  ne.setInputCloud (cloud);
  ne.setSearchMethod (tree);

  /**
   * NOTE: setting viewpoint is very important, so that we can ensure
   * normals are all pointed in the same direction!
   */
  ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

  // calculate normals with the small scale
  std::cout << "Calculating normals for scale..." << scale1_ << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

  ne.setRadiusSearch (scale1_);
  ne.compute (*normals_small_scale);

  // calculate normals with the large scale
  std::cout << "Calculating normals for scale..." << scale2_ << std::endl;
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

  ne.setRadiusSearch (scale2_);
  ne.compute (*normals_large_scale);

  // Create output cloud for DoN results
  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

  std::cout << "Calculating DoN... " << std::endl;
  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
  don.setInputCloud (cloud);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (normals_small_scale);

  if (!don.initCompute ())
  {
    std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
    exit (EXIT_FAILURE);
  }

  // Compute DoN
  don.computeFeature (*doncloud);

  // Save DoN features
  pcl::PCDWriter writer;
  writer.write<pcl::PointNormal> ("don.pcd", *doncloud, false); 

  // Filter by magnitude
  std::cout << "Filtering out DoN mag <= " << threshold_ << "..." << std::endl;

  // Build the condition for filtering
  pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
    new pcl::ConditionOr<pcl::PointNormal> ()
    );
  range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
                               new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold_))
                             );
  // Build the filter
  pcl::ConditionalRemoval<pcl::PointNormal> condrem;
  condrem.setCondition (range_cond);
  condrem.setInputCloud (doncloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

  // Apply filter
  condrem.filter (*doncloud_filtered);

  doncloud = doncloud_filtered;

  pcl::PointCloud<pcl::PointXYZ>::Ptr doncloud_filtered_xyz (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*doncloud_filtered, *doncloud_filtered_xyz);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud<pcl::PointXYZ, pcl::PointXYZ>(*cloud, *cloud2);

  // Save filtered output
  std::cout << "Filtered Pointcloud: " << doncloud->points.size () << " data points." << std::endl;

  // writer.write<pcl::PointNormal> ("don_filtered.pcd", *doncloud, false); 

  // // Estimate the feature
  PointCloudOut output;
  output = *doncloud_filtered_xyz;
  // impl_.compute (output);

  // Publish a Boost shared ptr const data
  // Enforce that the TF frame and the timestamp are copied
  output.header = cloud->header;


  // https://stackoverflow.com/questions/41960911/how-can-i-get-the-pointindices-of-a-given-pointcloud

  // https://stackoverflow.com/questions/44921987/removing-points-from-a-pclpointcloudpclpointxyzrgb

  // http://www.pcl-users.org/Subtracting-clouds-td3569049.html

  // http://www.pcl-users.org/Using-kdtree-radiusSearch-to-Remove-Points-td4025266.html#a4025281


  //kdTree 
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree; 
  kdtree.setInputCloud (doncloud_filtered_xyz); 
  kdtree.setSortedResults(true); 

  for (int i = 0; i < cloud->size(); i++) 
  { 

    //searchPoint 
    pcl::PointXYZ searchPoint = cloud->at(i) ; 

    //result from radiusSearch() 
    std::vector<int> pointIdxRadiusSearch; 
    std::vector<float> pointRadiusSquaredDistance; 

    if ( kdtree.radiusSearch (searchPoint, search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ) 
    { 
      //delete every point in target 
      for (size_t j = 0; j < pointIdxRadiusSearch.size (); ++j) 
      { 
              //is this the way to erase correctly??? 
              cloud2->erase(cloud2->points.begin() + pointIdxRadiusSearch[j]); 
              
      } 
    }
  } 

pub_output_.publish (cloud2);

}

typedef pcl_ros::DiffNormals DiffNormals;
PLUGINLIB_EXPORT_CLASS(DiffNormals, nodelet::Nodelet)

