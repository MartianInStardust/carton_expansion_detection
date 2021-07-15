/*
 * @Author: Aiden
 * @Date: 2021-07-05 13:57:01
 * @LastEditTime: 2021-07-14 12:42:20
 */
#ifndef PCL_CLUSTER_EXTRACTION_H
#define PCL_CLUSTER_EXTRACTION_H

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>

#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/region_growing.h>

// #include <pcl/filters/extract_indices.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

#include <pcl/filters/voxel_grid.h>             // downsample
#include <pcl/filters/approximate_voxel_grid.h> //
#include <pcl/kdtree/kdtree_flann.h>

#include <algorithm>
#include <map>
#include <functional>

#include "pcl_utility.h"

struct CLUSTER_INFO
{
   int size;
   int indice;
};
struct CmpByKeySize
{
   bool operator()(const CLUSTER_INFO &k1, const CLUSTER_INFO &k2)
   {
      return k1.size > k2.size;
   }
};
class Pcl_cluster_method
{
public:
   Pcl_cluster_method();
   ~Pcl_cluster_method();

   //! pcl::NormalEstimationOMP will faster
   void EstimateNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals, ESTIMATE_NORMAL_METHOD normal_method);

   void FittingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     pcl::ModelCoefficients::Ptr &coefficients,
                     pcl::PointIndices::Ptr &inliers);

   void FittingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                     const pcl::PointCloud<pcl::Normal>::Ptr normals,
                     pcl::ModelCoefficients::Ptr &coefficients,
                     pcl::PointIndices::Ptr &inliers);

   void FilterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, FILTER_METHOD method, int mean_k = 50, std::string field_name = "z", float min_limit = 0.0, float max_limit = 1.0);

   //# approximate voxelgrid method may not better than voxel gird,but it run faster
   void DownSampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float size, DOWNSAMPLE_EMTHOD method);

   void PreprocessCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, DOWNSAMPLE_EMTHOD downsample_method, FILTER_METHOD filter_method, float leaf_size, int mean_k = 50, std::string field_name = "z", float min_limit = 0.0, float max_limit = 1.0);

   //# Find the bottom edge composed of expect_bottom_pt_nums in the point set of the target point cluster with the smallest distance from the ground
   std::vector<int> FindBottomEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf coefficients);

   eigen_alloc_vector<Eigen::VectorXf> GetBoxBottomEdgePlaneCoeffs(pcl::PointCloud<pcl::PointXYZ>::Ptr contourAreaCloud, const Eigen::VectorXf standard_ground_coeffs, const Eigen::Vector3f &standard_box_front_normal, CHECK_DIRECTION if_check, const double &max_shift, const double &threshold, const bool &pos_or_neg);

   // According to the points of edge and known ground normal vector, Calculate the plane equation of the bottom side
   Eigen::VectorXf FittingBottomEdgePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f ground_normal);

   double FindMaxDistanceToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::VectorXf &coeff);

   /**
 * @brief region growing algorithm.each cluster is a set of points that are considered to be a part of the same smooth surface. 
 * The work of this algorithm is based on the comparison of the angles between the points normal
 * @param in_cloud input object cloud 
 * @param normals input, object cloud normals
 * @param out_colored_cloud output, all extracted points clusters with color rendering
 * @return clusters indexs vector   
 * @ref https://pcl.readthedocs.io/projects/tutorials/en/latest/region_growing_segmentation.html#region-growing-segmentation
*/
   std::vector<pcl::PointIndices> GetRegionGrowingCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_colored_cloud);

   void ClusterSegementation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointIndices> &clusters);

   pcl::ModelCoefficients::Ptr GetGroundPlaneCoeff(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ground, std::vector<pcl::PointIndices> &clusters);

   // According to the standard normal vector, it is converted into a positive direction
   // return the minimum included angle
   bool SetPositiveDirectNormal(Eigen::VectorXf &input_coeff, Eigen::Vector3f standard_normal, double &included_angle);

   // step1: choose big size cloud
   // step2: fitting cluster plane
   // step3: calculate each cluster normal,judge whether the direction of the vector is correct
   int FindBoxFrontClusterIndice(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointIndices> clusters, Eigen::Vector3f standard_box_front_normal);

   Eigen::VectorXf GetBoxBottomEdgePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_box_front, Eigen::VectorXf &ground_coefficients, CHECK_DIRECTION if_check, const Eigen::Vector3f &standard_box_front_normal = Eigen::MatrixXf::Zero(3, 1));

private:
   std::shared_ptr<Pcl_utility> sub_pcl_utility;
};
#endif //PCL_CLUSTER_EXTRACTION_H
