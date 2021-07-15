/*
 * @Author: Aiden
 * @Date: 2021-07-05 14:25:38
 * @LastEditTime: 2021-07-14 15:15:29
 */
#ifndef PCL_UTILITY_H
#define PCL_UTILITY_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <vtkPlaneSource.h> // graph plane

#include <thread>
#include <chrono>
#include <string>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <boost/format.hpp> // for formating strings

#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;

// #define USED_DISTANCE
#define USED_GROUND_PLANE

template <typename T>
using eigen_alloc_vector = std::vector<T, Eigen::aligned_allocator<T>>;

typedef enum DOWNSAMPLE_EMTHOD_LIST
{
   VOXELGRID = 0,
   APPROXIMATE_VOXELGRID = 1
} DOWNSAMPLE_EMTHOD_LIST;
typedef int32_t DOWNSAMPLE_EMTHOD;

typedef enum FILTER_METHOD_LIST
{
   NONE_PROCESS = 0,
   STATISICAL_OUTLIER_REMOVAL = 1,
   CONDITIONAL_REMOVAL = 2,
   USED_ALL_METHOD = 3

} FILTER_METHOD_LIST;
typedef int32_t FILTER_METHOD;

typedef enum ESTIMATE_NORMAL_METHOD_LIST
{
   KDTREE_MEAN = 0,
   KDTREE_RADIUS = 1
} ESTIMATE_NORMAL_METHOD_LIST;
typedef int32_t ESTIMATE_NORMAL_METHOD;

typedef enum CHECK_DIRECTION_LIST
{
   NO_CHECK = 0,
   CHECK_POSITIVE = 1

} CHECK_DIRECTION_LIST;
typedef int32_t CHECK_DIRECTION;
// Sort in ascending order, return the serial number of the original array
// reference: https://www.itranslater.com/qa/details/2120991385611404288
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{

   // initialize original index locations
   std::vector<size_t> idx(v.size());
   std::iota(idx.begin(), idx.end(), 0);

   // sort indexes based on comparing values in v
   std::sort(idx.begin(), idx.end(),
             [&v](size_t i1, size_t i2)
             { return v[i1] < v[i2]; });

   return idx;
}

class Pcl_utility
{
public:
   Pcl_utility();
   ~Pcl_utility();

   bool loadCloudFile(const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

   bool loadCloudRGB(const std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr);

   static bool loadConfigFile(const std::string filename);

   void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

   void rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<pcl::PointIndices> clusters);

   void visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                      const pcl::ModelCoefficients::Ptr coefficients,
                      const pcl::PointIndices::Ptr inliers);

   void visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::VectorXf &coeff,
                      const std::vector<int> inliers);

   //x, y, z indicate where to draw the plane, scale (x, y) is a targeted scaling factor for x, y of the plane
   vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients &coefficients, double x, double y, double z, float scale[2] = nullptr);

   double findMaxDistanceToPlane(Eigen::VectorXf &coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &select_cloud_ptr, std::vector<uint32_t> searchCloudIndex);

   //#######################Load Config Params##########################//
   static std::string sensor_name;
   static DOWNSAMPLE_EMTHOD downsample_method;
   static float leaf_size;

   static FILTER_METHOD filter_method;
   static int filter_meanK;
   static std::string field_name;
   static float min_limit;
   static float max_limit;

   static std::vector<float> standard_box_front_normal;
   static std::vector<float> standard_ground_plane_normal;
   static std::vector<float> standard_ground_plane_coeff;

   static int clusters_maxsize_segements;
   static int clusters_min_points_size;
   static int clusters_max_points_size;
   static int clusters_num_neighbours;
   static float clusters_normal_error_threshold;
   static float clusters_curvature_threshold;

   static int bottom_expect_points;
   static float bottom_fitting_angle_error;
   static float bottom_fitting_dist_error;

   static float ground_fitting_dist_error;

   static int standard_point_cloud_size;

   //////

   static int32_t depth_w;
   static int32_t depth_h;
   static std::vector<float> depth_intrinsic;
   static std::vector<float> depth_extrinsic;
   static std::vector<float> depth_distortion;

   static int32_t color_w;
   static int32_t color_h;
   static std::vector<float> color_intrinsic;
   static std::vector<float> color_extrinsic;
   static std::vector<float> color_distortion;

   static double max_contou_area;

   static bool debug_show;
};
#endif //PCL_UTILITY_H
