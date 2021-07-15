/*
 * @Author: Aiden
 * @Date: 2021-07-07 20:02:59
 * @LastEditTime: 2021-07-14 14:36:47
 */

#include <iostream>
#include "../include/pcl_cluster_extraction.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{

   std::shared_ptr<Pcl_utility> pcl_utility = std::make_shared<Pcl_utility>();

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

   if (argc < 4)
   {
      std::cerr << "Usage: ./test_get_standard_normal path_to_cloud path_to_config index_box_front index_ground" << std::endl;
      return (-1);
   }

   if (!pcl_utility->loadCloudFile(argv[1], cloud))
   {
      std::cerr << "failed load cloud file!!";
      return (-1);
   }
   if (!pcl_utility->loadConfigFile(argv[2]))
   {
      std::cerr << "failed load yaml config file!!";
      return (-1);
   }

   // indice of segementation
   int index_box_front = atoi(static_cast<std::string>(argv[3]).c_str());
   int index_ground = atoi(static_cast<std::string>(argv[4]).c_str());
   std::cout << "[set the cluster segmentation indices] the index_box_front :" << index_box_front << ", index_ground:" << index_ground << std::endl;

   // *********Start Run*********//
   std::shared_ptr<Pcl_cluster_method> pcl_cluster = std::make_shared<Pcl_cluster_method>();
   // preprocessing
   // pcl_cluster->PreprocessCloud(cloud, Pcl_utility::downsample_method, NONE_PROCESS, Pcl_utility::leaf_size, Pcl_utility::filter_meanK, Pcl_utility::field_name, Pcl_utility::min_limit, Pcl_utility::max_limit);
   pcl_cluster->PreprocessCloud(cloud, Pcl_utility::downsample_method, USED_ALL_METHOD, Pcl_utility::leaf_size, Pcl_utility::filter_meanK, Pcl_utility::field_name, Pcl_utility::min_limit, Pcl_utility::max_limit);

   // Clustering segmentation
   std::vector<pcl::PointIndices> clusters;
   pcl_cluster->ClusterSegementation(cloud, clusters);

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_front(new pcl::PointCloud<pcl::PointXYZ>),
       cloud_ground_plane(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::ModelCoefficients::Ptr ground_coefficients_ptr(new pcl::ModelCoefficients);

   // *************Step1:computed normal of ground************** //
   pcl::copyPointCloud(*cloud, clusters[index_ground], *cloud_ground_plane);
   ground_coefficients_ptr = pcl_cluster->GetGroundPlaneCoeff(cloud_ground_plane, clusters);

   // ***********Step2: computed normal of  box front ******//
   pcl::copyPointCloud(*cloud, clusters[index_box_front], *cloud_box_front);
   pcl_cluster->FilterCloud(cloud_box_front, STATISICAL_OUTLIER_REMOVAL, 50);

   Eigen::VectorXf standard_ground_coefficients(4);
   standard_ground_coefficients << ground_coefficients_ptr->values[0], ground_coefficients_ptr->values[1], ground_coefficients_ptr->values[2], ground_coefficients_ptr->values[3];

#if 1

   Eigen::VectorXf box_bottom_coefficients;
   box_bottom_coefficients = pcl_cluster->GetBoxBottomEdgePlane(cloud_box_front, standard_ground_coefficients, NO_CHECK);
   std::cerr << "Box bottom side plane coefficients: " << box_bottom_coefficients[0] << " "
             << box_bottom_coefficients[1] << " "
             << box_bottom_coefficients[2] << " "
             << box_bottom_coefficients[3] << std::endl;
#else
   // eigen_alloc_vector<Eigen::VectorXf> box_bottom_coeff_candidates;
   // box_bottom_coeff_candidates = pcl_cluster->GetBoxBottomEdgePlaneCoeffs(cloud_box_front, standard_ground_coefficients, Eigen::MatrixXf::Zero(3, 1), NO_CHECK, 5, 1, false);
   // std::cout << "the candidates box bottom plane size:" << box_bottom_coeff_candidates.size() << std::endl;

   // std::vector<double> vec_expansions;
   // for (auto value : box_bottom_coeff_candidates)
   // {
   //    double expansion = pcl_cluster->FindMaxDistanceToPlane(cloud_box_front, value);
   //    vec_expansions.emplace_back(expansion);
   //    std::cout << "the times!!!! [bottom pts]:" << value.size() << "\t[expansion]:" << expansion << std::endl
   //              << "**************END*************" << std::endl;
   // }

   // std::cout << "the vec_expansion size:" << vec_expansions.size() << std::endl
   //           << "the max data:" << *std::max_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
   //           << "the min data:" << *std::min_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
   //           << "the average data:" << (float)std::accumulate(vec_expansions.begin(), vec_expansions.end(), 0) / (float)vec_expansions.size() << std::endl;
#endif

   return (0);
}