/*
 * @Author: Aiden
 * @Date: 2021-07-05 13:53:25
 * @LastEditTime: 2021-07-14 14:42:56
 */
#include <iostream>
#include "../include/pcl_cluster_extraction.h"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{

   std::shared_ptr<Pcl_utility> pcl_utility = std::make_shared<Pcl_utility>();

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

   // *********Load Config Params *********//
   Eigen::Vector3f standard_box_front_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_box_front_normal.data());       // up
   Eigen::Vector3f standard_ground_plane_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_ground_plane_normal.data()); // front
   std::vector<float> standard_ground_plane_coeff = Pcl_utility::standard_ground_plane_coeff;                                    // front

   // *********Start Run*********//

   std::shared_ptr<Pcl_cluster_method> pcl_cluster = std::make_shared<Pcl_cluster_method>();
   // preprocessing
   pcl_cluster->PreprocessCloud(cloud, Pcl_utility::downsample_method, Pcl_utility::filter_method, Pcl_utility::leaf_size, Pcl_utility::filter_meanK, Pcl_utility::field_name, Pcl_utility::min_limit, Pcl_utility::max_limit);

   // Clustering segmentation
   std::vector<pcl::PointIndices> clusters;
   pcl_cluster->ClusterSegementation(cloud, clusters);

   // indice of segementation
   int index_box_front = 0;

   // *********Judge whether cluster is right box front *********//
   index_box_front = pcl_cluster->FindBoxFrontClusterIndice(cloud, clusters, standard_box_front_normal);
   std::cout << "success find the box front cluster,the index is: " << index_box_front << std::endl;

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_front(new pcl::PointCloud<pcl::PointXYZ>),
       cloud_ground_plane(new pcl::PointCloud<pcl::PointXYZ>);

   // ********* computed box front expansion ******//
   pcl::copyPointCloud(*cloud, clusters[index_box_front], *cloud_box_front);
   pcl_cluster->FilterCloud(cloud_box_front, STATISICAL_OUTLIER_REMOVAL, 50);

   Eigen::VectorXf standard_ground_coefficients(4);
   standard_ground_coefficients << standard_ground_plane_coeff[0], standard_ground_plane_coeff[1], standard_ground_plane_coeff[2], standard_ground_plane_coeff[3];

#if defined(USED_DISTANCE)

   Eigen::VectorXf box_bottom_coefficients;
   box_bottom_coefficients = pcl_cluster->GetBoxBottomEdgePlane(cloud_box_front, standard_ground_coefficients, CHECK_POSITIVE, standard_box_front_normal);
   double expansion = pcl_cluster->FindMaxDistanceToPlane(cloud_box_front, box_bottom_coefficients);

   std::cerr << "Box bottom side plane coefficients: " << box_bottom_coefficients[0] << " "
             << box_bottom_coefficients[1] << " "
             << box_bottom_coefficients[2] << " "
             << box_bottom_coefficients[3] << std::endl;
   std::cout << "the expansion dist:" << expansion << std::endl;
#elif defined(USED_GROUND_PLANE)
   eigen_alloc_vector<Eigen::VectorXf> box_bottom_coeff_candidates;
   box_bottom_coeff_candidates = pcl_cluster->GetBoxBottomEdgePlaneCoeffs(cloud_box_front, standard_ground_coefficients, standard_box_front_normal, CHECK_POSITIVE, 10, 1, false);
   std::cout << "the candidates box bottom plane size:" << box_bottom_coeff_candidates.size() << std::endl;

   std::vector<double> vec_expansions;
   for (auto value : box_bottom_coeff_candidates)
   {
      double expansion = pcl_cluster->FindMaxDistanceToPlane(cloud_box_front, value);
      vec_expansions.emplace_back(expansion);
      std::cout << "the times!!!! [bottom pts]:" << value.size() << "\t[expansion]:" << expansion << std::endl
                << "**************END*************" << std::endl;
   }

   std::cout << "the vec_expansion size:" << vec_expansions.size() << std::endl
             << "the max data:" << *std::max_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
             << "the min data:" << *std::min_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
             << "the average data:" << (float)std::accumulate(vec_expansions.begin(), vec_expansions.end(), 0) / (float)vec_expansions.size() << std::endl;
#endif
   return (0);
}