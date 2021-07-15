/*
 * @Author: Aiden
 * @Date: 2021-07-05 17:43:37
 * @LastEditTime: 2021-07-14 14:32:17
 */
#include <iostream>
#include "pcl_cluster_extraction.h"
#include "cv_image_extraction.h"

int main(int argc, char *argv[])
{
   std::shared_ptr<Pcl_utility> pcl_utility = std::make_shared<Pcl_utility>();
   std::shared_ptr<Image_extract> image_extract = std::make_shared<Image_extract>();
   std::shared_ptr<Pcl_cluster_method> pcl_cluster = std::make_shared<Pcl_cluster_method>();

   if (argc < 4)
   {
      std::cerr << "Usage: ./test_rgbd_detect path_to_cloud path_to_config index_box_front index_ground" << std::endl;
      return (-1);
   }

   std::string color_file_name = argv[1];
   std::string depth_file_name = argv[2];
   std::string cloud_file_name = argv[3];

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
   if (!pcl_utility->loadCloudRGB(cloud_file_name, point_cloud_ptr))
      return (-1);
   cv::Mat color = cv::imread(color_file_name, 1);
   // cv::Mat depth = cv::imread(depth_file_name, CV_LOAD_IMAGE_UNCHANGED);
   cv::Mat depth = cv::imread(depth_file_name, cv::IMREAD_UNCHANGED);

   // load config files
   if (!pcl_utility->loadConfigFile(argv[4]))
      return (-1);

   // *********Load Config Params *********//
   Eigen::Vector3f standard_box_front_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_box_front_normal.data());       // up
   Eigen::Vector3f standard_ground_plane_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_ground_plane_normal.data()); // front
   std::vector<float> standard_ground_plane_coeff = Pcl_utility::standard_ground_plane_coeff;

   TY_CAMERA_CALIB_INFO m_depth_calib = image_extract->SetParameters(Pcl_utility::depth_w, Pcl_utility::depth_h, Pcl_utility::depth_intrinsic, Pcl_utility::depth_extrinsic, Pcl_utility::depth_distortion);
   TY_CAMERA_CALIB_INFO m_color_calib = image_extract->SetParameters(Pcl_utility::color_w, Pcl_utility::color_h, Pcl_utility::color_intrinsic, Pcl_utility::color_extrinsic, Pcl_utility::color_distortion);

   int max_index;
   std::vector<std::vector<cv::Point>> contours;
   contours = image_extract->GetImageRoiContour(color);
   if (!image_extract->FindTargetContour(contours, max_index, Pcl_utility::max_contou_area))
   {
      std::cerr << "Faild find the target object in the image!!!" << std::endl
                << "please Check whether the image cannot be accurately extracted to the closed contour ~" << std::endl;
      return (-1);
   }

   // Get all the pixel indexs in the area
   std::vector<uint32_t> all_in_area_image_indexs;
   all_in_area_image_indexs = image_extract->GetAllImageIndexWithinArea(contours[max_index], Pcl_utility::color_w, Pcl_utility::color_h);
   std::cout << "first the area size:" << all_in_area_image_indexs.size() << std::endl;

   cv::Mat color_data_mat;
   std::vector<uint32_t> all_out_cloud_indexs;
   image_extract->MapImageIndexToCloud(m_depth_calib, m_color_calib, depth, color, color_data_mat, all_in_area_image_indexs, all_out_cloud_indexs);

   std::vector<int> contour_area_indexs(all_out_cloud_indexs.size());
   for (int i = 0; i < all_out_cloud_indexs.size(); i++)
      contour_area_indexs[i] = all_out_cloud_indexs[i];
   auto contourAreaCloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
   pcl::copyPointCloud(*point_cloud_ptr, contour_area_indexs, *contourAreaCloud);

   pcl_cluster->FilterCloud(contourAreaCloud, STATISICAL_OUTLIER_REMOVAL, 50);
   pcl_utility->simpleVis(contourAreaCloud);

   Eigen::VectorXf standard_ground_coefficients(4);
   standard_ground_coefficients << standard_ground_plane_coeff[0], standard_ground_plane_coeff[1], standard_ground_plane_coeff[2], standard_ground_plane_coeff[3];

#if defined(USED_DISTANCE)

   Eigen::VectorXf box_bottom_coefficients;
   box_bottom_coefficients = pcl_cluster->GetBoxBottomEdgePlane(contourAreaCloud, standard_ground_coefficients, CHECK_POSITIVE, standard_box_front_normal);
   double expansion = pcl_cluster->FindMaxDistanceToPlane(contourAreaCloud, box_bottom_coefficients);

   std::cerr << "Box bottom side plane coefficients: " << box_bottom_coefficients[0] << " "
             << box_bottom_coefficients[1] << " "
             << box_bottom_coefficients[2] << " "
             << box_bottom_coefficients[3] << std::endl;

   std::cout << "the expansion dist:" << expansion << std::endl;
#elif defined(USED_GROUND_PLANE)
   eigen_alloc_vector<Eigen::VectorXf> box_bottom_coeff_candidates;
   box_bottom_coeff_candidates = pcl_cluster->GetBoxBottomEdgePlaneCoeffs(contourAreaCloud, standard_ground_coefficients, standard_box_front_normal, CHECK_POSITIVE, 5, 1, false);
   std::cout << "the candidates box bottom plane size:" << box_bottom_coeff_candidates.size() << std::endl;

   std::vector<double> vec_expansions;
   for (auto value : box_bottom_coeff_candidates)
   {
      double expansion = pcl_cluster->FindMaxDistanceToPlane(contourAreaCloud, value);
      vec_expansions.emplace_back(expansion);
      std::cout << "the times!!!! [bottom pts]:" << value.size() << "\t[expansion]:" << expansion << std::endl
                << "**************END*************" << std::endl;
   }

   std::cout << "the vec_expansion size:" << vec_expansions.size() << std::endl
             << "the max data:" << *std::max_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
             << "the min data:" << *std::min_element(vec_expansions.begin(), vec_expansions.end()) << std::endl
             << "the average data:" << (float)std::accumulate(vec_expansions.begin(), vec_expansions.end(), 0) / (float)vec_expansions.size() << std::endl;
#endif
   return 0;
}