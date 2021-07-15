/*
 * @Author: Aiden
 * @Date: 2021-07-12 17:32:04
 * @LastEditTime: 2021-07-14 12:32:08
 */
#include "k4a_camera.h"
#include <iostream>

#include "pcl_cluster_extraction.h"
#include "cv_image_extraction.h"

using namespace multi_camera_calibration;

// #define PRE_SAVED_RESOURCES // noted it for saving raw color image and point cloud files

int main(int argc, char *argv[])
{
   std::vector<FrameInfo> frames;
   std::shared_ptr<K4aCamera> camera_ptr = std::make_shared<K4aCamera>();

   std::shared_ptr<Pcl_utility> pcl_utility = std::make_shared<Pcl_utility>();
   std::shared_ptr<Image_extract> image_extract = std::make_shared<Image_extract>();
   std::shared_ptr<Pcl_cluster_method> pcl_cluster = std::make_shared<Pcl_cluster_method>();

   size_t file_count = 1; // the nums of mkv files
   char *files[file_count];
   files[file_count - 1] = argv[1];

   camera_ptr->SetInputFiles(file_count, files);
   camera_ptr->GetFrames(frames);

   auto obj_frame = frames.front();
   int COLOR_W = obj_frame.ColorWidth;
   int COLOR_H = obj_frame.ColorHeight;
   cv::Mat depth = cv::Mat(obj_frame.DepthHeight, obj_frame.DepthWidth, CV_32FC1, (void *)obj_frame.DepthImage.data());

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   if (camera_ptr->point_cloud_depth_to_color(file_count - 1, obj_frame.k4a_depthImage, obj_frame.k4a_colorImage, color_point_cloud) == false)
   {
      return -1;
   }
#ifdef PRE_SAVED_RESOURCES
   cv::Mat raw_color = cv::Mat(COLOR_H, COLOR_W, CV_8UC4, (void *)obj_frame.ColorImage.data());
   std::cout << "the frone color size:" << obj_frame.ColorImage.size()
             << "the frone depth size:" << obj_frame.DepthImage.size() << std::endl;

   cv::imwrite("kinect_rawColor.png", raw_color);
   pcl::io::savePCDFileASCII("kinect_pointCloud.pcd", *color_point_cloud);
   return 0;
#endif // PRE_SAVED_RESOURCES

   std::string color_file_name = argv[2]; // color image which after extracting ROI
   cv::Mat color = cv::imread(color_file_name, 1);
   if (color.empty())
      return -1;

   // load config files
   if (!pcl_utility->loadConfigFile(argv[3]))
      return (-1);

   // *********Load Config Params *********//
   Eigen::Vector3f standard_box_front_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_box_front_normal.data());       // up
   Eigen::Vector3f standard_ground_plane_normal = Eigen::Map<Eigen::Vector3f>(Pcl_utility::standard_ground_plane_normal.data()); // front
   std::vector<float> standard_ground_plane_coeff = Pcl_utility::standard_ground_plane_coeff;

   // *************** Run ***************//
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
   all_in_area_image_indexs = image_extract->GetAllImageIndexWithinArea(contours[max_index], COLOR_W, COLOR_H);
   std::cout << "first the area size:" << all_in_area_image_indexs.size() << std::endl;

   std::vector<int> all_out_cloud_indexs;
   for (auto value : all_in_area_image_indexs)
   {
      if (color_point_cloud->points[value].z != 0)
         all_out_cloud_indexs.push_back(value);
   }

   // auto contourAreaCloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
   auto contourAreaCloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
   pcl::copyPointCloud(*color_point_cloud, all_out_cloud_indexs, *contourAreaCloud);
   // rgbVis(contourAreaCloud);

   // preprocessing
   pcl_cluster->PreprocessCloud(contourAreaCloud, Pcl_utility::downsample_method, Pcl_utility::filter_method, Pcl_utility::leaf_size, Pcl_utility::filter_meanK, Pcl_utility::field_name, Pcl_utility::min_limit, Pcl_utility::max_limit);

   pcl_utility->simpleVis(contourAreaCloud);

   Eigen::VectorXf standard_ground_coefficients(4);
   standard_ground_coefficients << standard_ground_plane_coeff[0], standard_ground_plane_coeff[1], standard_ground_plane_coeff[2], standard_ground_plane_coeff[3];
   Eigen::VectorXf box_bottom_coefficients;

   // pcl::io::savePCDFileASCII("area_cloud.pcd", *contourAreaCloud);

#if defined(USED_DISTANCE)
   box_bottom_coefficients = pcl_cluster->GetBoxBottomEdgePlane(contourAreaCloud, standard_ground_coefficients, CHECK_POSITIVE, standard_box_front_normal);
   double expansion = pcl_cluster->FindMaxDistanceToPlane(contourAreaCloud, box_bottom_coefficients);
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
   // std::cerr << "Box bottom side plane coefficients: " << box_bottom_coefficients[0] << " "
   //           << box_bottom_coefficients[1] << " "
   //           << box_bottom_coefficients[2] << " "
   //           << box_bottom_coefficients[3] << std::endl;

   return 0;
}