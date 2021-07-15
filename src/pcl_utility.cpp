/*
 * @Author: Aiden
 * @Date: 2021-07-05 14:26:09
 * @LastEditTime: 2021-07-14 15:16:03
 */
#include "../include/pcl_utility.h"

std::string Pcl_utility::sensor_name;
DOWNSAMPLE_EMTHOD Pcl_utility::downsample_method;
float Pcl_utility::leaf_size;

FILTER_METHOD Pcl_utility::filter_method;
int Pcl_utility::filter_meanK;
std::string Pcl_utility::field_name;
float Pcl_utility::min_limit;
float Pcl_utility::max_limit;

std::vector<float> Pcl_utility::standard_box_front_normal(3);
std::vector<float> Pcl_utility::standard_ground_plane_normal(3);
std::vector<float> Pcl_utility::standard_ground_plane_coeff(4);

int Pcl_utility::clusters_maxsize_segements;
int Pcl_utility::clusters_min_points_size;
int Pcl_utility::clusters_max_points_size;
int Pcl_utility::clusters_num_neighbours;
float Pcl_utility::clusters_normal_error_threshold;
float Pcl_utility::clusters_curvature_threshold;

int Pcl_utility::bottom_expect_points;
float Pcl_utility::bottom_fitting_angle_error;
float Pcl_utility::bottom_fitting_dist_error;

float Pcl_utility::ground_fitting_dist_error;

int Pcl_utility::standard_point_cloud_size;

////////////////////

int32_t Pcl_utility::depth_w;
int32_t Pcl_utility::depth_h;
std::vector<float> Pcl_utility::depth_intrinsic(9);
std::vector<float> Pcl_utility::depth_extrinsic(16);
std::vector<float> Pcl_utility::depth_distortion(12);

int32_t Pcl_utility::color_w;
int32_t Pcl_utility::color_h;
std::vector<float> Pcl_utility::color_intrinsic(9);
std::vector<float> Pcl_utility::color_extrinsic(16);
std::vector<float> Pcl_utility::color_distortion(12);

double Pcl_utility::max_contou_area;

bool Pcl_utility::debug_show;

Pcl_utility::Pcl_utility()
{
}
Pcl_utility::~Pcl_utility()
{
}

bool Pcl_utility::loadCloudFile(const std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
   if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) //* load the file
   {
      PCL_ERROR("Couldn't read file PCD format file \n");
      if (pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud) == -1)
      {
         PCL_ERROR("Couldn't read file PLY format file \n");
         if (loadCloudRGB(filename, point_cloud_ptr))
         {
            pcl::copyPointCloud(*point_cloud_ptr, *cloud);
            PCL_WARN("SUCCESS LOAD XYZ format cloud file \n");
         }
         else
         {
            PCL_ERROR("Couldn't read file XYZ format file \n");
            return false;
         }
      }
   }

   return true;
}
bool Pcl_utility::loadCloudRGB(const std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr)
{
   ifstream fs;
   fs.open(filename.c_str(), ios::binary);
   if (!fs.is_open() || fs.fail())
   {
      PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
      fs.close();
      return (false);
   }

   std::string line;
   std::vector<std::string> st;

   while (!fs.eof())
   {
      getline(fs, line);
      // Ignore empty lines
      if (line.empty())
         continue;

      // Tokenize the line
      boost::trim(line);
      boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

      // if (st.size() != 7)
      if (st.size() != 6)
         continue;

      // beacuse the point flippling around xy-axes
      pcl::PointXYZRGB point;
      point.x = float(atof(st[0].c_str()));
      point.y = float(atof(st[1].c_str())); // maybe minus
      point.z = float(atof(st[2].c_str())); // maybe minus
      point.r = int(atoi(st[3].c_str()));
      point.g = int(atoi(st[3].c_str()));
      point.b = int(atoi(st[3].c_str()));

      point_cloud_ptr->points.push_back(point);
   }
   fs.close();

   point_cloud_ptr->width = point_cloud_ptr->size();
   point_cloud_ptr->height = 1;
   std::cout << "the load cloud size :" << point_cloud_ptr->size() << std::endl;
   return (true);
}
bool Pcl_utility::loadConfigFile(const std::string filename)
{
   YAML::Node config = YAML::LoadFile(filename);
   sensor_name = config["sensor.name"].as<std::string>();
   std::cout << "name:" << sensor_name << std::endl;

   auto temp_downsample = config["downsample.method"].as<std::string>();
   if (strcmp(temp_downsample.c_str(), "voxelgrid") == 0)
      downsample_method = VOXELGRID;
   else
      downsample_method = APPROXIMATE_VOXELGRID;
   leaf_size = config["downsample.leaf_size"].as<float>();

   auto temp_filter = config["filter.method"].as<std::string>();
   if (strcmp(temp_filter.c_str(), "none_process") == 0)
      filter_method = NONE_PROCESS;
   else if (strcmp(temp_filter.c_str(), "statistical_outlier_removal") == 0)
      filter_method = STATISICAL_OUTLIER_REMOVAL;
   else if (strcmp(temp_filter.c_str(), "conditional_removal") == 0)
      filter_method = CONDITIONAL_REMOVAL;
   else if (strcmp(temp_filter.c_str(), "used_all_method") == 0)
      filter_method = USED_ALL_METHOD;

   filter_meanK = config["filter.mean_k"].as<int>();
   field_name = config["filter.field_name"].as<std::string>();
   min_limit = config["filter.min_limit"].as<float>();
   max_limit = config["filter.max_limit"].as<float>();

   standard_box_front_normal = config["standard.front_normal"].as<std::vector<float>>();
   standard_ground_plane_normal = config["standard.ground_normal"].as<std::vector<float>>();
   standard_ground_plane_coeff = config["standard.ground_coeff"].as<std::vector<float>>();

   standard_point_cloud_size = config["standard.point_cloud_size"].as<int>();

   clusters_maxsize_segements = config["clusters.maxsize_segements"].as<int>();
   clusters_min_points_size = config["clusters.min_points_size"].as<int>();
   clusters_max_points_size = config["clusters.max_points_size"].as<int>();
   clusters_num_neighbours = config["clusters.num_neighbours"].as<int>();
   clusters_normal_error_threshold = config["clusters.normal_error_threshold"].as<float>();
   clusters_curvature_threshold = config["clusters.curvature_threshold"].as<float>();

   ground_fitting_dist_error = config["bottom.expect_points"].as<float>();

   bottom_expect_points = config["bottom.expect_points"].as<int>();
   bottom_fitting_angle_error = config["bottom.fitting_angle_error"].as<float>();
   bottom_fitting_dist_error = config["bottom.fitting_dist_error"].as<float>();

   if (!config["debug.visualization"].as<int>())
      debug_show = false;
   else
      debug_show = true;

   if (strcmp(sensor_name.c_str(), "percipioRGBD") == 0)
   {
      depth_w = config["depth.width"].as<int32_t>();
      depth_h = config["depth.height"].as<int32_t>();
      depth_intrinsic = config["depth.intrinsic"].as<std::vector<float>>();
      depth_extrinsic = config["depth.extrinsic"].as<std::vector<float>>();
      depth_distortion = config["depth.distortion"].as<std::vector<float>>();

      color_w = config["color.width"].as<int32_t>();
      color_h = config["color.height"].as<int32_t>();
      color_intrinsic = config["color.intrinsic"].as<std::vector<float>>();
      color_extrinsic = config["color.extrinsic"].as<std::vector<float>>();
      color_distortion = config["color.distortion"].as<std::vector<float>>();

      max_contou_area = config["contour.max_area"].as<double>();
   }
   if (strcmp(sensor_name.c_str(), "Azure_Kinect") == 0)
   {
      max_contou_area = config["contour.max_area"].as<double>();
      //TODO
   }

   std::cerr << "load config params:" << std::endl;
   std::cerr << "\tsensor_name: " << Pcl_utility::sensor_name << std::endl
             << "\tfilter method:" << temp_filter << std::endl
             << "\tleaf_size: " << Pcl_utility::leaf_size << std::endl
             << "\tstandard_ground_plane_coeff: " << standard_ground_plane_coeff.front() << std::endl;

   return true;
}
void Pcl_utility::simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
   if (!debug_show)
      return;

   pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer->setBackgroundColor(0, 0, 0);
   viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
   //viewer->addCoordinateSystem (1.0, "global");
   viewer->initCameraParameters();
   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(100ms);
   }
}

void Pcl_utility::rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<pcl::PointIndices> clusters)
{
   if (!debug_show)
      return;
   // --------------------------------------------
   // -----Open 3D viewer and add point cloud-----
   // --------------------------------------------
   pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
   viewer->setBackgroundColor(0, 0, 0);
   pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
   viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

   auto segment_cloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
   char str[512];
   for (unsigned int i = 0; i < clusters.size(); i++)
   {
      pcl::copyPointCloud(*cloud, clusters[i], *segment_cloud);
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*segment_cloud, centroid);
      sprintf(str, "Index#%03d", i);
      viewer->addText3D(str, pcl::PointXYZ(centroid[0], centroid[1], centroid[2]), 10);
   }

   viewer->addCoordinateSystem(1.0);
   viewer->initCameraParameters();
   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      std::this_thread::sleep_for(100ms);
   }
}

void Pcl_utility::visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                const pcl::ModelCoefficients::Ptr coefficients,
                                const pcl::PointIndices::Ptr inliers)
{
   if (!debug_show)
      return;
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Plane"));
   viewer->setBackgroundColor(0, 0, 0);

   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*cloud, centroid);
   // std::cout << "centroid2: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ", " << centroid[3] << std::endl;
   int scale = 100;
   pcl::PointXYZ p1(centroid[0], centroid[1], centroid[2]);
   pcl::PointXYZ p2(centroid[0] + scale * coefficients->values[0],
                    centroid[1] + scale * coefficients->values[1],
                    centroid[2] + scale * coefficients->values[2]);
   viewer->addArrow(p2, p1, 1, 0, 0, false);

   viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

   pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::copyPointCloud(*cloud, *inliers, *inlier_cloud);
   viewer->addPointCloud<pcl::PointXYZ>(inlier_cloud, "inlier_cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "inlier_cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inlier_cloud");

   viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, 50, 1, "normals"); // maybe need Point type is PointXYZRGB will show normal

   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
}

void Pcl_utility::visualization(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::VectorXf &coeff,
                                const std::vector<int> inliers)
{
   if (!debug_show)
      return;
   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Plane"));
   viewer->setBackgroundColor(0, 0, 0);

   viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "sample cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

   Eigen::Vector4f centroid;
   pcl::compute3DCentroid(*cloud, centroid);
   // std::cout << "centroid2: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ", " << centroid[3] << std::endl;
   int scale = 100;
   pcl::PointXYZ p1(centroid[0], centroid[1], centroid[2]);
   pcl::PointXYZ p2(centroid[0] + scale * coeff[0],
                    centroid[1] + scale * coeff[1],
                    centroid[2] + scale * coeff[2]);
   viewer->addArrow(p2, p1, 1, 0, 0, false);
   {
      pcl::ModelCoefficients::Ptr plane_1(new pcl::ModelCoefficients);
      plane_1->values.resize(4);
      plane_1->values[0] = coeff[0];
      plane_1->values[1] = coeff[1];
      plane_1->values[2] = coeff[2];
      plane_1->values[3] = coeff[3];
      float scale[2] = {300, 300};
      auto plane = createPlane(*plane_1, p1.x, p1.y, p1.z, scale);
      viewer->addModelFromPolyData(plane, "plane_1");
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.1, 0.9, "plane_1", 0);
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
   }

   pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::copyPointCloud(*cloud, inliers, *inlier_cloud);
   viewer->addPointCloud<pcl::PointXYZ>(inlier_cloud, "inlier_cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "inlier_cloud");
   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "inlier_cloud");

   viewer->addCoordinateSystem(1.0, "global");

   while (!viewer->wasStopped())
   {
      viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
}
vtkSmartPointer<vtkPolyData> Pcl_utility::createPlane(const pcl::ModelCoefficients &coefficients, double x, double y, double z, float scale[2])
{
   vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();

   double norm_sqr = 1.0 / (coefficients.values[0] * coefficients.values[0] +
                            coefficients.values[1] * coefficients.values[1] +
                            coefficients.values[2] * coefficients.values[2]);

   plane->SetNormal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
   double t = x * coefficients.values[0] + y * coefficients.values[1] + z * coefficients.values[2] + coefficients.values[3];
   x -= coefficients.values[0] * t * norm_sqr;
   y -= coefficients.values[1] * t * norm_sqr;
   z -= coefficients.values[2] * t * norm_sqr;

   plane->SetCenter(x, y, z);

   {
      double pt1[3], pt2[3], orig[3], center[3];
      plane->GetPoint1(pt1);
      plane->GetPoint2(pt2);
      plane->GetOrigin(orig);
      plane->GetCenter(center);

      float scale1 = 3.0;
      float scale2 = 3.0;
      if (scale != nullptr)
      {
         scale1 = scale[0];
         scale2 = scale[1];
      }
      double _pt1[3], _pt2[3];
      for (int i = 0; i < 3; i++)
      {
         _pt1[i] = scale1 * (pt1[i] - orig[i]);
         _pt2[i] = scale2 * (pt2[i] - orig[i]);
      }
      for (int i = 0; i < 3; ++i)
      {
         pt1[i] = orig[i] + _pt1[i];
         pt2[i] = orig[i] + _pt2[i];
      }
      plane->SetPoint1(pt1);
      plane->SetPoint2(pt2);

      //        //延长origin
      //        double _origin[3];
      //        for(int i=0; i<3;++i)
      //        {
      //            _origin[i] = scale*(orig[i]-pt1[i]);
      //        }
      //        for(int i=0; i<3;++i)
      //        {
      //            orig[i] = pt1[i] + _origin[i];
      //        }
      //        plane->SetOrigin(orig);
   }
   plane->Update();

   return (plane->GetOutput());
}
double Pcl_utility::findMaxDistanceToPlane(Eigen::VectorXf &coeff, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &select_cloud_ptr, std::vector<uint32_t> searchCloudIndex)
{

   double a, b, c, d;
   a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];

   double expansion = 0.0;
   for (auto value : searchCloudIndex)
   {
      double temp_dist = abs(a * select_cloud_ptr->at(value).x + b * select_cloud_ptr->at(value).y + c * select_cloud_ptr->at(value).z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
      if (temp_dist > expansion)
         expansion = temp_dist;
   }
   std::cout << "the expansion dist:" << expansion << std::endl;
   return expansion;
}