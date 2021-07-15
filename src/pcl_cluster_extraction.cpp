/*
 * @Author: Aiden
 * @Date: 2021-07-05 13:57:14
 * @LastEditTime: 2021-07-14 15:03:10
 */
#include "../include/pcl_cluster_extraction.h"

Pcl_cluster_method::Pcl_cluster_method()
{
   sub_pcl_utility = std::make_shared<Pcl_utility>();
}
Pcl_cluster_method::~Pcl_cluster_method()
{
}
void Pcl_cluster_method::EstimateNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                        pcl::PointCloud<pcl::Normal>::Ptr &normals, ESTIMATE_NORMAL_METHOD normal_method)
{
   pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
   pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
   normal_estimator.setInputCloud(cloud);
   normal_estimator.setSearchMethod(kdtree);

   if (normal_method == KDTREE_MEAN)
   {
      normal_estimator.setKSearch(50);
   }
   else if (normal_method == KDTREE_RADIUS)
   {
      normal_estimator.setRadiusSearch(0.8);
   }
   normal_estimator.compute(*normals);
}
void Pcl_cluster_method::FittingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      pcl::ModelCoefficients::Ptr &coefficients,
                                      pcl::PointIndices::Ptr &inliers)
{

   // Create the segmentation object
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setDistanceThreshold(Pcl_utility::ground_fitting_dist_error); //! 0.3
   seg.setMaxIterations(100);
   seg.setInputCloud(cloud);
   seg.segment(*inliers, *coefficients);
}
// overload function2 ,根据法线参考估计的拟合平面
void Pcl_cluster_method::FittingPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      const pcl::PointCloud<pcl::Normal>::Ptr normals,
                                      pcl::ModelCoefficients::Ptr &coefficients,
                                      pcl::PointIndices::Ptr &inliers)
{
   pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setNormalDistanceWeight(0.1); //设置表面法线权重系数
   seg.setInputNormals(normals);
   seg.setDistanceThreshold(Pcl_utility::ground_fitting_dist_error);
   seg.setMaxIterations(100);
   seg.setInputCloud(cloud);
   seg.segment(*inliers, *coefficients);
}
void Pcl_cluster_method::FilterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, FILTER_METHOD method, int mean_k, std::string field_name, float min_limit, float max_limit)
{
   // Create the filtering object
   if (method == CONDITIONAL_REMOVAL || method == USED_ALL_METHOD)
   {
      // build the condition
      pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZ>());
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(field_name, pcl::ComparisonOps::GT, min_limit)));
      range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(field_name, pcl::ComparisonOps::LT, max_limit)));
      // build the filter
      pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
      condrem.setCondition(range_cond);
      condrem.setInputCloud(cloud);
      condrem.setKeepOrganized(true);
      // apply filter
      condrem.filter(*cloud);
   }

   if (method == STATISICAL_OUTLIER_REMOVAL || method == USED_ALL_METHOD)
   {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(cloud);
      sor.setMeanK(mean_k);
      sor.setStddevMulThresh(1.0);
      sor.filter(*cloud);
   }
}

void Pcl_cluster_method::DownSampleCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float size, DOWNSAMPLE_EMTHOD method)
{
   std::cout << "Number of points before downsampling：" << cloud->size() << std::endl;
   if (method == VOXELGRID)
   {
      std::cout << "choose downsample method is VoxelGrid" << std::endl;
      // Create the filtering object for downsample
      pcl::VoxelGrid<pcl::PointXYZ> sor_downsample;
      sor_downsample.setInputCloud(cloud);
      sor_downsample.setLeafSize(size, size, size); // recommended 3.0f
      sor_downsample.filter(*cloud);
   }
   else if (method == APPROXIMATE_VOXELGRID)
   {
      std::cout << "choose downsample method is ApproximateVoxelGrid" << std::endl;
      pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
      approximate_voxel_filter.setLeafSize(size, size, size);
      approximate_voxel_filter.setInputCloud(cloud); // recommended 6.0f
      approximate_voxel_filter.filter(*cloud);

#if 0
      //-----------K nearest neighbor search------------
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud(cloud);
      pcl::PointIndicesPtr inds = boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices()); //采样后根据最邻近点提取的样本点下标索引
      for (size_t i = 0; i < cloud->points.size(); i++)
      {
         pcl::PointXYZ searchPoint;
         searchPoint.x = cloud->points[i].x;
         searchPoint.y = cloud->points[i].y;
         searchPoint.z = cloud->points[i].z;

         int K = 1; 
         std::vector<int> pointIdxNKNSearch(K);
         std::vector<float> pointNKNSquaredDistance(K);
         if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
         {

            inds->indices.push_back(pointIdxNKNSearch[0]);
         }
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr final_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud, inds->indices, *cloud);
#endif // improved voxel grid downsample.
   }
   std::cout << "Number of points after downsampling：" << cloud->width * cloud->height << std::endl;
}
void Pcl_cluster_method::PreprocessCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, DOWNSAMPLE_EMTHOD downsample_method, FILTER_METHOD filter_method, float leaf_size, int mean_k, std::string field_name, float min_limit, float max_limit)
{
   if (filter_method != NONE_PROCESS)
      FilterCloud(cloud, filter_method, mean_k, field_name, min_limit, max_limit);

   const auto tp_1 = std::chrono::steady_clock::now();
   DownSampleCloud(cloud, leaf_size, downsample_method);
   const auto tp_2 = std::chrono::steady_clock::now();
   const auto downsample_time = std::chrono::duration_cast<std::chrono::microseconds>(tp_2 - tp_1).count() / 1000000.0; //micros second
   std::cout << "the time consuming of downsample:" << downsample_time << " (s)" << std::endl;
}
std::vector<int> Pcl_cluster_method::FindBottomEdgePoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::VectorXf coefficients)
{
   std::vector<double> distances;

   pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
       model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud)); // plane
   // pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_p);
   model_p->getDistancesToModel(coefficients, distances);

   std::vector<int> bottom_pt_indexs;
   int expect_bottom_pt_nums = Pcl_utility::bottom_expect_points;
   for (auto i : sort_indexes(distances))
   {
      if (bottom_pt_indexs.size() <= expect_bottom_pt_nums)
         bottom_pt_indexs.push_back(i);
      else
         break;
   }

   return bottom_pt_indexs;
}

eigen_alloc_vector<Eigen::VectorXf> Pcl_cluster_method::GetBoxBottomEdgePlaneCoeffs(pcl::PointCloud<pcl::PointXYZ>::Ptr contourAreaCloud, const Eigen::VectorXf standard_ground_coeffs, const Eigen::Vector3f &standard_box_front_normal, CHECK_DIRECTION if_check, const double &max_shift, const double &threshold, const bool &pos_or_neg)
{
   std::vector<double> distances;

   std::vector<std::vector<int>> vec_inliers;
   Eigen::VectorXf ground_coeff_shift = standard_ground_coeffs;
   Eigen::Vector3f ground_normal = standard_ground_coeffs.head(3);

   // if the direction of normal is the opposite direction of the axis,the symbol is minus sign,otherwise is plus sign
   int sign;
   if (!pos_or_neg)
      sign = -1;
   else
      sign = 1;

   pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
       model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(contourAreaCloud)); // plane

   // Move the groud plane toward the normal vector, and the cut line segment of the box front is used as the target reference edge
   for (double step = 0.0; step <= max_shift; step += threshold)
   {
      std::vector<int> inliers;
      ground_coeff_shift[3] += (float)(sign * step);
      model_p->selectWithinDistance(ground_coeff_shift, threshold, inliers);
      vec_inliers.push_back(inliers);
      std::cout << "the inliers size:" << inliers.size() << std::endl;
   }

   std::cout << "the candidate_pts size:" << vec_inliers.size() << std::endl;
   eigen_alloc_vector<Eigen::VectorXf> box_bottom_coeff_candidates;

   for (auto value : vec_inliers)
   {
      if (value.empty() || value.size() < 5)
         continue;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_bottom(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*contourAreaCloud, value, *cloud_box_bottom);
      std::cout << "This time!!!! [size]:" << value.size() << std::endl;

      // calculate the fitting bottomedge plane coefficient
      Eigen::VectorXf box_bottom_coeff;
      box_bottom_coeff = FittingBottomEdgePlane(cloud_box_bottom, ground_normal);
      if (box_bottom_coeff.isZero())
      {
         std::cout << "the fitting bottom edge plane coefficient:" << box_bottom_coeff << std::endl;
         std::cerr << "[WARNING]fitting box bottom plane may wrong!!" << std::endl;
         continue;
      }

      double inc_angle;
      if (if_check != NO_CHECK)
         SetPositiveDirectNormal(box_bottom_coeff, standard_box_front_normal, inc_angle); // check the orientation

      box_bottom_coeff_candidates.push_back(box_bottom_coeff);
      sub_pcl_utility->visualization(contourAreaCloud, box_bottom_coeff, value); // show the selected edge points in the plane
   }
   return box_bottom_coeff_candidates;
}

Eigen::VectorXf Pcl_cluster_method::FittingBottomEdgePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Vector3f ground_normal)
{
   // Define a space plane segmentation model using angle constraints
   const double ea = (double)(Pcl_utility::bottom_fitting_angle_error * (M_PI / 180.0f)); //angle threshold : 3 degree
   // Eigen::Vector3f ground_normal = coefficients.head(3);

   pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>::Ptr
       model(new pcl::SampleConsensusModelParallelPlane<pcl::PointXYZ>(cloud)); // plane
   model->setAxis(ground_normal);
   model->setEpsAngle(ea);

   // ransac method to fitting the plane
   Eigen::VectorXf bottom_coeff;
   pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
   ransac.setDistanceThreshold(Pcl_utility::bottom_fitting_dist_error); // error less 3 cm
   ransac.computeModel();
   // ransac.getInliers(inliers);
   ransac.getModelCoefficients(bottom_coeff);
   return bottom_coeff;
}
double Pcl_cluster_method::FindMaxDistanceToPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::VectorXf &coeff)
{
   double a, b, c, d;
   a = coeff[0], b = coeff[1], c = coeff[2], d = coeff[3];

   double expansion = 0.0;
   for (auto value : cloud->points)
   {
      // double temp_dist = abs(a * value.x + b * value.y + c * value.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
      double temp_dist = (a * value.x + b * value.y + c * value.z + d) / sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
      if (temp_dist > expansion)
         expansion = temp_dist;
   }
   return expansion;
}

std::vector<pcl::PointIndices> Pcl_cluster_method::GetRegionGrowingCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_colored_cloud)
{
   std::cout << "in cloud size:" << in_cloud->size() << ", the normals size:" << normals->size() << std::endl;
   pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(new pcl::search::KdTree<pcl::PointXYZ>);
   pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
   reg.setMinClusterSize(Pcl_utility::clusters_min_points_size);
   reg.setMaxClusterSize(Pcl_utility::clusters_max_points_size);
   reg.setSearchMethod(tree);
   reg.setNumberOfNeighbours(Pcl_utility::clusters_num_neighbours);
   reg.setInputCloud(in_cloud);
   //reg.setIndices (indices);
   reg.setInputNormals(normals);
   // the normal error threshold
   reg.setSmoothnessThreshold(Pcl_utility::clusters_normal_error_threshold / 180.0 * M_PI);
   // Curvature threshold
   reg.setCurvatureThreshold(Pcl_utility::clusters_curvature_threshold);

   std::vector<pcl::PointIndices> clusters;
   reg.extract(clusters);

   out_colored_cloud = reg.getColoredCloud();
   std::cout << "the clusters size:" << clusters.size() << ", the out colored cloud size:" << out_colored_cloud->size() << std::endl;
   return clusters;
}
void Pcl_cluster_method::ClusterSegementation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<pcl::PointIndices> &clusters)
{
   pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
   // CaculateCloudNormal(cloud, normals);
   EstimateNormal(cloud, normals, KDTREE_MEAN);

   auto colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>().makeShared();
   clusters = GetRegionGrowingCluster(cloud, normals, colored_cloud);

   sub_pcl_utility->rgbVis(colored_cloud, clusters);
   std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
   std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
   std::cout << "These are the indices of the points of the initial" << std::endl
             << "cloud that belong to the first cluster:" << std::endl;
}

pcl::ModelCoefficients::Ptr Pcl_cluster_method::GetGroundPlaneCoeff(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_ground, std::vector<pcl::PointIndices> &clusters)
{
   pcl::ModelCoefficients::Ptr ground_coefficients(new pcl::ModelCoefficients);

   // calculate the normal
   pcl::PointCloud<pcl::Normal>::Ptr normals_ground(new pcl::PointCloud<pcl::Normal>);
   EstimateNormal(cloud_ground, normals_ground, KDTREE_RADIUS);

   // Fitting plane
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
   FittingPlane(cloud_ground, ground_coefficients, inliers); // 不使用法线估计
   // FittingPlane(cloud_ground, normals_ground, ground_coefficients, inliers); // 使用法线估计
   // double eps_angle_in_rad = 3 * M_PI / 180;                                                 // 估计的法线和初始法线相差3°以内
   // FittingPlane(cloud_ground, ground_normal, eps_angle_in_rad, ground_coefficients, inliers); // 使用法线估计
   std::cerr << "Ground plane Model coefficients: " << ground_coefficients->values[0] << " "
             << ground_coefficients->values[1] << " "
             << ground_coefficients->values[2] << " "
             << ground_coefficients->values[3] << std::endl;
   // point cloud rendering and visulization
   sub_pcl_utility->visualization(cloud_ground, normals_ground, ground_coefficients, inliers);
   return ground_coefficients;
}

bool Pcl_cluster_method::SetPositiveDirectNormal(Eigen::VectorXf &input_coeff, Eigen::Vector3f standard_normal, double &included_angle)
{
   Eigen::Vector3f input_normal = input_coeff.head(3);

   double angle_1 = std::acos(input_normal.dot(standard_normal) / input_normal.norm() * standard_normal.norm()); // unit: rad
   double angle_2 = std::acos((-1 * input_normal).dot(standard_normal) / input_normal.norm() * standard_normal.norm());
   // double angle_1 = std::acos(cosVal_1) * 180 / M_PI; // convert to degree unit

   if (std::isnan(angle_1) || std::isnan(angle_2) || (!std::isfinite(angle_1)) || (!std::isfinite(angle_2)))
   {
      std::cerr << "[WARNING]this normal of segement may parallel to ground normal!!" << std::endl;
      return false;
   }
   std::cout << "raw angle_1:" << angle_1 * 180 / M_PI << ",angle_2:" << angle_2 * 180 / M_PI << std::endl;

   //Normalization
   input_coeff[3] = input_coeff[3] / input_normal.norm();
   input_normal.normalize(); // may not needed,the coefficient had be normalized

   double inc_angle;
   if (angle_1 > angle_2 && angle_2 < M_PI / 2)
   {
      std::cout << "HAD changed the direction of normal！！！！" << std::endl;
      input_normal = -1 * input_normal;
      input_coeff[3] *= -1;
      inc_angle = angle_2;
   }
   else
      inc_angle = angle_1;

   input_coeff.head(3) = input_normal;

   included_angle = inc_angle;

   return true;
}
// the clusters is sorted by cloud size
int Pcl_cluster_method::FindBoxFrontClusterIndice(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, std::vector<pcl::PointIndices> clusters, Eigen::Vector3f standard_box_front_normal)
{

   std::map<CLUSTER_INFO, pcl::PointCloud<pcl::PointXYZ>::Ptr, CmpByKeySize> m_segements;

   for (unsigned int i = 0; i < clusters.size(); i++)
   {
      auto segment_cloud = pcl::PointCloud<pcl::PointXYZ>().makeShared();
      CLUSTER_INFO temp_info;

      pcl::copyPointCloud(*cloud, clusters[i], *segment_cloud);
      temp_info.size = segment_cloud->size();
      temp_info.indice = i;

      m_segements.insert(std::make_pair(temp_info, segment_cloud));
      // std::cout << "first sorting the segement ,the size order is:" << segment_cloud->size() << std::endl;
   }

   // Step1: choose big size cloud
   int count = 0;
   int max_segements_nums = Pcl_utility::clusters_maxsize_segements; // Select only the first three maximums
   std::map<double, int> good_cluster_indices;
   for (auto value : m_segements)
   {
      if (count == max_segements_nums)
         break;

      auto segement_pt = value.second;
      std::cout << "sorting the segement ,the size is:" << value.second->size() << " the raw index:" << value.first.indice << std::endl;

      if (value.second->size() < Pcl_utility::standard_point_cloud_size)
         continue;

      // Step2: fitting cluster plane
      // calculating segment's normal
      pcl::PointCloud<pcl::Normal>::Ptr normals_pt(new pcl::PointCloud<pcl::Normal>);
      EstimateNormal(segement_pt, normals_pt, KDTREE_RADIUS);
      pcl::ModelCoefficients::Ptr coefficients_pt(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_pt(new pcl::PointIndices);
      FittingPlane(segement_pt, normals_pt, coefficients_pt, inliers_pt);

      // Step3: calculate the angle between each cluster normal and the normal vector of the frontal view
      // Eigen::Vector3f cur_normal(coefficients_pt->values[0], coefficients_pt->values[1], coefficients_pt->values[2]);
      Eigen::VectorXf cur_coeff(4);
      cur_coeff << coefficients_pt->values[0], coefficients_pt->values[1], coefficients_pt->values[2], coefficients_pt->values[3];
      double inc_angle;
      if (!SetPositiveDirectNormal(cur_coeff, standard_box_front_normal, inc_angle))
      {
         count++;
         continue;
      }

      if (inc_angle)
         good_cluster_indices.insert(std::make_pair(inc_angle, value.first.indice));

      count++;
   }

   return good_cluster_indices.begin()->second;
}
Eigen::VectorXf Pcl_cluster_method::GetBoxBottomEdgePlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_box_front, Eigen::VectorXf &ground_coefficients, CHECK_DIRECTION if_check, const Eigen::Vector3f &standard_box_front_normal)
{
   std::cout << "in the box bottom edge plane,the standard_box_front_normal: " << standard_box_front_normal.transpose() << std::endl;
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box_bottom(new pcl::PointCloud<pcl::PointXYZ>);
   // find bottom points according distances
   std::vector<int> select_bottom_edge;
   select_bottom_edge = FindBottomEdgePoints(cloud_box_front, ground_coefficients);
   pcl::copyPointCloud(*cloud_box_front, select_bottom_edge, *cloud_box_bottom);

   Eigen::Vector3f ground_normal = ground_coefficients.head(3);
   Eigen::VectorXf box_bottom_coeff = FittingBottomEdgePlane(cloud_box_bottom, ground_normal);
   if (box_bottom_coeff.isZero())
   {
      std::cout << "the fitting bottom edge plane coefficient:" << box_bottom_coeff << std::endl;
      std::cerr << "[WARNING]fitting box bottom plane may wrong!!" << std::endl;
      abort();
   }

   double inc_angle;
   if (if_check != NO_CHECK)
      SetPositiveDirectNormal(box_bottom_coeff, standard_box_front_normal, inc_angle); // check the orientation

   sub_pcl_utility->visualization(cloud_box_front, box_bottom_coeff, select_bottom_edge); // show the selected edge points in the plane
   // sub_pcl_utility->simpleVis(cloud_box_front, box_bottom_coeff);       // fiting the bottom edge plane

   return box_bottom_coeff;
}
