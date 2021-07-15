/*
 * @Author: Aiden
 * @Date: 2021-07-08 14:48:22
 * @LastEditTime: 2021-07-12 20:14:55
 */
#ifndef K4A_CAMERA_H_
#define K4A_CAMERA_H_

#include <iostream>
#include <string>
#include <vector>

#include <k4a/k4a.hpp>
#include <k4arecord/playback.hpp>

#include "camera_date.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace multi_camera_calibration
{
   struct recording_t
   {
      char *filename;
      k4a_playback_t handle;
      k4a_record_configuration_t record_config;
      k4a_calibration_t k4a_calibration;
      k4a_transformation_t k4a_transformation;
      CameraCalibration calibration{};
      k4a_capture_t capture;
   };

   class K4aCamera
   {
   public:
      K4aCamera();
      ~K4aCamera();
      bool SetInputFiles(int size, char *filename[]);
      bool GetFrames(std::vector<FrameInfo> &frames);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pcl_point_cloud(const k4a_image_t point_cloud_image,
                                                                      const k4a_image_t color_image);

      bool point_cloud_depth_to_color(int camera_count,
                                      const k4a_image_t depth_image,
                                      const k4a_image_t color_image,
                                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_point_cloud);

      k4a_calibration_t get_k4a_calibration(int camera_count);
      k4a_transformation_t get_k4a_transformation(int camera_count);

   private:
      void CopyInstrinsics(const k4a_calibration_camera_t &from, CameraIntrinsics &to);
      void CalibrationFromK4a(const k4a_calibration_t &from, CameraCalibration &to);
      k4a_result_t SkipCapture(recording_t &file, int size);
      FrameInfo ProcessCapture(recording_t *file);

   private:
      std::vector<recording_t> files;
      int camera_count;
   };
} // namespace multi_camera_calibration

#endif