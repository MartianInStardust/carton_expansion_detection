#ifndef CAMERA_DATE_H_
#define CAMERA_DATE_H_

#include <iostream>
#include <vector>

#include <Eigen/Eigen>

namespace multi_camera_calibration
{

   struct CameraIntrinsics
   {
      // Sensor resolution
      int32_t Width, Height;

      // Intrinsics
      float cx, cy;
      float fx, fy;
      float k[6];
      float codx, cody;
      float p1, p2;
   };

   // struct CameraExtrinsics
   // {
   //    Eigen::Vector3f translation;
   //    Eigen::Quaternionf rotation;

   //    template <class Archive>
   //    void serialize(Archive &ar)
   //    {
   //       ar(cereal::make_nvp("translation", translation), cereal::make_nvp("rotation", rotation));
   //    }
   // };

   struct CameraCalibration
   {
      // Intrinsics for each camera
      CameraIntrinsics Color, Depth;

      // Extrinsics transform from 3D depth camera point to 3D point relative to color camera
      float RotationFromDepth[3 * 3];
      float TranslationFromDepth[3];
   };

   struct FrameInfo
   {

      CameraCalibration Calibration{};

      // Accelerometer reading for extrinsics calibration
      float Accelerometer[3];

      // Color image
      std::vector<uint8_t> ColorImage;
      int ColorWidth = 0, ColorHeight = 0, ColorStride = 0;

      k4a_image_t k4a_colorImage = NULL;
      k4a_image_t k4a_depthImage = NULL;

      // Depth image
      std::vector<uint16_t> DepthImage;
      int DepthWidth = 0, DepthHeight = 0, DepthStride = 0;

      // Point cloud data
      std::vector<int16_t> PointCloudData;

      uint32_t CameraIndex;
      int FrameNumber;
      char *filename;
   };

   struct AlignmentTransform
   {
      float Transform[16];
      bool Identity = true;
      inline void operator=(const Eigen::Matrix4f &src)
      {
         Identity = src.isIdentity();
         for (int row = 0; row < 4; ++row)
         {
            for (int col = 0; col < 4; ++col)
            {
               Transform[row * 4 + col] = src(row, col);
            }
         }
      }
      inline void Set(Eigen::Matrix4f &dest) const
      {
         if (Identity)
         {
            dest = Eigen::Matrix4f::Identity();
         }
         else
         {
            for (int row = 0; row < 4; ++row)
            {
               for (int col = 0; col < 4; ++col)
               {
                  dest(row, col) = Transform[row * 4 + col];
               }
            }
         }
      }
   };

} // namespace multi_camera_calibration

#endif