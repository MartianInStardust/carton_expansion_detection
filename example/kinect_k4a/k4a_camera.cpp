#include "k4a_camera.h"

namespace multi_camera_calibration
{
   K4aCamera::K4aCamera()
   {
   }

   bool K4aCamera::SetInputFiles(int size, char *filename[])
   {
      k4a_result_t result = K4A_RESULT_FAILED;
      files.resize(size);
      camera_count = size;
      for (int i = 0; i < size; i++)
      {
         files[i].filename = filename[i];
         if (k4a_playback_open(files[i].filename, &files[i].handle) != K4A_RESULT_SUCCEEDED)
         {
            printf("Failed to open recording file: %s\n", files[i].filename);
            return 0;
         }
         if (k4a_playback_get_record_configuration(files[i].handle, &files[i].record_config) != K4A_RESULT_SUCCEEDED)
         {
            printf("Failed to get record configuration for file: %s\n", files[i].filename);
            return 0;
         }
         if (k4a_playback_get_calibration(files[i].handle, &files[i].k4a_calibration) != K4A_RESULT_SUCCEEDED)
         {
            printf("Failed to get record calibration for file: %s\n", files[i].filename);
            return 0;
         }
         if (k4a_playback_set_color_conversion(files[i].handle, K4A_IMAGE_FORMAT_COLOR_BGRA32) != K4A_RESULT_SUCCEEDED)
         {
            printf("Failed to set color conversion for file: %s\n", files[i].filename);
            return 0;
         }
         CalibrationFromK4a(files[i].k4a_calibration, files[i].calibration);
         files[i].k4a_transformation = k4a_transformation_create(&files[i].k4a_calibration);
         result = SkipCapture(files[i], 30);
      }
      if (result == K4A_RESULT_SUCCEEDED)
      {
         return true;
      }
      return 0;
   }

   bool K4aCamera::GetFrames(std::vector<FrameInfo> &frames)
   {
      k4a_stream_result_t stream_result;

      // Find the lowest timestamp out of each of the current captures.
      for (size_t i = 0; i < camera_count; i++)
      {
         stream_result = k4a_playback_get_next_capture(files[i].handle, &files[i].capture);
         if (K4A_STREAM_RESULT_SUCCEEDED == stream_result)
         {

            recording_t *min_file = &files[i];
            FrameInfo frame = ProcessCapture(min_file);
            frame.Calibration = files[i].calibration;
            k4a_imu_sample_t imu_sample;
            stream_result = k4a_playback_get_next_imu_sample(files[i].handle, &imu_sample);
            for (int j = 0; j < 3; ++j)
            {
               frame.Accelerometer[j] = imu_sample.acc_sample.v[j];
            }
            frame.CameraIndex = i;
            frame.filename = files[i].filename;
            frames.push_back(frame);
            // k4a_capture_release(min_file->capture);
            // min_file->capture = NULL;
            std::cout << "Double check in k4a info [color_image]:" << k4a_image_get_size(frame.k4a_colorImage) << ",[depth_image]:" << k4a_image_get_size(frame.k4a_depthImage) << std::endl;
         }
         else
         {
            return false;
         }
      }
      return true;
   }

   K4aCamera::~K4aCamera()
   {
      for (size_t i = 0; i < files.size(); i++)
      {
         if (files[i].handle != NULL)
         {
            k4a_playback_close(files[i].handle);
            files[i].handle = NULL;
         }
      }
   }

   void K4aCamera::CopyInstrinsics(const k4a_calibration_camera_t &from, CameraIntrinsics &to)
   {
      to.Width = from.resolution_width;
      to.Height = from.resolution_height;

      const k4a_calibration_intrinsic_parameters_t &params = from.intrinsics.parameters;
      to.cx = params.param.cx;
      to.cy = params.param.cy;
      to.fx = params.param.fx;
      to.fy = params.param.fy;
      to.k[0] = params.param.k1;
      to.k[1] = params.param.k2;
      to.k[2] = params.param.k3;
      to.k[3] = params.param.k4;
      to.k[4] = params.param.k5;
      to.k[5] = params.param.k6;
      to.codx = params.param.codx;
      to.cody = params.param.cody;
      to.p1 = params.param.p1;
      to.p2 = params.param.p2;
   }

   void K4aCamera::CalibrationFromK4a(const k4a_calibration_t &from, CameraCalibration &to)
   {
      CopyInstrinsics(from.depth_camera_calibration, to.Depth);
      CopyInstrinsics(from.color_camera_calibration, to.Color);
      // Extrinsics from depth to color camera
      const k4a_calibration_extrinsics_t *extrinsics = &from.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
      for (int i = 0; i < 9; ++i)
      {
         to.RotationFromDepth[i] = extrinsics->rotation[i];
      }
      for (int i = 0; i < 3; ++i)
      {
         to.TranslationFromDepth[i] = extrinsics->translation[i];
      }
   }

   FrameInfo K4aCamera::ProcessCapture(recording_t *file)
   {
      FrameInfo frame;
      k4a_image_t images[2];
      images[0] = k4a_capture_get_color_image(file->capture);
      images[1] = k4a_capture_get_depth_image(file->capture);

      // Copy color
      frame.ColorWidth = k4a_image_get_width_pixels(images[0]);
      frame.ColorHeight = k4a_image_get_height_pixels(images[0]);
      frame.ColorStride = k4a_image_get_stride_bytes(images[0]);
      const uint8_t *color_image = reinterpret_cast<const uint8_t *>(k4a_image_get_buffer(images[0]));
      const size_t color_size = k4a_image_get_size(images[0]);
      frame.ColorImage.resize(color_size);
      memcpy(frame.ColorImage.data(), color_image, color_size);

      // Copy depth
      frame.DepthWidth = k4a_image_get_width_pixels(images[1]);
      frame.DepthHeight = k4a_image_get_height_pixels(images[1]);
      frame.DepthStride = k4a_image_get_stride_bytes(images[1]);
      const unsigned depth_size = frame.DepthStride * frame.DepthHeight;
      frame.DepthImage.resize(depth_size);
      const uint16_t *depth_image = reinterpret_cast<const uint16_t *>(k4a_image_get_buffer(images[1]));
      memcpy(frame.DepthImage.data(), depth_image, depth_size * 2);

      k4a_image_t transformed_depth_image = NULL;
      if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                   frame.ColorWidth,
                                                   frame.ColorHeight,
                                                   frame.ColorWidth * (int)sizeof(uint16_t),
                                                   &transformed_depth_image))
      {
         std::cout << "Failed to create transformed depth image" << std::endl;
      }

      k4a_image_t point_cloud_image = NULL;
      if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   frame.ColorWidth,
                                                   frame.ColorHeight,
                                                   frame.ColorWidth * 3 * (int)sizeof(int16_t),
                                                   &point_cloud_image))
      {
         printf("Failed to create point cloud image\n");
      }
      if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_color_camera(file->k4a_transformation, images[1], transformed_depth_image))
      {
         std::cout << "Failed to compute transformed depth image" << std::endl;
      }
      if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(file->k4a_transformation, transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, point_cloud_image))
      {
         std::cout << "Failed to compute point cloud image" << std::endl;
      }

      std::cout << "in k4a info [depth_image]:" << k4a_image_get_size(images[1]) << ",[transformed_depth_image]:" << k4a_image_get_size(transformed_depth_image) << ",[point_cloud_image]:" << k4a_image_get_size(point_cloud_image) << std::endl;
      frame.k4a_colorImage = images[0];
      frame.k4a_depthImage = images[1];
      std::cout << "After in k4a info [color_image]:" << k4a_image_get_size(frame.k4a_colorImage) << ",[depth_image]:" << k4a_image_get_size(frame.k4a_depthImage) << std::endl;

      // Copy point cloud
      const size_t cloud_size = k4a_image_get_size(point_cloud_image);
      const int16_t *point_cloud_image_data =
          reinterpret_cast<const int16_t *>(k4a_image_get_buffer(point_cloud_image));
      frame.PointCloudData.resize(cloud_size);
      memcpy(frame.PointCloudData.data(), point_cloud_image_data, cloud_size);

      k4a_image_release(transformed_depth_image);
      k4a_image_release(point_cloud_image);

      printf("%-32s", file->filename);
      for (int i = 0; i < 2; i++)
      {
         if (images[i] != NULL)
         {
            uint64_t timestamp = k4a_image_get_device_timestamp_usec(images[i]);
            printf("  %7ju usec", timestamp);
            k4a_image_release(images[i]);
            images[i] = NULL;
         }
         else
         {
            printf("  %12s", "");
         }
      }
      printf("\n");

      return frame;
   }

   k4a_result_t K4aCamera::SkipCapture(recording_t &file, int size)
   {
      k4a_result_t result = K4A_RESULT_SUCCEEDED;
      for (int j = 0; j < size; j++)
      {
         k4a_stream_result_t stream_result = k4a_playback_get_next_capture(file.handle, &file.capture);
         if (stream_result == K4A_STREAM_RESULT_EOF)
         {
            printf("ERROR: Recording file is empty: %s\n", file.filename);
            result = K4A_RESULT_FAILED;
            break;
         }
         else if (stream_result == K4A_STREAM_RESULT_FAILED)
         {
            printf("ERROR: Failed to read first capture from file: %s\n", file.filename);
            result = K4A_RESULT_FAILED;
            break;
         }
         k4a_capture_release(file.capture);
      }
      return result;
   }
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr K4aCamera::generate_pcl_point_cloud(const k4a_image_t point_cloud_image,
                                                                              const k4a_image_t color_image)
   {
      int width = k4a_image_get_width_pixels(point_cloud_image);
      int height = k4a_image_get_height_pixels(color_image);
      std::cout << "generate_pcl_point_cloud [width,height]:" << width << "," << height << std::endl;

      int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image);
      uint8_t *color_image_data = k4a_image_get_buffer(color_image);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      auto setDefault = [](pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_ptr)
      {
         pcl::PointXYZRGB point_cloud;
         point_cloud.x = 0.0f;
         point_cloud.y = 0.0f;
         point_cloud.z = 0.0f;
         point_cloud.r = 0;
         point_cloud.g = 0;
         point_cloud.b = 0;
         point_cloud_ptr->points.push_back(point_cloud);
      };

      for (int i = 0; i < width * height; i++)
      {

         pcl::PointXYZRGB point_cloud;
         point_cloud.x = (float)point_cloud_image_data[3 * i + 0];
         point_cloud.y = (float)point_cloud_image_data[3 * i + 1];
         point_cloud.z = (float)point_cloud_image_data[3 * i + 2];

         if (point_cloud.z == 0)
         {
            setDefault(point_cloud_ptr);
            continue;
         }

         // the image type BGRA
         point_cloud.r = (int)(color_image_data[4 * i + 2]);
         point_cloud.g = (int)(color_image_data[4 * i + 1]);
         point_cloud.b = (int)(color_image_data[4 * i + 0]);
         uint8_t alpha = color_image_data[4 * i + 3];

         if (point_cloud.r == 0 && point_cloud.g == 0 && point_cloud.b == 0 && alpha == 0)
         {
            setDefault(point_cloud_ptr);
            continue;
         }

         point_cloud_ptr->points.push_back(point_cloud);
      }

      point_cloud_ptr->width = point_cloud_ptr->size();
      point_cloud_ptr->height = 1;

      return point_cloud_ptr;
   }

   bool K4aCamera::point_cloud_depth_to_color(int camera_count,
                                              const k4a_image_t depth_image,
                                              const k4a_image_t color_image,
                                              pcl::PointCloud<pcl::PointXYZRGB>::Ptr &color_point_cloud)
   {
      k4a_transformation_t transformation_handle;
      transformation_handle = get_k4a_transformation(camera_count);

      // transform color image into depth camera geometry
      int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
      int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
      k4a_image_t transformed_depth_image = NULL;
      if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                                                   color_image_width_pixels,
                                                   color_image_height_pixels,
                                                   color_image_width_pixels * (int)sizeof(uint16_t),
                                                   &transformed_depth_image))
      {
         printf("Failed to create transformed depth image\n");
         return false;
      }

      k4a_image_t point_cloud_image = NULL;
      if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   color_image_width_pixels,
                                                   color_image_height_pixels,
                                                   color_image_width_pixels * 3 * (int)sizeof(int16_t),
                                                   &point_cloud_image))
      {
         printf("Failed to create point cloud image\n");
         return false;
      }

      if (K4A_RESULT_SUCCEEDED !=
          k4a_transformation_depth_image_to_color_camera(transformation_handle, depth_image, transformed_depth_image))
      {
         printf("Failed to compute transformed depth image\n");
         return false;
      }

      if (K4A_RESULT_SUCCEEDED != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
                                                                                transformed_depth_image,
                                                                                K4A_CALIBRATION_TYPE_COLOR,
                                                                                point_cloud_image))
      {
         printf("Failed to compute point cloud\n");
         return false;
      }

      color_point_cloud = generate_pcl_point_cloud(point_cloud_image, color_image);

      k4a_image_release(transformed_depth_image);
      k4a_image_release(point_cloud_image);

      return true;
   }
   k4a_calibration_t K4aCamera::get_k4a_calibration(int camera_count)
   {
      return files[camera_count].k4a_calibration;
   }
   k4a_transformation_t K4aCamera::get_k4a_transformation(int camera_count)
   {
      return files[camera_count].k4a_transformation;
   }

} // namespace multi_camera_calibration