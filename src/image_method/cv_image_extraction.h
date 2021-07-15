/*
 * @Author: Aiden
 * @Date: 2021-07-05 17:44:51
 * @LastEditTime: 2021-07-07 17:44:24
 */

#include <limits>
#include <cassert>
#include <cmath>
#include "common/common.hpp"
#include "TYImageProc.h"

struct CallbackData
{
   int index;
   TY_DEV_HANDLE hDevice;
   TY_ISP_HANDLE isp_handle;
   TY_CAMERA_CALIB_INFO depth_calib;
   TY_CAMERA_CALIB_INFO color_calib;

   DepthRender *render;

   bool saveOneFramePoint3d;
   bool exit_main;
   int fileIndex;
};

class Image_extract
{
public:
   Image_extract();
   ~Image_extract();

   // default max_area:500
   bool FindTargetContour(std::vector<std::vector<cv::Point>> &contours, int &max_index, double max_area);

   std::vector<uint32_t> GetAllImageIndexWithinArea(std::vector<cv::Point> targetContour, int color_w, int color_h);

   TY_CAMERA_CALIB_INFO SetParameters(int32_t width, int32_t height, std::vector<float> intrinsic, std::vector<float> extrinsic, std::vector<float> distortion);

   void MapImageIndexToCloud(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth, const cv::Mat &color, cv::Mat &out, std::vector<uint32_t> &inAllPointsIndex, std::vector<uint32_t> &outAllPointsIndex);

   std::vector<std::vector<cv::Point>> GetImageRoiContour(cv::Mat color);
};