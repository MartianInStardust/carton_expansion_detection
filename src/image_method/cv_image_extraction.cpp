/*
 * @Author: Aiden
 * @Date: 2021-07-05 17:45:04
 * @LastEditTime: 2021-07-14 14:29:29
 */

#include "cv_image_extraction.h"
Image_extract::Image_extract()
{
}
Image_extract::~Image_extract()
{
}
// Exclude less than max_area, select the largest area
bool Image_extract::FindTargetContour(std::vector<std::vector<cv::Point>> &contours, int &max_index, double max_area)
{
   int index = 0;
   double max_area_temp = 0.0;
   int max_index_temp;
   std::vector<std::vector<cv::Point>>::iterator iter = contours.begin();
   for (; iter != contours.end();)
   {
      double g_dConArea = contourArea(*iter); // caclulates contour area
      if (g_dConArea < max_area)
      {
         iter = contours.erase(iter); // exclude contour area that not large enough
      }
      else
      {
         ++iter;
         std::cout << "the area is:" << g_dConArea << ", Index:" << index << std::endl;

         if (max_area_temp < g_dConArea)
         {
            max_area_temp = g_dConArea;
            max_index_temp = index;
         }
         index++;
      }
   }
   max_index = max_index_temp;
   std::cout << "the numbers of all valid contours: " << (int)contours.size() << ",the max area:" << max_area_temp << ", the select object index:" << max_index << std::endl;

   if ((int)contours.size() > 0)
      return true;
   else
      return false;
}

std::vector<uint32_t> Image_extract::GetAllImageIndexWithinArea(std::vector<cv::Point> targetContour, int color_w, int color_h)
{
   std::vector<uint32_t> allIndex;

   for (int j = 0; j < color_h; j++)
   {
      for (int i = 0; i < color_w; i++)
      {
         auto result = cv::pointPolygonTest(targetContour, cv::Point2f(i, j), false);
         if (result > 0)
         {
            allIndex.push_back(i + j * color_w);
         }
      }
   }

   return allIndex;
}

TY_CAMERA_CALIB_INFO Image_extract::SetParameters(int32_t width, int32_t height, std::vector<float> intrinsic, std::vector<float> extrinsic, std::vector<float> distortion)
{
   TY_CAMERA_CALIB_INFO m_calib;
   m_calib.intrinsicWidth = width;
   m_calib.intrinsicHeight = height;
   for (int i = 0; i < 9; i++)
      m_calib.intrinsic.data[i] = intrinsic[i];
   for (int i = 0; i < 16; i++)
      m_calib.extrinsic.data[i] = extrinsic[i];
   for (int i = 0; i < 12; i++)
      m_calib.distortion.data[i] = distortion[i];

   return m_calib;
}

void Image_extract::MapImageIndexToCloud(const TY_CAMERA_CALIB_INFO &depth_calib, const TY_CAMERA_CALIB_INFO &color_calib, const cv::Mat &depth, const cv::Mat &color, cv::Mat &out, std::vector<uint32_t> &inAllPointsIndex, std::vector<uint32_t> &outAllPointsIndex)
{
   // do rgb undistortion
   TY_IMAGE_DATA src;
   src.width = color.cols;
   src.height = color.rows;
   src.size = color.size().area() * 3;
   src.pixelFormat = TY_PIXEL_FORMAT_RGB;
   src.buffer = color.data;

   cv::Mat undistort_color = cv::Mat(color.size(), CV_8UC3);
   TY_IMAGE_DATA dst;
   dst.width = color.cols;
   dst.height = color.rows;
   dst.size = undistort_color.size().area() * 3;
   dst.buffer = undistort_color.data;
   dst.pixelFormat = TY_PIXEL_FORMAT_RGB;
   ASSERT_OK(TYUndistortImage(&color_calib, &src, NULL, &dst));

   // do register
   out.create(depth.size(), CV_8UC3);

   ASSERT_OK(MapColorToDepthLookupTable2(
       &depth_calib,
       depth.cols, depth.rows, depth.ptr<uint16_t>(),
       &color_calib,
       undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint8_t>(),
       out.ptr<uint8_t>(), inAllPointsIndex, outAllPointsIndex));
}

std::vector<std::vector<cv::Point>> Image_extract::GetImageRoiContour(cv::Mat color)
{
   cv::Rect roi_area = cv::selectROI("select roi image", color, false);
   cv::Point origin_roi = cv::Point(roi_area.x, roi_area.y);
   cv::Mat roi_img(color, roi_area); //create roi image from raw
   cv::Mat src_gray;
   // cv::cvtColor(roi_img, src_gray, CV_BGR2GRAY);
   cv::cvtColor(roi_img, src_gray, cv::COLOR_BGR2GRAY);
   cv::Mat src_dstImage;

   cv::GaussianBlur(src_gray, src_dstImage, cv::Size(5, 5), 0.0, 0, cv::BORDER_DEFAULT);

   // Canny detected edges
   cv::Mat canny_output;
   std::vector<std::vector<cv::Point>> contours;
   std::vector<cv::Vec4i> hierarchy;
   int thresh = 30;
   cv::Canny(src_dstImage, canny_output, thresh, thresh * 3, 3);

   // dilate in order to get complete boundary
   cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
   cv::dilate(canny_output, canny_output, element);
   // namedWindow("canny", CV_WINDOW_AUTOSIZE);
   cv::namedWindow("canny", cv::WINDOW_NORMAL);
   cv::imshow("canny", canny_output);
   cv::moveWindow("canny", 550, 20);

   // cv::findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, origin_roi);
   cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, origin_roi);
   return contours;
}
