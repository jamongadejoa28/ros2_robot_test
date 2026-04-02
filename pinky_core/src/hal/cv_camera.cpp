#include "pinky_core/hal/cv_camera.h"
#include <iostream>

namespace pinky {

CvCameraDriver::CvCameraDriver() = default;

CvCameraDriver::~CvCameraDriver() {
#ifdef PINKY_HAS_OPENCV
  if (cap_.isOpened()) {
    cap_.release();
  }
#endif
}

bool CvCameraDriver::Init() {
#ifdef PINKY_HAS_OPENCV
  cap_.open(0);
  if (!cap_.isOpened()) {
    std::cerr << "CvCameraDriver: Failed to open camera 0\n";
    return false;
  }
  // lower resolution to improve stream speed
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
  cap_.set(cv::CAP_PROP_FPS, 10);
  return true;
#else
  std::cerr << "CvCameraDriver: OpenCV not enabled\n";
  return false;
#endif
}

bool CvCameraDriver::CaptureJpeg(std::vector<uint8_t>& jpeg_out, uint16_t& width, uint16_t& height) {
#ifdef PINKY_HAS_OPENCV
  if (!cap_.isOpened()) return false;
  cv::Mat frame;
  if (!cap_.read(frame) || frame.empty()) return false;
  
  width = frame.cols;
  height = frame.rows;
  
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 50};
  return cv::imencode(".jpg", frame, jpeg_out, params);
#else
  return false;
#endif
}

}  // namespace pinky
