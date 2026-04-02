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
  std::string pipeline_libcam = "libcamerasrc ! video/x-raw, width=320, height=240, framerate=10/1 ! videoconvert ! appsink";
  std::string pipeline_v4l2_mjpg = "v4l2src device=/dev/video0 ! image/jpeg, width=320, height=240, framerate=10/1 ! jpegdec ! videoconvert ! appsink";
  
  cap_.open(pipeline_libcam, cv::CAP_GSTREAMER);
  if (!cap_.isOpened()) {
    std::cerr << "CvCameraDriver: libcamerasrc failed, trying v4l2src MJPG pipeline\n";
    cap_.open(pipeline_v4l2_mjpg, cv::CAP_GSTREAMER);
  }

  if (!cap_.isOpened()) {
    std::cerr << "CvCameraDriver: pipelines failed, trying direct V4L2 MJPG\n";
    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      std::cerr << "CvCameraDriver: Failed to open camera\n";
      return false;
    }
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap_.set(cv::CAP_PROP_FPS, 10);
  }
  
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
