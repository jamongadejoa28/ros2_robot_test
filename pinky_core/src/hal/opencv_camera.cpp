#include "pinky_core/hal/opencv_camera.h"
#include <iostream>

namespace pinky {

OpencvCamera::OpencvCamera() {}

OpencvCamera::~OpencvCamera() {
#ifdef PINKY_HAS_OPENCV
  if (cap_.isOpened()) {
    cap_.release();
  }
#endif
}

bool OpencvCamera::Init() {
#ifdef PINKY_HAS_OPENCV
  cap_.open(0);
  if (!cap_.isOpened()) {
    std::cerr << "OpencvCamera: Failed to open camera device 0\n";
    return false;
  }
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  std::cout << "OpencvCamera: Initialized.\n";
  return true;
#else
  std::cerr << "OpencvCamera: OpenCV not available, camera disabled.\n";
  return false;
#endif
}

bool OpencvCamera::CaptureJpeg(std::vector<uint8_t>& jpeg_out,
                               uint16_t& width, uint16_t& height) {
#ifdef PINKY_HAS_OPENCV
  if (!cap_.isOpened()) return false;

  cv::Mat frame;
  if (!cap_.read(frame) || frame.empty()) {
    return false;
  }

  // Rotate 180 degrees to match hardware orientation (as in pinkylib)
  cv::rotate(frame, frame, cv::ROTATE_180);

  width = static_cast<uint16_t>(frame.cols);
  height = static_cast<uint16_t>(frame.rows);

  std::vector<int> params;
  params.push_back(cv::IMWRITE_JPEG_QUALITY);
  params.push_back(60);

  return cv::imencode(".jpg", frame, jpeg_out, params);
#else
  return false;
#endif
}

}  // namespace pinky