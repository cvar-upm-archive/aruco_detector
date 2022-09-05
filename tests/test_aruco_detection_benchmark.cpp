
#include <benchmark/benchmark.h>

#include <exception>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <random>
#include <vector>

// #include "DF_controller.hpp"

#include "aruco_detector.hpp"
#include "rclcpp/rclcpp.hpp"

ArucoDetector *ptr = nullptr;
sensor_msgs::msg::Image::SharedPtr image;

static void BM_COMPUTE_ACTIONS(benchmark::State &state)
{
  for (auto _ : state)
  {
    ptr->imageCallback(image);
  }
}
BENCHMARK(BM_COMPUTE_ACTIONS)->Threads(1)->Repetitions(10);

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Usage: " << argv[0] << " <path to image>" << std::endl;
    return 1;
  }
  std::string image_path = argv[1];

  rclcpp::init(argc, argv);
  as2::Node node("test");
  ptr = new ArucoDetector();

  if (rcutils_logging_set_logger_level(ptr->get_logger().get_name(), RCUTILS_LOG_SEVERITY_WARN) ==
      RCUTILS_RET_ERROR)
    throw std::runtime_error("Error setting logger level");

  cv::Mat img = cv::imread(image_path);
  image = std::make_shared<sensor_msgs::msg::Image>();

  // load img in image_msg
  cv_bridge::CvImage cv_img;
  cv_img.image = img;
  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.toImageMsg(*image);

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();
  delete ptr;
}
