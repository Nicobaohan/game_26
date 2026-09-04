#include "yolo.hpp"

#include <yaml-cpp/yaml.h>

#include "yolos/yolo11.hpp"
#include "yolos/yolov5.hpp"
#include "yolos/yolov8.hpp"

#if defined(SP_VISION_HAS_TENSORRT) && SP_VISION_HAS_TENSORRT
#include "yolos/yolov5_trt.hpp"
#endif

namespace auto_aim
{
YOLO::YOLO(const std::string & config_path, bool debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  const auto infer_backend =
    yaml["infer_backend"] ? yaml["infer_backend"].as<std::string>() : "openvino";

  if (yolo_name == "yolov8") {
    yolo_ = std::make_unique<YOLOV8>(config_path, debug);
  }

  else if (yolo_name == "yolo11") {
    yolo_ = std::make_unique<YOLO11>(config_path, debug);
  }

  else if (yolo_name == "yolov5" && infer_backend == "openvino") {
    yolo_ = std::make_unique<YOLOV5>(config_path, debug);
  }

#if defined(SP_VISION_HAS_TENSORRT) && SP_VISION_HAS_TENSORRT
  else if (yolo_name == "yolov5" && infer_backend == "trt") {
    yolo_ = std::make_unique<YOLOV5TRT>(config_path, debug);
  }
#endif

  else {
    throw std::runtime_error(
      "Unsupported detector configuration: yolo_name=" + yolo_name +
      " infer_backend=" + infer_backend);
  }
}

std::list<Armor> YOLO::detect(const cv::Mat & img, int frame_count)
{
  return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
