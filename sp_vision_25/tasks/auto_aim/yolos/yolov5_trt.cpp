#include "yolov5_trt.hpp"

#include <fmt/chrono.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace
{

std::size_t dims_volume(const nvinfer1::Dims & dims)
{
  std::size_t volume = 1;
  for (int i = 0; i < dims.nbDims; ++i) {
    if (dims.d[i] <= 0) {
      throw std::runtime_error("TensorRT engine contains a dynamic or invalid tensor dimension");
    }
    volume *= static_cast<std::size_t>(dims.d[i]);
  }
  return volume;
}

void check_cuda(cudaError_t result, const char * operation)
{
  if (result != cudaSuccess) {
    throw std::runtime_error(
      std::string(operation) + " failed: " + cudaGetErrorString(result));
  }
}

}  // namespace

namespace auto_aim
{

void YOLOV5TRT::TRTLogger::log(Severity severity, const char * message) noexcept
{
  if (severity == Severity::kINTERNAL_ERROR || severity == Severity::kERROR) {
    tools::logger()->error("[TensorRT] {}", message);
  } else if (severity == Severity::kWARNING) {
    tools::logger()->warn("[TensorRT] {}", message);
  } else if (severity == Severity::kINFO) {
    tools::logger()->info("[TensorRT] {}", message);
  }
}

YOLOV5TRT::YOLOV5TRT(const std::string & config_path, bool debug)
: debug_(debug), detector_(config_path, false)
{
  const auto yaml = YAML::LoadFile(config_path);
  engine_path_ = yaml["yolov5_trt_engine_path"].as<std::string>();
  gpu_id_ = yaml["gpu_id"] ? yaml["gpu_id"].as<int>() : 0;
  min_confidence_ = yaml["min_confidence"].as<double>();
  use_roi_ = yaml["use_roi"].as<bool>();
  use_traditional_ = yaml["use_traditional"].as<bool>();

  roi_ = cv::Rect(
    yaml["roi"]["x"].as<int>(), yaml["roi"]["y"].as<int>(),
    yaml["roi"]["width"].as<int>(), yaml["roi"]["height"].as<int>());
  offset_ = cv::Point2f(roi_.x, roi_.y);

  save_path_ = "imgs";
  std::filesystem::create_directories(save_path_);

  check_cuda(cudaSetDevice(gpu_id_), "cudaSetDevice");
  load_engine();
  allocate_buffers();

  tools::logger()->info(
    "TensorRT YOLOv5 ready: engine={} input={}x{} output={}x{} gpu={}", engine_path_,
    input_width_, input_height_, output_dims_.d[1], output_dims_.d[2], gpu_id_);
}

YOLOV5TRT::~YOLOV5TRT()
{
  release_buffers();
}

void YOLOV5TRT::load_engine()
{
  std::ifstream engine_file(engine_path_, std::ios::binary);
  if (!engine_file) {
    throw std::runtime_error("Failed to open TensorRT engine: " + engine_path_);
  }
  engine_file.seekg(0, std::ios::end);
  const auto engine_size = static_cast<std::size_t>(engine_file.tellg());
  engine_file.seekg(0, std::ios::beg);
  if (engine_size == 0) {
    throw std::runtime_error("TensorRT engine is empty: " + engine_path_);
  }

  std::vector<char> engine_data(engine_size);
  engine_file.read(engine_data.data(), static_cast<std::streamsize>(engine_data.size()));
  if (!engine_file) {
    throw std::runtime_error("Failed to read TensorRT engine: " + engine_path_);
  }

  runtime_.reset(nvinfer1::createInferRuntime(logger_));
  if (!runtime_) throw std::runtime_error("Failed to create TensorRT runtime");
  engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
  if (!engine_) throw std::runtime_error("Failed to deserialize TensorRT engine");
  context_.reset(engine_->createExecutionContext());
  if (!context_) throw std::runtime_error("Failed to create TensorRT execution context");

  for (int i = 0; i < engine_->getNbIOTensors(); ++i) {
    const char * name = engine_->getIOTensorName(i);
    const auto mode = engine_->getTensorIOMode(name);
    if (mode == nvinfer1::TensorIOMode::kINPUT && input_name_.empty()) {
      input_name_ = name;
    } else if (mode == nvinfer1::TensorIOMode::kOUTPUT && output_name_.empty()) {
      output_name_ = name;
    }
  }
  if (input_name_.empty() || output_name_.empty()) {
    throw std::runtime_error("TensorRT engine must contain one input and one output");
  }

  input_dims_ = context_->getTensorShape(input_name_.c_str());
  output_dims_ = context_->getTensorShape(output_name_.c_str());
  if (
    input_dims_.nbDims != 4 || input_dims_.d[0] != 1 || input_dims_.d[1] != 3 ||
    output_dims_.nbDims != 3 || output_dims_.d[0] != 1 || output_dims_.d[2] != 22) {
    throw std::runtime_error("TensorRT engine shape does not match YOLOv5 armor detector");
  }
  if (
    engine_->getTensorDataType(input_name_.c_str()) != nvinfer1::DataType::kHALF ||
    engine_->getTensorDataType(output_name_.c_str()) != nvinfer1::DataType::kFLOAT) {
    throw std::runtime_error("TensorRT engine must use FP16 input and FP32 output");
  }

  input_height_ = input_dims_.d[2];
  input_width_ = input_dims_.d[3];
  input_bytes_ = dims_volume(input_dims_) * sizeof(std::uint16_t);
  output_bytes_ = dims_volume(output_dims_) * sizeof(float);
}

void YOLOV5TRT::allocate_buffers()
{
  check_cuda(cudaStreamCreate(&stream_), "cudaStreamCreate");
  check_cuda(cudaMalloc(&device_input_, input_bytes_), "cudaMalloc(input)");
  check_cuda(cudaMalloc(&device_output_, output_bytes_), "cudaMalloc(output)");
  check_cuda(
    cudaMallocHost(reinterpret_cast<void **>(&host_input_), input_bytes_),
    "cudaMallocHost(input)");
  check_cuda(
    cudaMallocHost(reinterpret_cast<void **>(&host_output_), output_bytes_),
    "cudaMallocHost(output)");
}

void YOLOV5TRT::release_buffers() noexcept
{
  if (stream_ != nullptr) cudaStreamSynchronize(stream_);
  if (device_input_ != nullptr) cudaFree(device_input_);
  if (device_output_ != nullptr) cudaFree(device_output_);
  if (host_input_ != nullptr) cudaFreeHost(host_input_);
  if (host_output_ != nullptr) cudaFreeHost(host_output_);
  if (stream_ != nullptr) cudaStreamDestroy(stream_);
  device_input_ = nullptr;
  device_output_ = nullptr;
  host_input_ = nullptr;
  host_output_ = nullptr;
  stream_ = nullptr;
  context_.reset();
  engine_.reset();
  runtime_.reset();
}

void YOLOV5TRT::preprocess(const cv::Mat & bgr_img)
{
  cv::Mat input_bgr(input_height_, input_width_, CV_8UC3, cv::Scalar(0, 0, 0));
  const auto x_scale = static_cast<double>(input_width_) / bgr_img.cols;
  const auto y_scale = static_cast<double>(input_height_) / bgr_img.rows;
  const auto scale = std::min(x_scale, y_scale);
  const int resized_width = static_cast<int>(bgr_img.cols * scale);
  const int resized_height = static_cast<int>(bgr_img.rows * scale);
  cv::resize(
    bgr_img, input_bgr(cv::Rect(0, 0, resized_width, resized_height)),
    cv::Size(resized_width, resized_height));

  cv::Mat input_rgb;
  cv::cvtColor(input_bgr, input_rgb, cv::COLOR_BGR2RGB);
  cv::Mat input_f16;
  input_rgb.convertTo(input_f16, CV_16FC3, 1.0 / 255.0);
  std::vector<cv::Mat> channels;
  cv::split(input_f16, channels);
  if (channels.size() != 3) throw std::runtime_error("TensorRT preprocessing failed");

  const auto plane_bytes =
    static_cast<std::size_t>(input_width_) * input_height_ * sizeof(std::uint16_t);
  for (std::size_t channel = 0; channel < channels.size(); ++channel) {
    if (!channels[channel].isContinuous()) channels[channel] = channels[channel].clone();
    std::memcpy(
      host_input_ + channel * static_cast<std::size_t>(input_width_) * input_height_,
      channels[channel].data, plane_bytes);
  }
}

std::list<Armor> YOLOV5TRT::detect(const cv::Mat & raw_img, int frame_count)
{
  if (raw_img.empty()) {
    tools::logger()->warn("Empty img!, camera drop!");
    return {};
  }

  cv::Mat bgr_img;
  if (use_roi_) {
    if (roi_.width == -1) roi_.width = raw_img.cols;
    if (roi_.height == -1) roi_.height = raw_img.rows;
    bgr_img = raw_img(roi_);
  } else {
    bgr_img = raw_img;
  }

  const auto x_scale = static_cast<double>(input_width_) / bgr_img.cols;
  const auto y_scale = static_cast<double>(input_height_) / bgr_img.rows;
  const auto scale = std::min(x_scale, y_scale);
  preprocess(bgr_img);

  check_cuda(
    cudaMemcpyAsync(
      device_input_, host_input_, input_bytes_, cudaMemcpyHostToDevice, stream_),
    "cudaMemcpyAsync(input)");
  if (!context_->setTensorAddress(input_name_.c_str(), device_input_)) {
    throw std::runtime_error("Failed to set TensorRT input address");
  }
  if (!context_->setTensorAddress(output_name_.c_str(), device_output_)) {
    throw std::runtime_error("Failed to set TensorRT output address");
  }
  if (!context_->enqueueV3(stream_)) throw std::runtime_error("TensorRT enqueueV3 failed");
  check_cuda(
    cudaMemcpyAsync(
      host_output_, device_output_, output_bytes_, cudaMemcpyDeviceToHost, stream_),
    "cudaMemcpyAsync(output)");
  check_cuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");

  cv::Mat output(output_dims_.d[1], output_dims_.d[2], CV_32F, host_output_);
  return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV5TRT::parse(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  std::vector<int> color_ids, num_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;
  std::vector<std::vector<cv::Point2f>> armor_key_points_list;
  for (int row = 0; row < output.rows; ++row) {
    const double confidence = sigmoid(output.at<float>(row, 8));
    if (confidence < score_threshold_) continue;

    cv::Point class_id, color_id;
    double class_score = 0.0, color_score = 0.0;
    cv::minMaxLoc(output.row(row).colRange(9, 13), nullptr, &color_score, nullptr, &color_id);
    cv::minMaxLoc(output.row(row).colRange(13, 22), nullptr, &class_score, nullptr, &class_id);

    std::vector<cv::Point2f> points = {
      {static_cast<float>(output.at<float>(row, 0) / scale),
       static_cast<float>(output.at<float>(row, 1) / scale)},
      {static_cast<float>(output.at<float>(row, 6) / scale),
       static_cast<float>(output.at<float>(row, 7) / scale)},
      {static_cast<float>(output.at<float>(row, 4) / scale),
       static_cast<float>(output.at<float>(row, 5) / scale)},
      {static_cast<float>(output.at<float>(row, 2) / scale),
       static_cast<float>(output.at<float>(row, 3) / scale)}};

    float min_x = points.front().x;
    float max_x = points.front().x;
    float min_y = points.front().y;
    float max_y = points.front().y;
    for (const auto & point : points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }

    color_ids.emplace_back(color_id.x);
    num_ids.emplace_back(class_id.x);
    confidences.emplace_back(static_cast<float>(confidence));
    boxes.emplace_back(
      static_cast<int>(min_x), static_cast<int>(min_y),
      static_cast<int>(max_x - min_x), static_cast<int>(max_y - min_y));
    armor_key_points_list.emplace_back(std::move(points));
  }

  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

  std::list<Armor> armors;
  for (const auto index : indices) {
    if (use_roi_) {
      armors.emplace_back(
        color_ids[index], num_ids[index], confidences[index], boxes[index],
        armor_key_points_list[index], offset_);
    } else {
      armors.emplace_back(
        color_ids[index], num_ids[index], confidences[index], boxes[index],
        armor_key_points_list[index]);
    }
  }

  tmp_img_ = bgr_img;
  for (auto iterator = armors.begin(); iterator != armors.end();) {
    if (!check_name(*iterator) || !check_type(*iterator)) {
      iterator = armors.erase(iterator);
      continue;
    }
    if (use_traditional_) detector_.detect(*iterator, bgr_img);
    iterator->center_norm = get_center_norm(bgr_img, iterator->center);
    ++iterator;
  }

  if (debug_) draw_detections(bgr_img, armors, frame_count);
  return armors;
}

bool YOLOV5TRT::check_name(const Armor & armor) const
{
  return armor.name != ArmorName::not_armor && armor.confidence > min_confidence_;
}

bool YOLOV5TRT::check_type(const Armor & armor) const
{
  return (armor.type == ArmorType::small)
           ? (armor.name != ArmorName::one && armor.name != ArmorName::base)
           : (armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
              armor.name != ArmorName::outpost);
}

cv::Point2f YOLOV5TRT::get_center_norm(
  const cv::Mat & image, const cv::Point2f & center) const
{
  return {center.x / image.cols, center.y / image.rows};
}

void YOLOV5TRT::draw_detections(
  const cv::Mat & image, const std::list<Armor> & armors, int frame_count) const
{
  auto detection = image.clone();
  tools::draw_text(detection, fmt::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
  for (const auto & armor : armors) {
    const auto info = fmt::format(
      "{:.2f} {} {} {}", armor.confidence, COLORS[armor.color], ARMOR_NAMES[armor.name],
      ARMOR_TYPES[armor.type]);
    tools::draw_points(detection, armor.points, {0, 255, 0});
    tools::draw_text(detection, info, armor.center, {0, 255, 0});
  }
  if (use_roi_) cv::rectangle(detection, roi_, cv::Scalar(0, 255, 0), 2);
  cv::resize(detection, detection, {}, 0.5, 0.5);
  cv::imshow("detection", detection);
}

void YOLOV5TRT::save(const Armor & armor) const
{
  const auto filename = fmt::format(
    "{}/{}_{}.jpg", save_path_, armor.name,
    fmt::format("{:%Y-%m-%d_%H-%M-%S}", std::chrono::system_clock::now()));
  cv::imwrite(filename, tmp_img_);
}

double YOLOV5TRT::sigmoid(double value)
{
  if (value > 0.0) return 1.0 / (1.0 + std::exp(-value));
  const auto exponential = std::exp(value);
  return exponential / (1.0 + exponential);
}

std::list<Armor> YOLOV5TRT::postprocess(
  double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count)
{
  return parse(scale, output, bgr_img, frame_count);
}

}  // namespace auto_aim
