#ifndef AUTO_AIM__YOLOV5_TRT_HPP
#define AUTO_AIM__YOLOV5_TRT_HPP

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstdint>
#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"

namespace auto_aim
{

class YOLOV5TRT : public YOLOBase
{
public:
  YOLOV5TRT(const std::string & config_path, bool debug);
  ~YOLOV5TRT() override;

  std::list<Armor> detect(const cv::Mat & bgr_img, int frame_count) override;

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) override;

private:
  class TRTLogger : public nvinfer1::ILogger
  {
  public:
    void log(Severity severity, const char * message) noexcept override;
  };

  void load_engine();
  void allocate_buffers();
  void release_buffers() noexcept;
  void preprocess(const cv::Mat & bgr_img);

  bool check_name(const Armor & armor) const;
  bool check_type(const Armor & armor) const;
  cv::Point2f get_center_norm(const cv::Mat & image, const cv::Point2f & center) const;
  std::list<Armor> parse(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);
  void save(const Armor & armor) const;
  void draw_detections(
    const cv::Mat & image, const std::list<Armor> & armors, int frame_count) const;
  static double sigmoid(double value);

  std::string engine_path_;
  std::string save_path_;
  int gpu_id_ = 0;
  bool debug_ = false;
  bool use_roi_ = false;
  bool use_traditional_ = false;

  const float nms_threshold_ = 0.3F;
  const float score_threshold_ = 0.7F;
  double min_confidence_ = 0.8;

  cv::Rect roi_;
  cv::Point2f offset_;
  cv::Mat tmp_img_;
  Detector detector_;

  TRTLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;

  std::string input_name_;
  std::string output_name_;
  nvinfer1::Dims input_dims_{};
  nvinfer1::Dims output_dims_{};
  int input_width_ = 640;
  int input_height_ = 640;
  std::size_t input_bytes_ = 0;
  std::size_t output_bytes_ = 0;

  cudaStream_t stream_ = nullptr;
  void * device_input_ = nullptr;
  void * device_output_ = nullptr;
  std::uint16_t * host_input_ = nullptr;
  float * host_output_ = nullptr;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLOV5_TRT_HPP
