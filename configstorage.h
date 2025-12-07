#pragma once

#include <jsoncpp/json/json.h>

#include <filesystem>
#include <opencv2/opencv.hpp>
#include <string>

class Config {
public:
  Config(const std::filesystem::path& path);
  cv::Mat getTransformMatrix(const std::string& satellite, const std::string& channel);
  void setTransfromMatrix(const cv::Mat& matrix, const std::string& satellite, const std::string& channel);

private:
  void load();
  void save();

private:
  const std::filesystem::path mConfigPath;
  Json::Value mJsonConfig;
};