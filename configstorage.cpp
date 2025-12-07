#include "configstorage.h"

#include <fstream>

Config::Config(const std::filesystem::path& path)
  : mConfigPath(path) {
  std::ifstream ifstream(path);
  if (ifstream.is_open()) {
    ifstream >> mJsonConfig;
    ifstream.close();
  }
}

cv::Mat Config::getTransformMatrix(const std::string& satellite, const std::string& channel) {
  if (mJsonConfig[satellite].isNull() == true) {
    throw std::runtime_error("Selected satellite '" + satellite + "' config doesn't exist");
  }

  auto satConf = mJsonConfig[satellite];
  if (satConf[channel].isNull() == true) {
    throw std::runtime_error("Selected satellite channel '" + channel + "' config doesn't exist");
  }

  auto channelConf = satConf[channel];
  float a = channelConf["a"].asFloat();
  float b = channelConf["b"].asFloat();
  float c = channelConf["c"].asFloat();
  float d = channelConf["d"].asFloat();
  float tx = channelConf["tx"].asFloat();
  float ty = channelConf["ty"].asFloat();

  return (cv::Mat_<float>(2, 3) << a, b, tx, c, d, ty);
}
void Config::setTransfromMatrix(const cv::Mat& matrix, const std::string& satellite, const std::string& channel) {
  Json::Value config(Json::objectValue);
  config["a"] = matrix.at<float>(0, 0);
  config["b"] = matrix.at<float>(0, 1);
  config["c"] = matrix.at<float>(1, 0);
  config["d"] = matrix.at<float>(1, 1);
  config["tx"] = matrix.at<float>(0, 2);
  config["ty"] = matrix.at<float>(1, 2);

  mJsonConfig[satellite][channel] = config;

  save();
}

void Config::save() {
  int rc = 0;
  Json::StreamWriterBuilder jsonStreamWriter;
  jsonStreamWriter["indentation"] = "";
  std::string jsonString = Json::writeString(jsonStreamWriter, mJsonConfig);

  std::ofstream ofstream(mConfigPath, std::fstream::out | std::ios::trunc);
  if (ofstream.is_open()) {
    ofstream << jsonString;
    ofstream.close();
  } else {
  }
}