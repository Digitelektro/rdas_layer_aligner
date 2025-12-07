#include "parameterparser.h"


ParameterParser::ParameterParser() {
  mSettingsList.push_back(ParameterData("--help", "-h", "Print help"));
  mSettingsList.push_back(ParameterData("--sat", "-s", "Satellite name"));
  mSettingsList.push_back(ParameterData("--mode", "-m", "calibrate|generate (default:generate)"));
  mSettingsList.push_back(ParameterData("--image", "-i", "Input image"));
  mSettingsList.push_back(ParameterData("--outimage", "-o", "Output image"));
}

void ParameterParser::parseArgs(int argc, char** argv) {
  for (int i = 1; i < (argc - 1); i += 2) {
    mArgs.insert(std::make_pair(argv[i], argv[i + 1]));
  }
}

std::string ParameterParser::getHelp() const {
  std::list<ParameterData>::const_iterator it;
  std::stringstream ss;

  // ss << "RDAS Layer allign Version " << VERSION_MAJOR << "." << VERSION_MINOR << "." << VERSION_FIX << std::endl;
  for (it = mSettingsList.begin(); it != mSettingsList.end(); ++it) {
    ss << (*it).argNameShort << "\t" << (*it).argName << "\t" << (*it).helpText << std::endl;
  }

  return ss.str();
}

ParameterParser::Mode ParameterParser::getMode() const {
  std::string modeStr = std::string("");
  Mode mode = Mode::cGenerate;

  if (mArgs.count("-m")) {
    modeStr = mArgs.at("-m");
  }
  if (mArgs.count("--mode")) {
    modeStr = mArgs.at("--mode");
  }

  if (modeStr == "calibrate") {
    mode = Mode::cCalibrate;
  } else if (modeStr == "generate") {
    mode = Mode::cGenerate;
  } else {
    throw std::runtime_error("Invalid mode parameter given");
  }

  return mode;
}

std::string ParameterParser::getSatellite() const {
  std::string sat = std::string("");

  if (mArgs.count("-s")) {
    sat = mArgs.at("-s");
  }
  if (mArgs.count("--sat")) {
    sat = mArgs.at("--sat");
  }

  if (sat == "") {
    throw std::runtime_error("No satellite is selected");
  }

  return sat;
}

std::filesystem::path ParameterParser::getImagePath() const {
  std::string path = std::string("");

  if (mArgs.count("-i")) {
    path = mArgs.at("-i");
  }
  if (mArgs.count("--image")) {
    path = mArgs.at("--image");
  }

  return path;
}

std::filesystem::path ParameterParser::outputPath() const {
  std::string path = std::string("");

  if (mArgs.count("-o")) {
    path = mArgs.at("-o");
  }
  if (mArgs.count("--outimage")) {
    path = mArgs.at("--outimage");
  }

  if (path == "") {
    throw std::runtime_error("No output path given");
  }

  return path;
}