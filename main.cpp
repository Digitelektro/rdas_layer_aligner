#include <algorithm>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>

#include "configstorage.h"
#include "imageproc.h"
#include "opencv2/core/ocl.hpp"
#include "parameterparser.h"
#include "tps.h"


void valuesFromEuclidean(const cv::Mat& warp);
std::tuple<cv::Mat, cv::Mat> calculateMatrix(cv::Mat reference, cv::Mat ch1, cv::Mat ch2);
std::tuple<cv::Mat, cv::Mat> loadMatrix(std::filesystem::path ch1Config, std::filesystem::path ch2Config);
void valuesFromAffine(const cv::Mat& warp, const std::string& prefix);

int main(int argc, char** argv) {
  cv::ocl::setUseOpenCL(true);
  if (cv::ocl::useOpenCL()) {
    std::cout << "OpenCL is enabled!" << std::endl;
  } else {
    std::cout << "OpenCL is disabled or not supported." << std::endl;
  }

  ParameterParser parameters;
  parameters.parseArgs(argc, argv);

  Config config("./config.json");

  // CH2 is shifted by 1804
  auto ch2Shift = ImageProc::shiftImageMatrix(0, 1804);
  // CH3 is shifted by 3606
  auto ch3Shift = ImageProc::shiftImageMatrix(0, 3606);

  if (parameters.getMode() == ParameterParser::Mode::cCalibrate) {
    auto imagePath = parameters.getImagePath();
    if (imagePath == "") {
      throw std::runtime_error("Image 321 must be given to be able to calibrate channels");
    }

    cv::Mat rgb = cv::imread(imagePath);
    std::vector<cv::Mat> channels;
    cv::split(rgb, channels);

    std::cout << "Calibration begin" << std::endl;

    cv::Mat ch2WarpMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);
    auto ch2thread = std::thread([&]() {
      cv::Mat ch2Alligned;
      cv::warpAffine(channels[1], ch2Alligned, ch2Shift, channels[1].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
      ch2WarpMatrix = ImageProc::findMatrix(channels[0], ch2Alligned);
    });


    cv::Mat ch3WarpMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);
    auto ch3thread = std::thread([&]() {
      cv::Mat ch3Alligned;
      cv::warpAffine(channels[2], ch3Alligned, ch3Shift, channels[2].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
      ch3WarpMatrix = ImageProc::findMatrix(channels[0], ch3Alligned);
    });

    ch2thread.join();
    ch3thread.join();

    // Add the shifts
    ch2WarpMatrix.at<float>(0, 2) += ch2Shift.at<float>(0, 2);
    ch2WarpMatrix.at<float>(1, 2) += ch2Shift.at<float>(1, 2);
    ch3WarpMatrix.at<float>(0, 2) += ch3Shift.at<float>(0, 2);
    ch3WarpMatrix.at<float>(1, 2) += ch3Shift.at<float>(1, 2);

    std::cout << "Calibration done" << std::endl;
    valuesFromAffine(ch2WarpMatrix, "CH2 ");
    valuesFromAffine(ch3WarpMatrix, "CH3 ");

    config.setTransfromMatrix(ch2WarpMatrix, parameters.getSatellite(), "CH2");
    config.setTransfromMatrix(ch3WarpMatrix, parameters.getSatellite(), "CH3");
  } else {
    auto imagePath = parameters.getImagePath();
    cv::UMat rgb;
    cv::imread(imagePath).copyTo(rgb);
    std::vector<cv::UMat> channels;
    cv::split(rgb, channels);
    auto outputPath = parameters.outputPath();

    auto warpCh2 = config.getTransformMatrix(parameters.getSatellite(), "CH2");
    auto warpCh3 = config.getTransformMatrix(parameters.getSatellite(), "CH3");

    valuesFromAffine(warpCh2, "CH2 ");
    valuesFromAffine(warpCh3, "CH3 ");

    cv::UMat ch2Alligned;
    cv::UMat ch3Alligned;
    cv::warpAffine(channels[1], ch2Alligned, warpCh2, channels[1].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    cv::warpAffine(channels[2], ch3Alligned, warpCh3, channels[2].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

    std::vector<cv::UMat> allignedChannels;
    allignedChannels.push_back(channels[0]);
    allignedChannels.push_back(ch2Alligned);
    allignedChannels.push_back(ch3Alligned);

    cv::UMat outputImage;
    cv::merge(allignedChannels, outputImage);

    cv::imwrite(outputPath, outputImage);
  }
  return 0;

  /*
    cv::UMat rgb;
    cv::imread("merged2.png").copyTo(rgb);
    std::vector<cv::UMat> channels;
    cv::split(rgb, channels);


    cv::Mat warp_matrix_ch1 = ImageProc::shiftImageMatrix(0, -1760);
    cv::Mat warp_matrix_ch2 = ImageProc::shiftImageMatrix(0, -3565);
    // std::tuple<cv::Mat, cv::Mat> warpMatrices{warp_matrix_ch1, warp_matrix_ch2};


    // valuesFromAffine(warp_matrix_ch1, "CH2 ");
    // valuesFromAffine(warp_matrix_ch2, "CH3 ");

    // auto warpMatrices = calculateMatrix(channels[0], channels[1], channels[2]);
    auto warpMatrices = loadMatrix("ch1.config", "ch2.config");


    // Config::saveTransfromMatrix(std::get<0>(warpMatrices), "ch1.config");
    // Config::saveTransfromMatrix(std::get<1>(warpMatrices), "ch2.config");

    valuesFromAffine(std::get<0>(warpMatrices), "CH1 ");
    // valuesFromAffine(std::get<1>(warpMatrices), "CH2 ");

    // Storage for warped image.
    cv::UMat ch1Alligned;
    cv::warpAffine(channels[1], ch1Alligned, std::get<0>(warpMatrices), channels[1].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

    cv::UMat ch2Alligned;
    cv::warpAffine(channels[2], ch2Alligned, std::get<1>(warpMatrices), channels[2].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);


    cv::UMat merged;
    std::vector<cv::UMat> allignedChannels;
    allignedChannels.push_back(channels[0]);
    allignedChannels.push_back(ch1Alligned);
    allignedChannels.push_back(ch2Alligned);
    cv::merge(allignedChannels, merged);

    cv::imwrite("./merged4.png", merged);

    const std::string winName = "Aligned Image";
    cv::namedWindow(winName, cv::WINDOW_NORMAL);
    cv::imshow(winName, merged);
    cv::resizeWindow(winName, {1024, 1024});
    // cv::waitKey();

    // Loop until window closed
    while (true) {
      int key = cv::waitKey(30); // 30 ms delay between checks
      if (cv::getWindowProperty(winName, cv::WND_PROP_VISIBLE) < 1) {
        break; // window closed by user
      }
    }

    cv::destroyAllWindows();

    return 0;
    */
}

std::tuple<cv::Mat, cv::Mat> calculateMatrix(cv::Mat reference, cv::Mat ch1, cv::Mat ch2) {
  cv::Mat warpCh1;
  cv::Mat warpCh2;
  // Run the ECC algorithm. The results are stored in warp_matrix.
  std::thread ch1Thread([&] {
    warpCh1 = ImageProc::findMatrix(reference, ch1);
  });
  std::thread ch2Thread([&] {
    warpCh2 = ImageProc::findMatrix(reference, ch2);
  });
  ch1Thread.join();
  ch2Thread.join();
  return {warpCh1, warpCh2};
}

/*std::tuple<cv::Mat, cv::Mat> loadMatrix(std::filesystem::path ch1Config, std::filesystem::path ch2Config) {
  cv::Mat warpCh1 = Config::loadTransformMatrix(ch1Config);
  cv::Mat warpCh2 = Config::loadTransformMatrix(ch2Config);
  return {warpCh1, warpCh2};
}*/

void valuesFromAffine(const cv::Mat& warp, const std::string& prefix) {
  // Extract affine components
  float a = warp.at<float>(0, 0);
  float b = warp.at<float>(0, 1);
  float c = warp.at<float>(1, 0);
  float d = warp.at<float>(1, 1);
  float dx = warp.at<float>(0, 2);
  float dy = warp.at<float>(1, 2);

  // --- Rotation (degrees) ---
  float rotation_rad = atan2(c, a);
  float rotation_deg = rotation_rad * 180.0 / CV_PI;

  // --- Scale ---
  float scale_x = std::sqrt(a * a + b * b);
  float scale_y = std::sqrt(c * c + d * d);

  // --- Shear (slant) ---
  float shear_rad = atan2(-b, d);
  float shear_deg = shear_rad * 180.0 / CV_PI;

  std::cout << prefix << "Translation: dx=" << dx << ", dy=" << dy << std::endl;
  std::cout << prefix << "Rotation(rad/deg)= " << rotation_rad << "/" << rotation_deg << "°" << std::endl;
  std::cout << prefix << "Scale: x=" << scale_x << ", y=" << scale_y << std::endl;
  std::cout << prefix << "Shear: (rad/deg)= " << shear_rad << "/" << shear_deg << "°" << std::endl;
}

void valuesFromEuclidean(const cv::Mat& warp) {
  float dx = warp.at<float>(0, 2);
  float dy = warp.at<float>(1, 2);

  float cos_theta = warp.at<float>(0, 0);
  float sin_theta = warp.at<float>(1, 0);
  float theta_rad = atan2(sin_theta, cos_theta);
  float theta_deg = theta_rad * 180.0 / CV_PI;

  std::cout << "dx = " << dx << "  dy = " << dy << "  rotation(rad/deg) = " << theta_rad << "/" << theta_deg << "°" << std::endl;
}

void oldStuff() {
  /// Old stuff, thin plate spline
  /*cv::Mat matchCh12;
  cv::matchTemplate(ch1, ch2, matchCh12, 3);

  cv::namedWindow("Aligned Image", cv::WINDOW_NORMAL);
  cv::imshow("Aligned Image", matchCh12);
  cv::resizeWindow("Aligned Image", {1024, 1024});
  cv::waitKey();


  auto transfomer =
  cv::createThinPlateSplineShapeTransformer2("../kernels/tps.cl");
  transfomer->estimateTransformation(pointsCh1, pointsCh2, matches);
  transfomer->warpImage(ch1, alignedCh1);

  pointsCh2.clear();
  matches.clear();
  for (size_t i = 0; i < matchesCh3Ch2.size(); i++) {
      pointsCh3.push_back(keyPointsCh3[matchesCh3Ch2[i].queryIdx].pt);
      pointsCh2.push_back(keyPointsCh2[matchesCh3Ch2[i].trainIdx].pt);
      matches.push_back(cv::DMatch(i, i, 0));
  }

  transfomer = cv::createThinPlateSplineShapeTransformer2("../kernels/tps.cl");
  transfomer->estimateTransformation(pointsCh3, pointsCh2, matches);
  transfomer->warpImage(ch3, alignedCh3);



  std::vector<cv::Mat> channels = {alignedCh1, ch2, alignedCh3};
  cv::Mat result;
  cv::merge(channels, result);
  cv::imwrite("../MSU-GS/1.jpg", alignedCh1);
  cv::imwrite("../MSU-GS/2.jpg", ch2);
  cv::imwrite("../MSU-GS/3.jpg", alignedCh3);*/
  // cv::imwrite("../MSU-GS/alligned.png", result);
  // cv::imshow("Aligned Image", alignedCh3);
  // cv::waitKey(0);
  /// Old stuff end
}