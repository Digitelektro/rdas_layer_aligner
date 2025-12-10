#include <filesystem>
#include <opencv2/core/mat.hpp>
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
// #include "tps.h"

static constexpr int CH2_OFFSET = 3606;
static constexpr int CH3_OFFSET = 1804;


void valuesFromEuclidean(const cv::Mat& warp);
std::tuple<cv::Mat, cv::Mat> calculateMatrix(cv::Mat reference, cv::Mat ch1, cv::Mat ch2);
std::tuple<cv::Mat, cv::Mat> loadMatrix(std::filesystem::path ch1Config, std::filesystem::path ch2Config);
void valuesFromAffine(const cv::Mat& warp, const std::string& prefix);

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

/**
 * @brief Finds affines between ch1 and ch2/3, saves them as affineIndex
 *
 * @param config Affine value config file
 * @param affineIndex Index to save affine values as
 * @param input 3 input channels to find the affines for
 */
void findAffines(Config config, std::string affineIndex, std::vector<cv::Mat> input) {

  // Vertical offsets between channels on MSU-GS, consistent between all satellites
  auto ch2Shift = ImageProc::shiftImageMatrix(0, CH2_OFFSET);
  auto ch3Shift = ImageProc::shiftImageMatrix(0, CH3_OFFSET);

  std::cout << "Calibration started for " << affineIndex << std::endl;

  // - Alignment and affine parameter finding for channel 2
  cv::Mat ch2WarpMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);
  auto ch2thread = std::thread([&]() {
    cv::Mat ch2Aligned;
    cv::warpAffine(input[1], ch2Aligned, ch2Shift, input[1].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    ch2WarpMatrix = ImageProc::findMatrix(input[0], ch2Aligned);
  });

  // - Alignment and affine parameter finding for channel 3
  cv::Mat ch3WarpMatrix = (cv::Mat_<float>(2, 3) << 1, 0, 0, 0, 1, 0);
  auto ch3thread = std::thread([&]() {
    cv::Mat ch3Aligned;
    cv::warpAffine(input[2], ch3Aligned, ch3Shift, input[2].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
    ch3WarpMatrix = ImageProc::findMatrix(input[0], ch3Aligned);
  });

  ch2thread.join();
  ch3thread.join();

  ch2WarpMatrix.at<float>(0, 2) += ch2Shift.at<float>(0, 2); // Horizontal
  ch2WarpMatrix.at<float>(1, 2) += ch2Shift.at<float>(1, 2); // Vertical

  ch3WarpMatrix.at<float>(0, 2) += ch3Shift.at<float>(0, 2); // Horizontal
  ch3WarpMatrix.at<float>(1, 2) += ch3Shift.at<float>(1, 2); // Vertical

  std::cout << "Calibration done for " << affineIndex << std::endl;

  valuesFromAffine(ch2WarpMatrix, "CH2 ");
  valuesFromAffine(ch3WarpMatrix, "CH3 ");

  config.setTransfromMatrix(ch2WarpMatrix, affineIndex, "CH2");
  config.setTransfromMatrix(ch3WarpMatrix, affineIndex, "CH3");
}

/**
 * @brief Writes out an NC from a vector of RGB channels
 *
 * @param channels R, G, B channels for the NC
 * @param outputPath File to write the NC to
 */
void writeNaturalColor(std::vector<cv::UMat> channels, std::filesystem::path outputPath) {
  cv::UMat RGB321Image;
  cv::merge(channels, RGB321Image);

  // Hue shift red +60°, yellow -70°, overlap 40% - this is the NC
  std::vector<ImageProc::HueShiftRule> rules = {
    // RED
    //{175, 5, 30},
    // YELLOW
    {20, 45, -35},
  };
  auto hueCorrectedImg = ImageProc::hue(RGB321Image, rules, 40.0);

  // For now just write out the raw 321
  cv::imwrite(outputPath, hueCorrectedImg);
}

/**
 * @brief Aligns the input array of channels, resulting in three uniform VIS channels
 *
 * @param config Affine value config file
 * @param affineIndex Index of the affine values
 * @param input Vector of 3 raw input channels
 * @param output Vector of 3 aligned output channels
 */
void alignChannels(Config config, std::string affineIndex, std::vector<cv::Mat> input, std::vector<cv::UMat>& output) {

  cv::Mat warpCh2 = config.getTransformMatrix(affineIndex, "CH2");
  cv::Mat warpCh3 = config.getTransformMatrix(affineIndex, "CH3");

  valuesFromAffine(warpCh2, "CH2 ");
  valuesFromAffine(warpCh3, "CH3 ");

  cv::UMat ch2Aligned;
  cv::UMat ch3Aligned;
  cv::warpAffine(input[1], ch2Aligned, warpCh2, input[1].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);
  cv::warpAffine(input[2], ch3Aligned, warpCh3, input[2].size(), cv::INTER_LINEAR + cv::WARP_INVERSE_MAP);

  // Channel 1 is left unchanged, calibration ramp will stay on it
  output.push_back(input[0].getUMat(cv::ACCESS_READ));
  output.push_back(ch2Aligned);
  output.push_back(ch3Aligned);

  // Crops away useless empty bottom, this is present because of the shifted channels
  cv::Rect cropSize(0, 0, output[0].cols, output[0].rows - 3606);
  output[0] = output[0](cropSize);
  output[1] = output[1](cropSize);
  output[2] = output[2](cropSize);
}


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

  // - Calibration mode -
  // Gets the affines for both channels (2 & 3) to overlay them over ch1 with the best overlap

  if (parameters.getMode() == ParameterParser::Mode::cCalibrate) {
    auto RDASDirectory = parameters.getRDASDirectory();

    if (!std::filesystem::exists(RDASDirectory / "MSUGS_VIS1") || !std::filesystem::exists(RDASDirectory / "MSUGS_VIS2")) {
      throw std::runtime_error("You didn't point to a live-decoded RDAS directory! No MSUGS-VIS1 and/or MSUGS-VIS2 folders were found");
    }

    // VIS1
    std::vector<cv::Mat> VIS1_channels;

    cv::Mat VIS1_ch1 = cv::imread(RDASDirectory / "MSUGS_VIS1" / "MSUGS-VIS-1.png", cv::IMREAD_GRAYSCALE); // R
    cv::Mat VIS1_ch2 = cv::imread(RDASDirectory / "MSUGS_VIS1" / "MSUGS-VIS-2.png", cv::IMREAD_GRAYSCALE); // G
    cv::Mat VIS1_ch3 = cv::imread(RDASDirectory / "MSUGS_VIS1" / "MSUGS-VIS-3.png", cv::IMREAD_GRAYSCALE); // B

    VIS1_channels.push_back(VIS1_ch1);
    VIS1_channels.push_back(VIS1_ch2);
    VIS1_channels.push_back(VIS1_ch3);

    findAffines(config, parameters.getSatellite() + "VIS1", VIS1_channels);

    // VIS2
    std::vector<cv::Mat> VIS2_channels;

    cv::Mat VIS2_ch1 = cv::imread(RDASDirectory / "MSUGS_VIS2" / "MSUGS-VIS-1.png", cv::IMREAD_GRAYSCALE); // R
    cv::Mat VIS2_ch2 = cv::imread(RDASDirectory / "MSUGS_VIS2" / "MSUGS-VIS-2.png", cv::IMREAD_GRAYSCALE); // G
    cv::Mat VIS2_ch3 = cv::imread(RDASDirectory / "MSUGS_VIS2" / "MSUGS-VIS-3.png", cv::IMREAD_GRAYSCALE); // B

    VIS2_channels.push_back(VIS2_ch1);
    VIS2_channels.push_back(VIS2_ch2);
    VIS2_channels.push_back(VIS2_ch3);

    findAffines(config, parameters.getSatellite() + "VIS2", VIS2_channels);

    std::cout << "Affine values for both channels have been saved!" << std::endl;

    return 0;
  }

  // - Generation mode -
  // Aligns both VIS channels, outputs products to dir

  auto RDASDirectory = parameters.getRDASDirectory();
  auto NCPath = parameters.outputPath();
  auto VIS1_directory = RDASDirectory / "MSUGS_VIS1";
  auto VIS2_directory = RDASDirectory / "MSUGS_VIS2";

  if (!std::filesystem::exists(VIS1_directory) || !std::filesystem::exists(VIS2_directory)) {
    throw std::runtime_error("You didn't point to a live-decoded RDAS directory! No MSUGS-VIS1 and/or MSUGS-VIS2 folders were found");
  }

  {
    // Load VIS1 channels in
    std::vector<cv::Mat> VIS1_channels;
    cv::Mat ch1 = cv::imread(VIS1_directory / "MSUGS-VIS-1.png", cv::IMREAD_GRAYSCALE); // R
    cv::Mat ch2 = cv::imread(VIS1_directory / "MSUGS-VIS-2.png", cv::IMREAD_GRAYSCALE); // G
    cv::Mat ch3 = cv::imread(VIS1_directory / "MSUGS-VIS-3.png", cv::IMREAD_GRAYSCALE); // B

    VIS1_channels.push_back(ch1);
    VIS1_channels.push_back(ch2);
    VIS1_channels.push_back(ch3);


    std::vector<cv::UMat> VIS1_alignedChannels;

    alignChannels(config, parameters.getSatellite() + "VIS1", VIS1_channels, VIS1_alignedChannels);

    // Save the VIS1 image results
    // TODOREWORK: Only do this with the merged product in the end!

    std::filesystem::path output_directory = RDASDirectory / "MSUGS_VIS-1_Aligned";

    if (!std::filesystem::exists(output_directory))
      std::filesystem::create_directory(output_directory);

    cv::imwrite(output_directory / "MSUGS-VIS-1.png", VIS1_alignedChannels[0]);
    cv::imwrite(output_directory / "MSUGS-VIS-2.png", VIS1_alignedChannels[1]);
    cv::imwrite(output_directory / "MSUGS-VIS-3.png", VIS1_alignedChannels[2]);

    // Product.cbor is identical
    std::filesystem::path cborTarget = output_directory / "product.cbor";
    if (std::filesystem::exists(cborTarget))
      std::filesystem::remove(cborTarget);

    std::filesystem::copy(VIS1_directory / "product.cbor", cborTarget);

    // Writes an NC if path was supplied
    // TODOREWORK: Write out the NC from merged channels and not just one
    if (NCPath != "") {
      writeNaturalColor(VIS1_alignedChannels, NCPath);
    }
  }


  // Same logic for VIS2...
  {
    // Load VIS2 channels in
    std::vector<cv::Mat> VIS2_channels;
    cv::Mat VIS2_ch1 = cv::imread(VIS2_directory / "MSUGS-VIS-1.png", cv::IMREAD_GRAYSCALE); // R
    cv::Mat VIS2_ch2 = cv::imread(VIS2_directory / "MSUGS-VIS-2.png", cv::IMREAD_GRAYSCALE); // G
    cv::Mat VIS2_ch3 = cv::imread(VIS2_directory / "MSUGS-VIS-3.png", cv::IMREAD_GRAYSCALE); // B

    VIS2_channels.push_back(VIS2_ch1);
    VIS2_channels.push_back(VIS2_ch2);
    VIS2_channels.push_back(VIS2_ch3);


    std::vector<cv::UMat> VIS2_alignedChannels;

    alignChannels(config, parameters.getSatellite() + "VIS2", VIS2_channels, VIS2_alignedChannels);

    // Save the VIS2 image results
    // TODOREWORK: Only do this with the merged product in the end!
    {
      auto output_directory = RDASDirectory / "MSUGS_VIS-2_Aligned";

      if (!std::filesystem::exists(output_directory))
        std::filesystem::create_directory(output_directory);

      cv::imwrite(output_directory / "MSUGS-VIS-1.png", VIS2_alignedChannels[0]);
      cv::imwrite(output_directory / "MSUGS-VIS-2.png", VIS2_alignedChannels[1]);
      cv::imwrite(output_directory / "MSUGS-VIS-3.png", VIS2_alignedChannels[2]);

      // Product.cbor is identical
      std::filesystem::path cborTarget = output_directory / "product.cbor";
      if (std::filesystem::exists(cborTarget))
        std::filesystem::remove(cborTarget);

      std::filesystem::copy(VIS1_directory / "product.cbor", cborTarget);
    }
  }

  // TODOREWORK: Logic to merge MSUGS VIS2 can be added here
  // When done, remove save handling for the individual sides and just copy it once for the merged images


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
