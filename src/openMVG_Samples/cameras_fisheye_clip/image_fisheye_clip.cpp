
// Copyright (c) 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/cameras/Camera_Common.hpp"
#include "openMVG/cameras/Camera_undistort_image.hpp"
#include "openMVG/cameras/Camera_Spherical.hpp"
#include "openMVG/cameras/Camera_Pinhole_Fisheye.hpp"
#include "openMVG/cameras/Camera_Pinhole_Fisheye_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/image/sample.hpp"
#include "openMVG/image/image_container.hpp"
#include "openMVG/system/loggerprogress.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::image;

int main(int argc, char **argv) {
  CmdLine cmd;

  std::string sPath;
  std::string sOutPath;
  // Temp storage for the Brown's distortion model
  std::string suffix = "png";

  cmd.add(make_option('i', sPath, "imadir"));
  cmd.add(make_option('o', sOutPath, "outdir"));
  try {
    if (argc == 1)
      throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch (const std::string &s) {
    std::cerr << "Usage: " << argv[0] << ' ' << "[-i|--imadir - Input path]\n"
              << "[-o|--outdir - path for the undistorted JPG files]\n"
              << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutPath == sPath) {
    std::cerr << "Input and Ouput path are set to the same value" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutPath))
    stlplus::folder_create(sOutPath);

  // fisheye left
  Vec2 c;
  c << 414.9536792684886, 300.4558254416866;
  Vec4 k;
  k << -0.0158716, -0.00253978, -0.000803488, -1.33842e-05;
  double f = 253.35;
  // // fisheye right
  // Vec2 c;
  // c << 387.4686667372588, 310.03953448477324;
  // Vec4 k;
  // k << -0.0176681, -0.00200084, -0.000857367, -3.85742e-06;
  // double f = 252.5;

  std::cout << "Used KB8's distortion model values: \n"
            << "  Distortion center: " << c.transpose() << "\n"
            << "  Distortion coefficients (K1,K2,K3,K4): " << k.transpose()
            << "\n"
            << "  Distortion focal: " << f << std::endl;

  const std::vector<std::string> vec_fileNames =
      stlplus::folder_wildcard(sPath, "*." + suffix, false, true);
  std::cout << "\nLocated " << vec_fileNames.size() << " files in " << sPath
            << " with suffix " << suffix;

  Image<unsigned char> imageGreyIn, imageGreyU;
  Image<RGBColor> fisheye_image, imageRGBU;
  Image<RGBAColor> imageRGBAIn, imageRGBAU;
  auto sampler = image::Sampler2d<image::SamplerLinear>();

  system::LoggerProgress my_progress_bar(vec_fileNames.size());
  for (size_t j = 0; j < vec_fileNames.size(); ++j, ++my_progress_bar) {
    // read the depth
    int w, h, depth;
    std::vector<unsigned char> tmp_vec;
    const std::string sOutFileName = stlplus::create_filespec(
        sOutPath, stlplus::basename_part(vec_fileNames[j]), "png");
    const std::string sInFileName = stlplus::create_filespec(
        sPath, stlplus::filename_part(vec_fileNames[j]));
    const int res = ReadImage(sInFileName.c_str(), &tmp_vec, &w, &h, &depth);

    fisheye_image =
        Eigen::Map<Image<RGBColor>::Base>((RGBColor *)&tmp_vec[0], h, w);

    const Pinhole_Intrinsic_Fisheye fisheye_cam(w, h, f, c(0), c(1), k(0), k(1),
                                                k(2), k(3));

    // Initialize a camera model for each image domain
    // - the pinhole 
    const Pinhole_Intrinsic pinhole_cam(300, 300, 250, 300/2, 300/2);

    const int renderer_image_size = pinhole_cam.h();
    Image<RGBColor> pinhole_image(pinhole_cam.w(), pinhole_cam.h());

    const int image_width = pinhole_image.Width();
    const int image_height = pinhole_image.Height();

    // Use image coordinate in a matrix to use OpenMVG camera bearing vector
    // vectorization
    Mat2X xy_coords(2, static_cast<int>(image_width * image_height));
    for (int y = 0; y < image_height; ++y)
      for (int x = 0; x < image_width; ++x)
        xy_coords.col(x + image_width * y) << x + .5, y + .5;

    // Compute bearing vectors
    auto rot = RotationAroundY(D2R(45.0)); // for left fisheye
    // auto rot = RotationAroundY(D2R(-45.0)); // for right fisheye
    const Mat3X bearing_vectors = rot * pinhole_cam(xy_coords);

// For every pinhole image pixels
#pragma omp parallel for
    for (int it = 0; it < bearing_vectors.cols(); ++it) {
      // Project the bearing vector to the sphere
      const Vec2 fisheye_proj = fisheye_cam.project(bearing_vectors.col(it));
      // and use the corresponding pixel location in the panorama
      const Vec2 xy = xy_coords.col(it);
      if (fisheye_image.Contains(fisheye_proj.y(), fisheye_proj.x()) && bearing_vectors.col(it).z() > 0) {
        pinhole_image(xy.y(), xy.x()) =
            sampler(fisheye_image, fisheye_proj.y(), fisheye_proj.x());
      }
    }
    WriteImage(sOutFileName.c_str(), pinhole_image);
  } // end loop for each file
  return EXIT_SUCCESS;
}
