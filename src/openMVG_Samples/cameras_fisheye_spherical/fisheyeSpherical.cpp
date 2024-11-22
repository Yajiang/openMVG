
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

int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sPath;
  std::string sOutPath;
  // Temp storage for the Brown's distortion model
  std::string suffix = "png";

  cmd.add( make_option('i', sPath, "imadir") );
  cmd.add( make_option('o', sOutPath, "outdir") );
  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch (const std::string& s) {
      std::cerr << "Usage: " << argv[0] << ' '
      << "[-i|--imadir - Input path]\n"
      << "[-o|--outdir - path for the undistorted JPG files]\n"
      << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sOutPath == sPath)
  {
    std::cerr << "Input and Ouput path are set to the same value" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutPath))
    stlplus::folder_create(sOutPath);

//   # Left Camera calibration and distortion parameters (OpenCV)
// Camera1.fx: 253.74469953582386
// Camera1.fy: 253.00898309533483
// Camera1.cx: 414.9536792684886
// Camera1.cy: 300.4558254416866

// # Kannala-Brandt distortion parameters
// Camera1.k1: -0.0158716
// Camera1.k2: -0.00253978
// Camera1.k3: -0.000803488
// Camera1.k4: -1.33842e-05

  Vec2 c;
  c << 414.9536792684886, 300.4558254416866;
  Vec4 k;
  k << -0.0158716, -0.00253978, -0.000803488, -1.33842e-05;
  double f = 253.35;

  std::cout << "Used Brown's distortion model values: \n"
    << "  Distortion center: " << c.transpose() << "\n"
    << "  Distortion coefficients (K1,K2,K3,K4): "
    << k.transpose() << "\n"
    << "  Distortion focal: " << f << std::endl;

  const std::vector<std::string> vec_fileNames =
    stlplus::folder_wildcard(sPath, "*."+suffix, false, true);
  std::cout << "\nLocated " << vec_fileNames.size() << " files in " << sPath
    << " with suffix " << suffix;

  Image<unsigned char > imageGreyIn, imageGreyU;
  Image<RGBColor> imageRGBIn, imageRGBU;
  Image<RGBAColor> imageRGBAIn, imageRGBAU;

  system::LoggerProgress my_progress_bar( vec_fileNames.size() );
  for (size_t j = 0; j < vec_fileNames.size(); ++j, ++my_progress_bar)
  {
    //read the depth
    int w,h,depth;
    std::vector<unsigned char> tmp_vec;
    const std::string sOutFileName =
      stlplus::create_filespec(sOutPath, stlplus::basename_part(vec_fileNames[j]), "png");
    const std::string sInFileName =
      stlplus::create_filespec(sPath, stlplus::filename_part(vec_fileNames[j]));
    const int res = ReadImage(sInFileName.c_str(), &tmp_vec, &w, &h, &depth);

    const Pinhole_Intrinsic_Fisheye cam(w, h, f, c(0), c(1), k(0), k(1), k(2),
                                     k(3));

    if (res == 1)
    {
      switch (depth)
      {
        case 1: //Greyscale
          {
            imageGreyIn = Eigen::Map<Image<unsigned char>::Base>(&tmp_vec[0], h, w);
            UndistortImageResized(imageGreyIn, &cam, imageGreyU,Image<unsigned char>::Tpixel(0));
            WriteImage(sOutFileName.c_str(), imageGreyU);
            break;
          }
        case 3: //RGB
          {
            imageRGBIn = Eigen::Map<Image<RGBColor>::Base>((RGBColor*) &tmp_vec[0], h, w);
            UndistortImageResized(imageRGBIn, &cam, imageRGBU,Image<RGBColor>::Tpixel(0));
            WriteImage(sOutFileName.c_str(), imageRGBU);
            break;
          }
        case 4: //RGBA
          {
            imageRGBAIn = Eigen::Map<Image<RGBAColor>::Base>((RGBAColor*) &tmp_vec[0], h, w);
            UndistortImageResized(imageRGBAIn, &cam, imageRGBAU,
                                  Image<RGBAColor>::Tpixel(0));
            WriteImage(sOutFileName.c_str(), imageRGBAU);
            break;
          }
      }

    }//end if res==1
    else
    {
      std::cerr << "\nThe image contains " << depth << "layers. This depth is not supported!\n";
    }
  } //end loop for each file
  return EXIT_SUCCESS;
}
