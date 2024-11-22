
add_executable(openMVG_sample_cameras_fisheyeClip image_fisheye_clip.cpp)
target_link_libraries(openMVG_sample_cameras_fisheyeClip
  openMVG_geometry
  openMVG_image
  openMVG_multiview
  ${STLPLUS_LIBRARY})
set_property(TARGET openMVG_sample_cameras_fisheyeClip PROPERTY FOLDER OpenMVG/Samples)
