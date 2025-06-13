# generated from ament_cmake_export_libraries/cmake/template/ament_cmake_export_libraries.cmake.in

set(_exported_libraries "dynosam;/usr/local/lib/libopengv.a;/usr/local/lib/libconfig_utilities.so;/opt/ros/humble/lib/x86_64-linux-gnu/libgtsam.so.4.2.0;/usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_core.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_video.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_alphamat.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_barcode.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_face.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_intensity_transform.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_mcc.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_rapid.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_text.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_wechat_qrcode.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.5.4d;/usr/lib/x86_64-linux-gnu/libtbb.so.12.5;dynosam")
set(_exported_library_names "glog;gflags;png")

# populate dynosam_LIBRARIES
if(NOT _exported_libraries STREQUAL "")
  # loop over libraries, either target names or absolute paths
  list(LENGTH _exported_libraries _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_libraries ${_i} _arg)

    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND dynosam_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'dynosam' passes the build configuration keyword '${_cfg}' as the last exported library")
      endif()
      list(GET _exported_libraries ${_i} _library)
    else()
      # the value is a library without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    if(NOT IS_ABSOLUTE "${_library}")
      # search for library target relative to this CMake file
      set(_lib "NOTFOUND")
      find_library(
        _lib NAMES "${_library}"
        PATHS "${dynosam_DIR}/../../../lib"
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )

      if(NOT _lib)
        # warn about not existing library and ignore it
        message(FATAL_ERROR "Package 'dynosam' exports the library '${_library}' which couldn't be found")
      elseif(NOT IS_ABSOLUTE "${_lib}")
        # the found library must be an absolute path
        message(FATAL_ERROR "Package 'dynosam' found the library '${_library}' at '${_lib}' which is not an absolute path")
      elseif(NOT EXISTS "${_lib}")
        # the found library must exist
        message(FATAL_ERROR "Package 'dynosam' found the library '${_lib}' which doesn't exist")
      else()
        list(APPEND dynosam_LIBRARIES ${_cfg} "${_lib}")
      endif()

    else()
      if(NOT EXISTS "${_library}")
        # the found library must exist
        message(WARNING "Package 'dynosam' exports the library '${_library}' which doesn't exist")
      else()
        list(APPEND dynosam_LIBRARIES ${_cfg} "${_library}")
      endif()
    endif()
  endwhile()
endif()

# find_library() library names with optional LIBRARY_DIRS
# and add the libraries to dynosam_LIBRARIES
if(NOT _exported_library_names STREQUAL "")
  # loop over library names
  # but remember related build configuration keyword if available
  list(LENGTH _exported_library_names _length)
  set(_i 0)
  while(_i LESS _length)
    list(GET _exported_library_names ${_i} _arg)
    # pass linker flags along
    if("${_arg}" MATCHES "^-" AND NOT "${_arg}" MATCHES "^-[l|framework]")
      list(APPEND dynosam_LIBRARIES "${_arg}")
      math(EXPR _i "${_i} + 1")
      continue()
    endif()

    if("${_arg}" MATCHES "^(debug|optimized|general)$")
      # remember build configuration keyword
      # and get following library name
      set(_cfg "${_arg}")
      math(EXPR _i "${_i} + 1")
      if(_i EQUAL _length)
        message(FATAL_ERROR "Package 'dynosam' passes the build configuration keyword '${_cfg}' as the last exported target")
      endif()
      list(GET _exported_library_names ${_i} _library)
    else()
      # the value is a library target without a build configuration keyword
      set(_cfg "")
      set(_library "${_arg}")
    endif()
    math(EXPR _i "${_i} + 1")

    # extract optional LIBRARY_DIRS from library name
    string(REPLACE ":" ";" _library_dirs "${_library}")
    list(GET _library_dirs 0 _library_name)
    list(REMOVE_AT _library_dirs 0)

    set(_lib "NOTFOUND")
    if(NOT _library_dirs)
      # search for library in the common locations
      find_library(
        _lib
        NAMES "${_library_name}"
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING "Package 'dynosam' exports library '${_library_name}' which couldn't be found")
      endif()
    else()
      # search for library in the specified directories
      find_library(
        _lib
        NAMES "${_library_name}"
        PATHS ${_library_dirs}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH
      )
      if(NOT _lib)
        # warn about not existing library and later ignore it
        message(WARNING
          "Package 'dynosam' exports library '${_library_name}' with LIBRARY_DIRS '${_library_dirs}' which couldn't be found")
      endif()
    endif()
    if(_lib)
      list(APPEND dynosam_LIBRARIES ${_cfg} "${_lib}")
    endif()
  endwhile()
endif()

# TODO(dirk-thomas) deduplicate dynosam_LIBRARIES
# while maintaining library order
# as well as build configuration keywords
# as well as linker flags
