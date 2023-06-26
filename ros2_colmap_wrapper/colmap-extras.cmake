find_package(COLMAP REQUIRED)

set(ros2_colmap_wrapper_lib_path ${CMAKE_CURRENT_LIST_DIR}/../../../lib/colmap)
set(ros2_colmap_wrapper_LIBRARIES ${ros2_colmap_wrapper_lib_path}/libcolmap.a
        ${ros2_colmap_wrapper_lib_path}/liblsd.a ${ros2_colmap_wrapper_lib_path}/libsift_gpu.a
        ${ros2_colmap_wrapper_lib_path}/libcolmap_cuda.a
        ${ros2_colmap_wrapper_lib_path}/libpoisson_recon.a libvlfeat.a
        )
#set(ros2-tiny-cuda-nn_INCLUDE_DIR ${tiny-cuda-nn_INCLUDE_DIR})