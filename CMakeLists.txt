cmake_minimum_required(VERSION 2.8.11)

set(CamelCaseName InSituFTCalibration)
set(VARS_PREFIX "INSITU_FT_CALIBRATION")
set(library_target_name insitu-ft-calibration)

project(${CamelCaseName} CXX)

set(${VARS_PREFIX}_MAJOR_VERSION 0)
set(${VARS_PREFIX}_MINOR_VERSION 0)
set(${VARS_PREFIX}_PATCH_VERSION 1)
set(${VARS_PREFIX}_VERSION ${${VARS_PREFIX}_MAJOR_VERSION}.${${VARS_PREFIX}_MINOR_VERSION}.${${VARS_PREFIX}_PATCH_VERSION})

find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})

find_package(Eigen3 REQUIRED)

# Compile and install related commands
add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${CamelCaseName} VARS_PREFIX ${VARS_PREFIX}
                                             VERSION ${${VARS_PREFIX}_VERSION}
                                             COMPATIBILITY SameMajorVersion
                                             TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                             NO_CHECK_REQUIRED_COMPONENTS_MACRO)

include(AddUninstallTarget)

# Testing related commands
option(CREATE_TESTS "Create regression tests using CMake" FALSE)
if(CREATE_TESTS)
   enable_testing()
   add_subdirectory(test)
endif(CREATE_TESTS)
