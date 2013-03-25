cmake_minimum_required(VERSION 2.8)
project(Plugins)
include("../../CMakeModules/Logic/ProjectUtils.cmake")
include("CMakeEDRModules/Logic/Generators.cmake")
include("CMakeEDRModules/Logic/EDRProjectUtils.cmake")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CustomCMakeModules;${CMAKE_SOURCE_DIR}/CMakeEDRModules;${CMAKE_MODULE_PATH}")

INITIALIZE_PLUGINS_SOLUTION(
	"CORE;DFLIB;CORELIB;PLUGIN_VDF;PLUGIN_C3D;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML;C3DLIB"
	"CMakeEDRModules")

set(EDRUTILS_INCLUDE_ROOT "${PROJECT_BINARY_DIR}/../../EDR/trunk/src/edrutils/include" CACHE PATH "Location of edr utils public includes.")
set(EDRUTILS_BUILD_ROOT "${PROJECT_BINARY_DIR}/../edrUtils" CACHE PATH "Location of edrutils project build.")

set(EDR_INCLUDE_ROOT "${PROJECT_BINARY_DIR}/../../EDR/trunk/src/edr/include" CACHE PATH "Location of edr public includes.")
set(EDR_BUILD_ROOT "${PROJECT_BINARY_DIR}/../edr" CACHE PATH "Location of edr project build.")

FINALIZE_PLUGINS_SOLUTION()
