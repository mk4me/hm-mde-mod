cmake_minimum_required(VERSION 2.8)
project(Plugins)
include("../../CMakeModules/Logic/ProjectUtils.cmake")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CustomCMakeModules;${CMAKE_SOURCE_DIR}/CMakeEDRModules;${CMAKE_MODULE_PATH}")

INITIALIZE_SOLUTION(
	"CORE;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML;C3DLIB"
	"CMakeEDRModules")

if (WIN32)
	GET_FILENAME_COMPONENT(PROGRAM_FILES_DIR_TMP  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\PJWSTK\\EDR;ProgramFilesPath]" ABSOLUTE CACHE)
	file(TO_NATIVE_PATH ${PROGRAM_FILES_DIR_TMP} PROGRAM_FILES_DIR)
	set(PROCEDURAL_CORE_PATH ${PROGRAM_FILES_DIR}/bin CACHE PATH "Binary path.")
	set(PROCEDURAL_CORE_EXE core.exe CACHE STRING "Executable name")
else()
	set(PROCEDURAL_CORE_PATH ${PROJECT_BINARY_DIR}/../edr/bin CACHE PATH "Binary path.")
	set(PROCEDURAL_CORE_EXE old_view CACHE STRING "Executable name")
endif()

set(EDRUTILS_INCLUDE_ROOT "${PROJECT_BINARY_DIR}/../../EDR/trunk/src/edrutils/include" CACHE PATH "Location of edr utils public includes.")
set(EDRUTILS_BUILD_ROOT "${PROJECT_BINARY_DIR}/../edrUtils" CACHE PATH "Location of edrutils project build.")

set(EDR_INCLUDE_ROOT "${PROJECT_BINARY_DIR}/../../EDR/trunk/src/edr/include" CACHE PATH "Location of edr public includes.")
set(EDR_BUILD_ROOT "${PROJECT_BINARY_DIR}/../edr" CACHE PATH "Location of edr project build.")

FINALIZE_SOLUTION()
