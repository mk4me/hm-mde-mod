cmake_minimum_required(VERSION 2.8)
project(Plugins)
include("../../CMakeModules/Logic/ProjectUtils.cmake")

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CustomCMakeModules;${CMAKE_MODULE_PATH}")

INITIALIZE_SOLUTION("CORE;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML;C3DLIB")

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






# cmake_minimum_required(VERSION 2.8) 
# project (Plugins)

# # dodatkowe modu³y
# set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}")
# include(MotionFindUtils)
# include(MotionTargetUtils)
# set(CMAKE_USE_RELATIVE_PATHS TRUE)

# #---------------------------------------------------
# # blok definicji dla CMake'a

# set(PROJECT_LIBRARIES_ROOT "${PROJECT_SOURCE_DIR}/../lib" CACHE PATH "Location of libraries.")
# set(PROJECT_LIBRARIES_INCLUDE_ROOT "${PROJECT_SOURCE_DIR}/../include" CACHE PATH "Location of includes.")
# set(PROJECT_INCLUDE_ROOT "${PROJECT_SOURCE_DIR}/.." CACHE PATH "Location of includes.")
# set(PROJECT_ROOT "${PROJECT_SOURCE_DIR}")
# set(PROJECT_BUILD_ROOT "${PROJECT_BINARY_DIR}")
# #set(INSTALLED_APPLICATION_DIR "${PROJECT_EXEC_DIR}" CACHE PATH "Location of application")
# set(INSTALLED_APPLICATION_DIR "${CMAKE_INSTALL_PREFIX}" CACHE PATH "Location of application")

# # Blok definicje zale¿ne od platformy
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
# if(WIN32)
	# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	# set(PROJECT_LIBRARIES_PLATFORM "win32" CACHE STRING "Platform")
	# add_definitions(-D__WIN32__)
# elseif(UNIX)
	# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/lib")
	# set(PROJECT_LIBRARIES_PLATFORM "linux" CACHE STRING "Platform")
	# add_definitions(-D__UNIX__)
# else()
	# message(FATAL_ERROR "Platform not supported.")
# endif()

# set(PROJECT_LIBRARIES_ROOT_DEBUG "${PROJECT_LIBRARIES_ROOT}/${PROJECT_LIBRARIES_PLATFORM}/debug" CACHE PATH "Location of debug libraries")
# set(PROJECT_LIBRARIES_ROOT_RELEASE "${PROJECT_LIBRARIES_ROOT}/${PROJECT_LIBRARIES_PLATFORM}/release" CACHE PATH "Location of release libraries")

# # tylko debug/release - nie dzia³a!
# # set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Typy konfiguracji" FORCE)

# #---------------------------------------------------
# # opcje

# option(PROJECT_VEBOSE_CONFIG "Print verbose info?" OFF)
# set(FIND_VERBOSE ${PROJECT_VEBOSE_CONFIG})
# set(TARGET_VERBOSE ${PROJECT_VEBOSE_CONFIG})

# # Opcje konfiguracji
# CONFIG_OPTION(CORE_DISABLE_LOGGING "Disable logging?" OFF)
# CONFIG_OPTION(CORE_ENABLE_LOG4CXX "Enable logging?" ON)
# CONFIG_OPTION(CORE_ENABLE_LEAK_DETECTION "Enable leak detection?" OFF)

# # Grupy Ÿróde³
# set(SOURCEGROUP_PRIVATE_HEADERS "Header files" CACHE STRING "Filter for private headers.")
# set(SOURCEGROUP_SOURCES "Source files" CACHE STRING "Filter for sources.")
# set(SOURCEGROUP_PUBLIC_HEADERS "Header files" CACHE STRING "Filter for public headers.")
# set(SOURCEGROUP_UI "UI" CACHE STRING "Filter for .ui files and generated headers.")

# #---------------------------------------------------
# # wyszukanie potrzebnych bibliotek

# # konfiguracja modu³u wyszukuj¹cego
# set(FIND_LIBRARIES_INCLUDE_ROOT ${PROJECT_LIBRARIES_INCLUDE_ROOT})
# set(FIND_LIBRARIES_ROOT ${PROJECT_LIBRARIES_ROOT})
# set(FIND_LIBRARIES_ROOT_DEBUG ${PROJECT_LIBRARIES_ROOT_DEBUG})
# set(FIND_LIBRARIES_ROOT_RELEASE ${PROJECT_LIBRARIES_ROOT_RELEASE})
# set(FIND_PLATFORM ${PROJECT_LIBRARIES_PLATFORM})

# set(PROCEDURAL_TEMPLATES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/Templates")
# set(PROCEDURAL_WORK_DIR ${INSTALLED_APPLICATION_DIR})

# # Szukamy bibliotek
# find_package(Boost)
# find_package(OSG)
# if(WIN32)
	# find_package(Stdint)
# endif()
# find_package(Qt)
# find_package(QWT)
# find_package(TinyXML)
# set(FFMPEG_ONLY_MODULES ON)
# find_package(FFmpeg)
# find_package(VidLib)
# find_package(cURL)
# find_package(wsdlpull)
# find_package(Utils)
# find_package(osgui)
# find_package(KinematicLib)
# find_package(c3dlib)
# find_package(webserviceslib)
# find_package(qthtmlgenlib)
# find_package(TimelineLib)
# find_package(plugins)
# find_package(core)
# find_package(ODE)
# find_package(SQLITE3)
# #find_package(dfmlib)
# #find_package(vdfmlib)
# #find_package(TESTNODEFACTORY)

# # Biblioteczne wyszukanie OGL;
# find_package(OpenGL)
# set(OPENGL_LIBRARIES_PROXY ${OPENGL_LIBRARIES})
# set(OPENGL_LIBRARIES OPENGL_LIBRARIES_PROXY)

# #find_package(Prerequisites)

# #---------------------------------------------------

# # obs³uga modu³ów (.dll/.so)
# option(PROJECT_COPY_MODULES "Copy runtime modules into bin folder?" OFF)
# if(PROJECT_COPY_MODULES)
	# message("Copying modules")
	# FIND_HANDLE_MODULES(PROJECT_COPY_MODULES)
	# message("Copying finished. You should turn off option PROJECT_COPY_MODULES.")
# endif()

# set(PROJECT_REBUILD_DEPENDENCIES_DST "${PROJECT_SOURCE_DIR}/../../" CACHE PATH "Location of rebuilt dependencies structure")
# option(PROJECT_REBUILD_DEPENDENCIES "Rebuild dependencies?" OFF)
# if(PROJECT_REBUILD_DEPENDENCIES)
	# message("Rebuiling dependencies structure in ${PROJECT_REBUILD_DEPENDENCIES_DST}")
	# FIND_REBUILD_DEPENDENCIES("${PROJECT_REBUILD_DEPENDENCIES_DST}")
	# message("Rebuiling dependencies finished. You should turn off option PROJECT_REBUILD_DEPENDENCIES.")
# endif()

# #---------------------------------------------------
# # Projekty sk³adaj¹ce siê z podprojektów dodajamy makrem ADD_PROJECTS,
# # natiomast pojedyncze - ADD_PROJECT
# # ADD_PROJECT(template [{zalezne_projekty}])
# # ADD_PROJECTS(template)

# ADD_PROJECT(plugin1 "CORE;PLUGINS;BOOST;UTILS;OSG;QT;")
# ADD_PROJECT(plugin2 "CORE;PLUGINS;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML")
# ADD_PROJECT(exampleWorkflowPlugin "CORE;PLUGINS;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML")
# ADD_PROJECT(exampleDomainDataPlugin "CORE;PLUGINS;BOOST;UTILS;OSG;QT;KINEMATICLIB;OSGUI;TINYXML;C3DLIB")
# ADD_PROJECT(c3dMarkers "CORE;PLUGINS;BOOST;UTILS;OSG;KINEMATICLIB;TINYXML;QT;C3DLIB")
# #---------------------------------------------------