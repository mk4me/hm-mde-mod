project(exampleWorkflowPlugin)
# nazwa
set(TARGET_TARGETNAME
	plugin_exampleWorkflow
)
#add_definitions(-DVIDLIB_LIBRARY)
# UWAGA: poniewa¿ u¿ywamy "source_group", ka¿dy nag³ówek/Ÿród³o musi do którejœ
# trafiæ; w przeciwnym wypadku CMake generuje b³êdny projekt

#------------------------------------------------------------------------------

# nag³ówki prywatne
set(PRIVATE_H
	Plugin.h
	exampleIntParser.h
	exampleIntProcessorStatistic.h
	exampleIntProcessorFilter.h
	exampleIntWidgetProcessorFilterConfiguration.h
	exampleIntProcessorFlopSignMul.h
	exampleIntVisualizer.h
	exampleIntVisualizerStatistics.h
)
source_group("${SOURCEGROUP_PRIVATE_HEADERS}" FILES ${PRIVATE_H})

# nag³ówki publiczne
string(REPLACE ${PROJECT_ROOT} ${PROJECT_INCLUDE_ROOT} HEADER_PATH ${PROJECT_SOURCE_DIR})
set (HEADER_PATH ${PROJECT_SOURCE_DIR}/../include)
set(PUBLIC_H
	${HEADER_PATH}/exampleIntStatistics.h
)
source_group("${SOURCEGROUP_PUBLIC_HEADERS}" FILES ${PUBLIC_H})
set(TARGET_H ${PRIVATE_H} ${PUBLIC_H})

#------------------------------------------------------------------------------

# Ÿród³a
set(TARGET_SRC
	Plugin.cpp
	exampleIntParser.cpp
	exampleIntProcessorStatistic.cpp
	exampleIntProcessorFilter.cpp
	exampleIntWidgetProcessorFilterConfiguration.cpp
	exampleIntProcessorFlopSignMul.cpp
	exampleIntVisualizer.cpp
	exampleIntVisualizerStatistics.cpp
)
source_group("${SOURCEGROUP_SOURCES}" FILES ${TARGET_SRC})

# # metadane generowane przez MOC
 SET(TARGET_MOC_H
	exampleIntWidgetProcessorFilterConfiguration.h
	exampleIntVisualizerStatistics.h
 )
QT4_WRAP_CPP(TARGET_MOC_SRC ${TARGET_MOC_H})
source_group("${SOURCEGROUP_SOURCES}" FILES ${TARGET_MOC_SRC})
set(TARGET_SRC ${TARGET_SRC} ${TARGET_MOC_SRC})

#------------------------------------------------------------------------------
# UI
set(TARGET_UI
	ui/exampleIntWidgetProcessorFilterConfiguration.ui
)
QT4_WRAP_UI(TARGET_UI_H ${TARGET_UI})
source_group("${SOURCEGROUP_UI}" FILES ${TARGET_UI_H} ${TARGET_UI})
set(TARGET_H ${TARGET_H} ${TARGET_UI_H} ${TARGET_UI})


# #------------------------------------------------------------------------------
# # RC
# set(TARGET_RC
	# #ui/ConsoleWidget.qrc
# )
# QT4_ADD_RESOURCES(TARGET_RCC_SRC ${TARGET_RC})
# source_group("${SOURCEGROUP_UI}" FILES ${TARGET_RC} ${TARGET_RCC_SRC})
# set(TARGET_SRC ${TARGET_SRC} ${TARGET_RCC_SRC} ${TARGET_RC})

# #------------------------------------------------------------------------------
# # CONFIG
# set(CONFIG_H
	# #config/Config.h
# )
# source_group("${SOURCEGROUP_PRIVATE_HEADERS}\\config" FILES ${CONFIG_H})
# set(TARGET_H ${TARGET_H} ${CONFIG_H})

# set(CONFIG_SRC
	# config/Config.cpp
# )
# source_group("${SOURCEGROUP_SOURCES}\\config" FILES ${CONFIG_SRC})
# set(TARGET_SRC ${TARGET_SRC} ${CONFIG_SRC})

#------------------------------------------------------------------------------
# prekompilowany nag³ówek
set(PRECOMPILED_H PCH.h)
set(PRECOMPILED_SRC PCH.cpp)
SET_PRECOMPILED_HEADER(${PRECOMPILED_H} ${PRECOMPILED_SRC} "${TARGET_SRC}")
source_group("${SOURCEGROUP_PRIVATE_HEADERS}" FILES ${PRECOMPILED_H})
set(TARGET_H ${TARGET_H} ${PRECOMPILED_H})
source_group("${SOURCEGROUP_SOURCES}" FILES ${PRECOMPILED_SRC})
set(TARGET_SRC ${TARGET_SRC} ${PRECOMPILED_SRC})

#------------------------------------------------------------------------------
# tworzenie

# obsluga OpenGL
set(QT_USE_QTOPENGL TRUE) 
include(${QT_USE_FILE})

# kompilacja do dll
add_library(${TARGET_TARGETNAME} SHARED ${TARGET_SRC} ${TARGET_H})

#instalacja
install(FILES ${PUBLIC_H} DESTINATION include/${PROJECT_NAME} COMPONENT ${PROJECT_NAME})
install(TARGETS ${TARGET_TARGETNAME} RUNTIME DESTINATION bin LIBRARY DESTINATION lib/${PROJECT_NAME} ARCHIVE DESTINATION lib/${PROJECT_NAME} COMPONENT ${PROJECT_NAME})

GET_FILENAME_COMPONENT(PLUGINS_DESTINATION_DIR_TMP  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\PJWSTK\\EDR;ApplicationDataPath]" ABSOLUTE CACHE)

file(TO_NATIVE_PATH ${PLUGINS_DESTINATION_DIR_TMP} PLUGINS_DESTINATION_DIR_NATIVE)
message(${PLUGINS_DESTINATION_DIR_NATIVE})
add_custom_command(  TARGET ${TARGET_TARGETNAME}
                     POST_BUILD 
                     COMMAND  echo f | xcopy \"$(TargetPath)\" \"${PLUGINS_DESTINATION_DIR_NATIVE}\\plugins\\${TARGET_TARGETNAME}\\$(TargetFileName)\" /Y)

					

					 # nag³ówki
include_directories(${DEFAULT_PROJECT_INCLUDES})
# biblioteki
target_link_libraries(${TARGET_TARGETNAME} ${DEFAULT_PROJECT_LIBS})	
GET_FILENAME_COMPONENT(PROGRAM_FILES_DIR_TMP  "[HKEY_LOCAL_MACHINE\\SOFTWARE\\PJWSTK\\EDR;ProgramFilesPath]" ABSOLUTE CACHE)
file(TO_NATIVE_PATH ${PROGRAM_FILES_DIR_TMP} PROGRAM_FILES_DIR)
message(${PROGRAM_FILES_DIR_TMP})


procedural_create_vcproj_userfile(${TARGET_TARGETNAME} ${PROGRAM_FILES_DIR})
ON_PROJECT_ADDED(exampleWorkflowPlugin)
