###############################################################################
function(procedural_create_vcproj_userfile TARGETNAME WORKING_DIR)
  if (MSVC)
	if (${MSVC_VERSION} EQUAL 1600)
	   configure_file(
	  ${PROCEDURAL_TEMPLATES_DIR}/VisualStudioUserFile.vcxproj.user.in
	  ${CMAKE_CURRENT_BINARY_DIR}/${TARGETNAME}.vcxproj.user
	  @ONLY
	)
	else()
	  configure_file(
	  ${PROCEDURAL_TEMPLATES_DIR}/VisualStudioUserFile.vcproj.user.in
	  ${CMAKE_CURRENT_BINARY_DIR}/${TARGETNAME}.vcproj.user
	  @ONLY
	)
	endif()
  endif ()
 endfunction(procedural_create_vcproj_userfile)
 
# Inicjuje proces wyszukiwania biblioteki.
macro(ON_PROJECT_ADDED name)

	# ustawiamy nazwe dla artefaktow wersji debug tak aby do nazwy na koniec by³o doklejane d, dla release bez zmian
	set_target_properties(${TARGET_TARGETNAME} PROPERTIES DEBUG_POSTFIX "d")

	# flaga aby mozna bylo uzyc projektu w makrach
	set(${name}_FOUND 1 PARENT_SCOPE)
	# kopiujemy includy
	set(${name}_INCLUDE_DIR ${DEFAULT_PROJECT_INCLUDES} PARENT_SCOPE)
	
	# co budujemy?
	get_target_property(BUILD_TYPE ${name} TYPE)
	
	# NA RAZIE WY£ACZONY MECHANIZM PRZEKAZYWANIA BIBLIOTEK!
	# sprawdzamy to po to, ¿eby projekty zale¿ne (klienci bibliotek, testy) 
	# równie¿ widzia³y te same zale¿ne biblioteki, co projekt
	if (BUILD_TYPE STREQUAL "STATIC_LIBRARY" OR BUILD_TYPE STREQUAL "SHARED_LIBRARY")
		# wystarczy dodaæ target, reszt¹ zajmie siê CMake
		# musimy zrobiæ zmienn¹ poœrednicz¹c¹, ¿eby móc iterowaæ po xxx_LIBRARIES
		set(${name}_LIBRARIES_PROXY ${name} PARENT_SCOPE)
		set(${name}_LIBRARIES ${name}_LIBRARIES_PROXY PARENT_SCOPE)
	else()
		# to nie biblioteka wiêc rêcznie dodajemy bilioteki zale¿ne; tutaj wa¿ne:
		# do proxy kopiujemy sta³e reprezentuj¹ce nazwy bibliotek, których wartoœci
		# odwo³uj¹ siê ju¿ do fizycznych plików
		set(${name}_LIBRARIES_PROXY)
		foreach(value ${DEFAULT_PROJECT_DEPENDENCIES})
			list(APPEND ${name}_LIBRARIES_PROXY ${${value}_LIBRARIES})
		endforeach()
		#set(${name}_LIBRARIES ${${name}_LIBRARIES_PROXY} PARENT_SCOPE)
	endif()
	
	
	
endmacro(ON_PROJECT_ADDED)

###############################################################################

# Makro ustawiaj¹ce pewn¹ opcjê konfiguracji.
# Parametry:
#	name	Nazwa makra.
#	info	Tekstowa informacja o opcji.
#	default	ON / OFF
macro(CONFIG_OPTION name info default)
	option(CONFIG_${name} ${info} ${default})
	if (CONFIG_${name})
		set(${name} ${name})
	endif()
endmacro(CONFIG_OPTION)

###############################################################################

# Makro dodaj¹ce projekt
# Parametry
#	name Nazwa projektu (i podkatalogu katalogu)
#	dependencies Lista zale¿noœci
macro(ADD_PROJECT name dependencies)
	set(ADD_PROJECT_FAILED 0)
	set(ADD_PROJECT_MESSAGE "")
	# szukamy wszystkich brakuj¹cych zale¿noœci
	foreach (value ${dependencies})
		if (NOT ${value}_FOUND)
			set(ADD_PROJECT_FAILED 1)
			set(ADD_PROJECT_MESSAGE ${value} ", " ${ADD_PROJECT_MESSAGE})
			TARGET_NOTIFY(${name} "${value} not found")
		endif()
	endforeach()
	# sprawdzamy
	if (ADD_PROJECT_FAILED)
		message(${name} " not included because dependencies are missing: " ${ADD_PROJECT_MESSAGE})
	else()
		#message("${name}: ${dependencies}")
		set (DEFAULT_PROJECT_DEPENDENCIES ${dependencies})
		set (DEFAULT_PROJECT_LIBS)
		set (DEFAULT_PROJECT_INCLUDES "${PROJECT_BINARY_DIR}/${name};${PROJECT_BUILD_ROOT};${PROJECT_INCLUDE_ROOT};${PROJECT_ROOT};${PROJECT_BINARY_DIR}/${name}/src" )
		# kopiujemy dane
		foreach (value ${dependencies})
			#if (NOT ${createDependency})
			#	message("(${name}) RAW ${value} libraries: ${${value}_LIBRARIES}")
			#endif()
			TARGET_NOTIFY(${name} "RAW DEPENDENCY ${value} libraries: ${${value}_LIBRARIES}")
			foreach (value2 ${${value}_LIBRARIES})
				if (NOT ${createDependency})
					#message("(${name})Current library: ${value2} : ${${value2}} : ")
				endif()
				list(APPEND DEFAULT_PROJECT_LIBS ${${value2}})
			endforeach()
			list(APPEND DEFAULT_PROJECT_INCLUDES ${${value}_INCLUDE_DIR})
			# czy zdefiniowano katalog konfiguracyjny?
			if ( ${value}_INCLUDE_CONFIG_DIR )
				list(APPEND DEFAULT_PROJECT_INCLUDES ${${value}_INCLUDE_CONFIG_DIR})
			endif()
		endforeach()

		add_subdirectory(${name})
	endif()
endmacro(ADD_PROJECT)

###############################################################################

macro(ADD_PROJECTS name)

	add_subdirectory(${name})
	
endmacro(ADD_PROJECTS)

###############################################################################

MACRO(SET_PRECOMPILED_HEADER header source sources)
	if(MSVC)
		get_filename_component(_basename ${header} NAME_WE)
		set(_binary "${CMAKE_CURRENT_BINARY_DIR}/${_basename}.pch")
		set_source_files_properties(${source}
			PROPERTIES COMPILE_FLAGS "/Yc\"${header}\" /Fp\"${_binary}\""
			OBJECT_OUTPUTS "${_binary}")
		set_source_files_properties(${sources}
			PROPERTIES COMPILE_FLAGS "/Yu\"${_binary}\" /FI\"${_binary}\" /Fp\"${_binary}\""
			OBJECT_DEPENDS "${_binary}")  
	else()
		add_definitions(-DDISABLE_PRECOMPILED_HEADERS)
	endif()
ENDMACRO(SET_PRECOMPILED_HEADER)

###############################################################################

macro(TARGET_NOTIFY var msg)
	if (TARGET_VERBOSE)
		message(STATUS "TARGET>${var}>${msg}")
	endif()
endmacro()

