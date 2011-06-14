###############################################################################

 # Tworzy w output listê przeprowadzaj¹c kombinacj¹ list names, versions i suffixes
 # zgodnie z wzorem pattern (znaczniki to odpowiednio <n>, <v> i <s>)
 macro(GENERATE_NAMES pattern names versions suffixes output)
	# kopiujemy i dodajemy dummy wartoœæ, aby móc obs³u¿yæ pêtlê
	set(_names ${names})
	set(_versions ${versions})
	set(_suffixes ${suffixes})
	foreach (it _names _versions _suffixes)
		if ("${${it}}" STREQUAL "")
			set(${it} "?")
		endif()
	endforeach()
	foreach (name ${_names})
		# zerowanie jeœli potrzebne
		if ("${name}" STREQUAL "?")
			set(name  "")
		endif()
		string(REPLACE "<n>" "${name}" _named "${pattern}")
		foreach (version ${_versions})
			# zerowanie wersji jeœli jest potrzebne
			if ("${version}" STREQUAL "?")
				set(version "")
			endif()	
			string(REPLACE "<v>" "${version}" _versioned "${_named}")
			foreach (suffix ${_suffixes})
				# zerowanie jeœli potrzebne
				if ("${suffix}" STREQUAL "?")
					set(suffix "")
				endif()
				string(REPLACE "<s>" "${suffix}" _suffixed "${_versioned}")
				list(APPEND ${output} "${_suffixed}")
				#message("${pattern}")
			endforeach()		
		endforeach()
	endforeach()
	
endmacro(GENERATE_NAMES)

###############################################################################

# Makro wyszukuje bibliotek statycznych lub plików lib dla wspó³dzielonych bibliotek (windows).
# Zak³ada, ¿e istniej¹ dwie zmienne:
# FIND_DIR_DEBUG Miejsca gdzie szukaæ bibliotek w wersji debug
# FIND_DIR_RELEASE Miejsca gdzie szukaæ bibliotek w wersji release
# Wyjaœnienie: extension u¿ywany jest w sytuacji, gdy
# CMake nie potrafi wyszukaæ biblioteki bez rozszerzenia (np. biblioteki stayczne na Unixie)
# w 99% przypadków jednak nic nie zawiera i w tych wypadkach rozszerzenia brane s¹ z suffixes.
# Rezultaty:
# 	${variable}_LIBRARY_DEBUG lokalizacja biblioteki w wersji debug
#   ${variable}_LIBRARY_RELEASE lokazliacja biblioteki w wersji release 
macro(FIND_LIBS variable names suffixes extension)

	# ustawiamy nowe sufiksy
	set(suffixes_copy ${CMAKE_FIND_LIBRARY_SUFFIXES})
	set(CMAKE_FIND_LIBRARY_SUFFIXES ${suffixes})
	
	foreach (shortname ${names})
		# szukamy wersji debug
		foreach (debugSufix ${FIND_DEBUG_SUFFIXES})
			# najpierw wersje z sufiksem lub bez ale TYLKO w wyznaczonym katalogu
			if (NOT FIND_DISABLE_CUSTOM_DIRECTORY)
				find_library(${variable}_LIBRARY_DEBUG NAMES 
						lib${shortname}${debugSufix}${extension} 
						${shortname}${debugSufix}${extension} 
						lib${shortname}${extension} 
						${shortname}${extension} 
					PATHS ${FIND_DIR_DEBUG}
					DOC "Location of debug version of ${shortname}"
					NO_DEFAULT_PATH
				)
			endif()
			# potem z sufiksem w ca³ym systemie
			find_library(${variable}_LIBRARY_DEBUG NAMES 
					lib${shortname}${debugSufix}${extension} 
					${shortname}${debugSufix}${extension} 
				DOC "Location of debug version of ${shortname}"
			)
		endforeach()
	
		if (NOT FIND_DISABLE_CUSTOM_DIRECTORY)
			# szukamy wersji release, najpierw w wyznaczonym miejscu
			find_library(${variable}_LIBRARY_RELEASE
				NAMES ${shortname}${extension} lib${shortname}${extension} 
				PATHS ${FIND_DIR_RELEASE}
				DOC "Location of release version of ${shortname}"
				NO_DEFAULT_PATH
			)
		endif()
		# potem w ca³ym systemie
		find_library(${variable}_LIBRARY_RELEASE
			NAMES ${shortname}${extension} lib${shortname}${extension}
			DOC "Location of release version of ${shortname}"
		)
	endforeach()

	# przywracamy sufiksy
	set(CMAKE_FIND_LIBRARY_SUFFIXES ${suffixes_copy})

endmacro(FIND_LIBS)

###############################################################################

# Makro wyszukuje biblioteki dll (dla Windows)
# Zak³ada, ¿e istniej¹ dwie zmienne:
# FIND_DIR_DEBUG Miejsca gdzie szukaæ bibliotek w wersji debug
# FIND_DIR_RELEASE Miejsca gdzie szukaæ bibliotek w wersji release
# Rezultaty:
# 	${variable}_LIBRARY_DEBUG_DLL lokalizacja biblioteki w wersji debug
#   ${variable}_LIBRARY_RELEASE_DLL lokazliacja biblioteki w wersji release 
macro(FIND_DLLS variable names)

	if (CMAKE_HOST_WIN32)		
		
		# dekorujemy nazwy i zapisujemy do list
		set(find_dlls_list)
		set(find_dlls_list_debug)
		foreach (value ${names})
			list(APPEND find_dlls_list ${value}.dll)
			list(APPEND find_dlls_list_debug ${value}d.dll)
		endforeach()
	
		# szukamy wersji debug
		if (NOT FIND_DISABLE_CUSTOM_DIRECTORY)
			find_file(${variable}_LIBRARY_DEBUG_DLL
				NAMES ${find_dlls_list_debug} ${find_dlls_list}
				PATHS ${FIND_DIR_DEBUG}
				DOC "Location of debug version of ${variable}"
				NO_DEFAULT_PATH
			)
		endif()
		find_file(${variable}_LIBRARY_DEBUG_DLL
			NAMES ${find_dlls_list_debug}
			DOC "Location of debug version of ${variable}"
		)
			
		# szukamy wersji release
		if (NOT FIND_DISABLE_CUSTOM_DIRECTORY)
			find_file(${variable}_LIBRARY_RELEASE_DLL
				NAMES ${find_dlls_list}
				PATHS ${FIND_DIR_RELEASE}
				DOC "Location of release version of ${variable}"
				NO_DEFAULT_PATH
			)
		endif()
		find_file(${variable}_LIBRARY_RELEASE_DLL
			NAMES ${find_dlls_list}
			DOC "Location of release version of ${variable}"
		)
	else (CMAKE_HOST_WIN32)
		FIND_MESSAGE(ERROR "How come this macro is called?")
	endif (CMAKE_HOST_WIN32)
		
endmacro(FIND_DLLS)
