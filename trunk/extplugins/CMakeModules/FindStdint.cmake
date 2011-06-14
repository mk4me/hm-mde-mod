# stdint
if(CMAKE_HOST_WIN32)
	# sami dostarczamy
	set(STDINT_INCLUDE_DIR "${FIND_LIBRARIES_INCLUDE_ROOT}/stdint" CACHE PATH "Location of stdint headers.")
endif(CMAKE_HOST_WIN32)
set(STDINT_FOUND 1)