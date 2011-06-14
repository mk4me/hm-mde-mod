# przygotowanie do szukania
FIND_INIT(PREREQUISITES prerequisites)
unset(PREREQUISITES_INCLUDE_DIR CACHE)

# szukanie
if (CMAKE_HOST_WIN32)
	# szukamy biblioteki
	FIND_MODULE(PREREQUISITES_LIBPNG FALSE "libpng<?,13>" )
	FIND_MODULE(PREREQUISITES_ZLIB FALSE "zlib<?,1>" )
else()
	# uzywamy wbudowanego modułu wyszukiwania
	FIND_MODULE(PREREQUISITES_LIBPNG TRUE "libpng<?,13>" )
	FIND_MODULE(PREREQUISITES_ZLIB TRUE "z<?,lib><?,1>" )
endif()

# skopiowanie
FIND_FINISH(PREREQUISITES)
