# przygotowanie do szukania
FIND_INIT(C3D2ERD c3dparser)

# szukanie
FIND_STATIC_EXT(C3D2ERD_LIB "c3dParser" "c3dParser")

#koniec
FIND_FINISH(C3D2ERD)

# sprawdzenie
if (C3D2ERD_LIB)
	set(C3D2ERD_FOUND 1)
endif()