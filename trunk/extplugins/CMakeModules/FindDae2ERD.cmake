# przygotowanie do szukania
FIND_INIT(DAE2ERD dae2erd)

# szukanie
FIND_SHARED(DAE2ERD_LIB "dae2Motion_d" "dae2Motion_d")

# skopiowanie
FIND_FINISH(DAE2ERD)

# sprawdzenie
if (DAE2ERD_LIB)
	set(DAE2ERD_FOUND 1)
endif()