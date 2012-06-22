# przygotowanie do szukania
FIND_INIT(VLD vld)

# szukanie
FIND_STATIC(VLD_LIBCORE "vld")
FIND_MODULE(VLD_DBGHELP "dbghelp")
FIND_MODULE(VLD_MSVCR90 "msvcr90")

# skopiowanie
FIND_FINISH(VLD)

# sprawdzenie
if (FIND_RESULTS_LOGICAL_AND)
	set(VLD_FOUND 1)
endif()