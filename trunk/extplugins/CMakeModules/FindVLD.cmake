# przygotowanie do szukania
FIND_INIT(VLD vld)

# szukanie
FIND_SHARED(VLD_LIBCORE "vld" "vld")
FIND_MODULE(VLD_DBGHELP "dbghelp")
FIND_MODULE(VLD_MSVCR90 "msvcr90")

# skopiowanie
FIND_FINISH(VLD)

# sprawdzenie
if (FIND_RESULTS_LOGICAL_AND)
	set(VLD_FOUND 1)
endif()