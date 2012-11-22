# przygotowanie do szukania
FIND_INIT_CUSTOM_MODULE(NAVLIB "camera/navigation" ${MIS_INCLUDE_ROOT} ${MIS_BUILD_ROOT})

# szukanie
FIND_STATIC(NAVLIB "navlib")

# skopiowanie
FIND_FINISH(NAVLIB)
