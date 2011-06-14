# przygotowanie do szukania
FIND_INIT(DFMLIB dfmlib)
FIND_INCLUDE_PLATFORM_HEADERS(DFMLIB dfmlib)

# szukanie
FIND_STATIC(DFMLIB "dfmlib")

# skopiowanie
FIND_FINISH(DFMLIB)