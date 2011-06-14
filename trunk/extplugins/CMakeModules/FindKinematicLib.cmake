# przygotowanie do szukania
FIND_INIT(KINEMATICLIB kinematiclib)
FIND_INCLUDE_PLATFORM_HEADERS(KINEMATICLIB kinematiclib)

# szukanie
FIND_STATIC(KINEMATICLIB "kinematiclib")

# skopiowanie
FIND_FINISH(KINEMATICLIB)