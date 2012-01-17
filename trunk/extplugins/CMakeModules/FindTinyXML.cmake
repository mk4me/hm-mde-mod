# przygotowanie do szukania
FIND_INIT(TINYXML tinyxml)

# szukanie
FIND_STATIC_EXT(TINYXML "tinyxmlSTL" "tinyxmldSTL")

# koniec
FIND_FINISH(TINYXML)

# Wy³¹czamy automatyczne linkowanie boosta
add_definitions(-DTIXML_USE_STL)