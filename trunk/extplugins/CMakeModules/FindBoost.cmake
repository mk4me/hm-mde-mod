# przygotowanie do szukania
FIND_INIT(BOOST boost)

# szukanie
FIND_STATIC_EXT(BOOST_SYSTEM "boost_system-vc100<-mt,>-1_50" "boost_system-vc100<-mt,><-gd,-d>-1_50")
FIND_STATIC_EXT(BOOST_FILESYSTEM "boost_filesystem-vc100<-mt,>-1_50" "boost_filesystem-vc100<-mt,><-gd,-d>-1_50")
# koniec
FIND_FINISH(BOOST)

# Wy³¹czamy automatyczne linkowanie boosta
add_definitions(-DBOOST_ALL_NO_LIB)

# sprawdzenie
if (BOOST_SYSTEM_FOUND AND BOOST_FILESYSTEM_FOUND)
	set(BOOST_FOUND 1)

endif()