# przygotowanie do szukania
FIND_INIT(QWT qwt)

# szukanie
FIND_STATIC(QWT "qwt")

set(QWT_CUSTOM_COMPILER_DEFINITIONS "/D \"QWT_DLL\"")

# skopiowanie
FIND_FINISH(QWT)

if (NOT QT_FOUND)
	set(QWT_FOUND 0)
else()
	LIST(APPEND QWT_INCLUDE_DIR "${QT_INCLUDE_DIR}/Qt")
endif()
