# przygotowanie do szukania
FIND_INIT(VIDLIB vidlib)

# szukanie
FIND_STATIC(VIDLIB "vidlib")

# skopiowanie
FIND_FINISH(VIDLIB)

if (NOT FFMPEG_FOUND)
	set(VIDLIB_FOUND 0)
endif()