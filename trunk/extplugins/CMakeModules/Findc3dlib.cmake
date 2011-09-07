# przygotowanie do szukania
FIND_INIT(C3DLIB c3dlib)
FIND_INCLUDE_PLATFORM_HEADERS(C3DLIB c3dlib)

# szukanie
FIND_STATIC(C3DLIB c3dlib)

list( APPEND FIND_RESULTS C3DLIB)
# skopiowanie
FIND_FINISH(C3DLIB)