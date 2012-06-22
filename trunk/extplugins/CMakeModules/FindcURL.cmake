# przygotowanie do szukania
FIND_INIT(CURL curl)

# szukanie
FIND_STATIC(CURL "libcurl_imp")

# skopiowanie
FIND_FINISH(CURL)
