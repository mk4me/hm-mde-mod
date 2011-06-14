# przygotowanie do szukania
FIND_INIT(WSDLPULL wsdlpull)


# szukanie
FIND_STATIC(WSDLPULL_LIB "wsdlpull")

# skopiowanie
FIND_FINISH(WSDLPULL)

# sprawdzenie
if (WSDLPULL_LIB_FOUND)
	set(WSDLPULL_FOUND 1)
endif()
