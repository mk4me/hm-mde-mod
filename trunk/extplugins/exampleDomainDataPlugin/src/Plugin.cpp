/********************************************************************
    created:  2011/06/21
    created:  21:6:2011   11:45
    filename: Plugin.h
    author:   Mateusz Janiak
    
    purpose:  Przyk�adowy plugin operujacy na danych domenowych
              w tym wypadku jest to ScalarChannel. Pokazano jak zaimplementowa�
              element przetwarzaj�cy, jak opisac jego wej�cia i wyj�cia,
              jak pobiera� i �adowa� dane podczas przetwarzania. Dodatkowo
              pokazano jak definiowa� Plugin za pomoc� dostarczonych makrodefinicji.
*********************************************************************/
#ifndef HEADER_GUARD___PLUGIN_H__
#define HEADER_GUARD___PLUGIN_H__

#include <core/Plugin.h>
#include "exampleScalarChannelProcessor.h"

//rozpocz�scie definicji pluginu - podajemy jego nazwe i unikalny identyfikator
CORE_PLUGIN_BEGIN("ExampleDomainDataPlugin", UID::GenerateUniqueID("{07F0084C-B1EF-4D2F-8281-785D5EA5086F}"))
//dodajemy element przetwarzajacy - klase tego elementu, musi on mie� domy�lny konstruktor
CORE_PLUGIN_ADD_DATA_PROCESSOR(ExampleScalarChannelProcessor)
//konczymy definicje pluginu
CORE_PLUGIN_END

#endif  //  HEADER_GUARD___PLUGIN_H__
