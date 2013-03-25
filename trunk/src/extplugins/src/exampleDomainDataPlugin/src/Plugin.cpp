/********************************************************************
    created:  2011/06/21
    created:  21:6:2011   11:45
    filename: Plugin.h
    author:   Mateusz Janiak
    
    purpose:  Przyk³adowy plugin operujacy na danych domenowych
              w tym wypadku jest to ScalarChannel. Pokazano jak zaimplementowaæ
              element przetwarzaj¹cy, jak opisac jego wejœcia i wyjœcia,
              jak pobieraæ i ³adowaæ dane podczas przetwarzania. Dodatkowo
              pokazano jak definiowaæ Plugin za pomoc¹ dostarczonych makrodefinicji.
*********************************************************************/
#ifndef HEADER_GUARD___PLUGIN_H__
#define HEADER_GUARD___PLUGIN_H__

#include <corelib/IPlugin.h>
#include "exampleScalarChannelProcessor.h"
#include <plugins/newVdf/IDataFlowProvider.h>

//rozpoczêscie definicji pluginu - podajemy jego nazwe i unikalny identyfikator
CORE_PLUGIN_BEGIN("ExampleDomainDataPlugin", core::UID::GenerateUniqueID("{07F0084C-B1EF-4D2F-8281-785D5EA5086F}"))
// dodajemy element przetwarzajacy - klase tego elementu, musi on mieæ domyœlny konstruktor
// Wszystkie tego typu elementy musz¹ znaleŸæ siê wewn¹trz serwisu. 
VDF_SERVICE_BEGIN(ExampleDomainService, "{C45743E2-4D40-4B73-AE72-91E979FD0E8C}")
	VDF_ADD_DATA_PROCESSOR(ExampleScalarChannelProcessor, core::UID::GenerateUniqueID("{CEBCAE8E-FD4C-4862-BC52-FA287160DC23}"))
VDF_SERVICE_END(ExampleDomainService)

//konczymy definicje pluginu
CORE_PLUGIN_END

#endif  //  HEADER_GUARD___PLUGIN_H__
