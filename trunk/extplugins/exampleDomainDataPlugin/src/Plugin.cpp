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
#include <core/C3DChannels.h>

// Przyk�ad prostego elementu przetwarzaj�cego. Operuje on na danych typu scalar channel.
// Element prtzetwarzaj�cy neguje dane skalarne - mamy tutaj przyk�ad jak pibiera� dane z wej�cia,
// jak generowa� dane wyj�ciowe i umieszcza� je na wyj�ciu.
class ExampleScalarChannelProcessor : public core::IDataProcessor
{
    //nadanie unikalnego identyfikatora elementowi przetwarzaj�cemu
    //automatycznie dodaje metody getID i getDescription z interfejsu core::IIdentifable
    UNIQUE_ID("{04B67836-E8C8-4E07-A8D9-60E3089D2510}", "ExampleScalarChannelProcessor");

public:
    virtual const std::string & getName() const
    {
        static const std::string name("ExampleScalarChannelProcessor");
        return name;
    }

    virtual ExampleScalarChannelProcessor* createClone() const
    {
        return new ExampleScalarChannelProcessor();
    }

    virtual void process(core::IObjectSource* input, core::IObjectOutput* output)
    {
        //pobieram obiekt z danymi wejsciowymi z wejscia o podanym indeksie
        auto inScalars = input->getObjects(0);

        //sprawdzam czy cos jest na wejsciu
        if(inScalars.empty() == true){
            return;
        }

        //mam wejscie - przygotowuje wiec wyjscie, pobieram obiekt ladujacy dane wyjsciowe do wyjscia o zadanym indeksie
        auto outScalars = output->getObjects(0);

        //iteruje po danych wejsciowych
        for(int i = 0; i < inScalars.size(); i++){
            //przetwarzam dane wejsciowe
            //pobieram jeden element danych wejsciowych
            core::ScalarChannelConstPtr inScalarData = inScalars.getObject(i);
            //kopiuje go - przygotowuje sobie porcje danych wyjsciowych
            core::ScalarChannelPtr outScalarData(new core::ScalarChannel(inScalarData->getSamplesPerSec()));
            outScalarData->setName(inScalarData->getName());
            outScalarData->setXUnit(inScalarData->getXUnit());
            outScalarData->setYUnit(inScalarData->getYUnit());

            //modyfikuje dane ktore b�d� zaraz udostepnia� na wyj�ciu
            for(auto it = inScalarData->begin(); it != inScalarData->end(); it++){
                outScalarData->addPoint( - (*it).value);
            }

            outScalarData->normalize();

            //zapisuj� zmodyfikowane dane do udost�pniena
            outScalars.addObject(outScalarData);
        }
    }

    virtual void getInputInfo(std::vector<InputInfo>& info)
    {
        //opis oczekiwanych danych wejsciowych
        InputInfo input;
        //nazwa wyswietlana wejscia
        input.name = "Scalar";
        //czy wejscie wymagane
        input.required = true;
        //identyfikator typu dancyh akceptowanych na wejsciu
        input.type = typeid(core::ScalarChannel);
        //czy dane beda modyfikowane
        input.modify = true;

        //zaladowanie opisu jednego wejscia do kolekcji opisujacej wszystkie wejscia
        info.push_back(input);
    }

    virtual void getOutputInfo( std::vector<OutputInfo> & output )
    {
        //analogia do opisu wejscia - opis wyscia
        OutputInfo info;
        //opis wejsc od ktorych jest uzaleznione opisywane wejscie
        info.dependentInput.insert(0);
        //nazwa wyswietlana wyjscia
        info.name = "Scalar";
        //identyfikator typu danych oferowanych na wysjciu
        info.type = typeid(core::ScalarChannel);

        output.push_back(info);
    }

    virtual QWidget* getConfigurationWidget()
    {
        return nullptr;
    }

    virtual void reset()
    {

    }
};

//rozpocz�scie definicji pluginu - podajemy jego nazwe i unikalny identyfikator
CORE_PLUGIN_BEGIN("ExampleDomainDataPlugin", UID::GenerateUniqueID("{07F0084C-B1EF-4D2F-8281-785D5EA5086F}"))
//dodajemy element przetwarzajacy - klase tego elementu
CORE_PLUGIN_ADD_DATA_PROCESSOR(ExampleScalarChannelProcessor)
//konczymy definicje pluginu
CORE_PLUGIN_END

#endif  //  HEADER_GUARD___PLUGIN_H__
