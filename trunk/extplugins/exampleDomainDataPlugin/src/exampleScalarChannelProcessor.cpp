#include "exampleScalarChannelProcessor.h"
#include "exampleScalarConfigurationWidget.h"

ExampleScalarChannelProcessor::ExampleScalarChannelProcessor(double scale) : scale(scale)
{

}

const std::string & ExampleScalarChannelProcessor::getName() const
{
    static const std::string name("ExampleScalarChannelProcessor");
    return name;
}

ExampleScalarChannelProcessor* ExampleScalarChannelProcessor::createClone() const
{
    return new ExampleScalarChannelProcessor();
}

void ExampleScalarChannelProcessor::process(core::IObjectSource* input, core::IObjectOutput* output)
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

        //modyfikuje dane ktore bêdê zaraz udostepnia³ na wyjœciu
        for(auto it = inScalarData->begin(); it != inScalarData->end(); it++){
            //skalowanie
            outScalarData->addPoint( (*it).value * scale);
        }

        //normalizacja, niezbedna do pracy wizualizatorów
        //teraz wymagana, potem to uproœcimy
        outScalarData->normalize();

        //zapisujê zmodyfikowane dane do udostêpniena
        outScalars.addObject(outScalarData);
    }
}

void ExampleScalarChannelProcessor::getInputInfo(std::vector<InputInfo>& info)
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

void ExampleScalarChannelProcessor::getOutputInfo( std::vector<OutputInfo> & output )
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

QWidget* ExampleScalarChannelProcessor::getConfigurationWidget()
{
    return new ExampleScalarConfigurationWidget(this);
}

void ExampleScalarChannelProcessor::reset()
{

}

void ExampleScalarChannelProcessor::setScale(double scale)
{
    this->scale = scale;
}

double ExampleScalarChannelProcessor::getScale() const
{
    return scale;
}