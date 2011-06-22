#include "PCH.h"
#include "exampleIntProcessorStatistic.h"

ExampleIntProccesorStatistic* ExampleIntProccesorStatistic::createClone() const
{
    return new ExampleIntProccesorStatistic();
}

const std::string & ExampleIntProccesorStatistic::getName() const
{
    static const std::string name("ExampleIntProccesorStatistic");
    return name;
}

void ExampleIntProccesorStatistic::process(core::IObjectSource* input, core::IObjectOutput* output)
{
    auto inInts = input->getObjects(0);
    auto outInts = output->getObjects(0);

    if(inInts.empty() == true)
    {
        return;
    }

    for(int i = 0; i < inInts.size(); i++){
        IntsConstPtr vals = inInts.getObject(i);
        ExampleIntStatisticsPtr stats(new ExampleIntStatistics());

        for(auto it = vals->begin(); it != vals->end(); it++){
            stats->addSample(*it);
        }

        outInts.addObject(stats);
    }
}

void ExampleIntProccesorStatistic::getInputInfo(std::vector<InputInfo>& info)
{
    InputInfo input;
    input.name = "INT";
    input.required = true;
    input.modify = false;
    input.type = typeid(Ints);

    info.push_back(input);
}

void ExampleIntProccesorStatistic::getOutputInfo( std::vector<OutputInfo> & output )
{
    OutputInfo info;
    info.name = "STATS";
    info.dependentInput.insert(0);
    info.type = typeid(ExampleIntStatistics);

    output.push_back(info);
}

QWidget* ExampleIntProccesorStatistic::getConfigurationWidget()
{
    return nullptr;
}

void ExampleIntProccesorStatistic::reset()
{

}