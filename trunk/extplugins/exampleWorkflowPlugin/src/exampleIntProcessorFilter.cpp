#include "PCH.h"
#include "exampleIntProcessorFilter.h"
#include "exampleIntWidgetProcessorFilterConfiguration.h"

ExampleIntProccesorFilter::ExampleIntProccesorFilter(const DataFilter & dataFilter)
    : dataFilter(dataFilter)
{
    UTILS_ASSERT((dataFilter.empty() == false), "Bledny komparator!");
}

ExampleIntProccesorFilter::ExampleIntProccesorFilter(const ExampleIntProccesorFilter & processorFilter)
    : dataFilter(processorFilter.dataFilter)
{

}

const std::string & ExampleIntProccesorFilter::getName() const
{
    static const std::string name("ExampleIntProccesorFilter");
    return name;
}

ExampleIntProccesorFilter* ExampleIntProccesorFilter::createClone() const
{
    return new ExampleIntProccesorFilter(*this);
}

void ExampleIntProccesorFilter::process(core::IObjectSource* input, core::IObjectOutput* output)
{
    auto inInts = input->getObjects(0);
    auto outInts = output->getObjects(0);

    if(inInts.empty() == true)
    {
        return;
    }
    
    for(int i = 0; i < inInts.size(); i++){
        IntsConstPtr vals = inInts.getObject(i);
        IntsPtr ints(new Ints());

        for(auto it = vals->begin(); it != vals->end(); it++){
            if(dataFilter(*it) == true){
                ints->push_back(*it);
            }
        }

        outInts.addObject(ints);
    }
}

void ExampleIntProccesorFilter::getInputInfo(std::vector<InputInfo>& info)
{
    InputInfo input;
    input.required = true;
    input.modify = false;
    input.name = "INT";
    input.type = typeid(Ints);

    info.push_back(input);
}

void ExampleIntProccesorFilter::getOutputInfo( std::vector<OutputInfo> & output )
{
    OutputInfo info;
    info.dependentInput.insert(0);
    info.name = "INT";
    info.type = typeid(Ints);

    output.push_back(info);
}

QWidget* ExampleIntProccesorFilter::getConfigurationWidget()
{
    return new ExampleWidgetProcessorFilterConfiguration(this);
}

void ExampleIntProccesorFilter::setFilter(const DataFilter & dataFilter)
{
    UTILS_ASSERT((dataFilter.empty() == false), "Bledny komparator!");
    this->dataFilter = dataFilter;
}
const ExampleIntProccesorFilter::DataFilter & ExampleIntProccesorFilter::getFilter() const
{
    return dataFilter;
}

bool ExampleIntProccesorFilter::nonZeroFilter(int val)
{
    if(val != 0){
        return true;
    }

    return false;
}

void ExampleIntProccesorFilter::reset()
{

}