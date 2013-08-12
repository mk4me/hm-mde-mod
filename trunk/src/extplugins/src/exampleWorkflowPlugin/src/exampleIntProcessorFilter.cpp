#include "PCH.h"
#include "exampleIntProcessorFilter.h"
#include "exampleIntWidgetProcessorFilterConfiguration.h"

void ExampleIntProccesorFilter::_ExampleIntProcessorFilter()
{
	inPinA = new ExampleIntInputPin(this);
	outPinA = new ExampleIntOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

ExampleIntProccesorFilter::ExampleIntProccesorFilter(const DataFilter & dataFilter)
    : dataFilter(dataFilter)
{
    UTILS_ASSERT((dataFilter.empty() == false), "Bledny komparator!");
	_ExampleIntProcessorFilter();
}

ExampleIntProccesorFilter::ExampleIntProccesorFilter(const ExampleIntProccesorFilter & processorFilter)
    : dataFilter(processorFilter.dataFilter)
{
	_ExampleIntProcessorFilter();
}


void ExampleIntProccesorFilter::process()
{
	IntsConstPtr vals = inPinA->getValue();
	IntsPtr ints(new Ints());
	
	for(auto it = vals->begin(); it != vals->end(); it++){
	    if(dataFilter(*it) == true){
	        ints->push_back(*it);
	    }
	}
	
	outPinA->setValue(ints);
}

void ExampleIntProccesorFilter::reset()
{
}


void ExampleIntProccesorFilter::refreshConfiguration()
{
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
