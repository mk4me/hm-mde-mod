/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:23
    filename: exampleIntProcessorFilter.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__


#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include "exampleIntProcessorFlopSignMul.h"
#include <plugins/newVdf/INodeConfiguration.h>
#include <dflib/Pin.h>
#include <dflib/IDFPin.h>
#include <dflib/IDFNode.h>
#include <dflib/Node.h>

class ExampleIntProccesorFilter : public df::ProcessingNode, public df::IDFProcessor, public vdf::INodeConfiguration
{
public:
    typedef boost::function<bool (int)> DataFilter;

public:
    ExampleIntProccesorFilter(const DataFilter & comparator = boost::bind(&ExampleIntProccesorFilter::nonZeroFilter, _1));
    ExampleIntProccesorFilter(const ExampleIntProccesorFilter & processorFilter);

private:
	void _ExampleIntProcessorFilter();

public:
    void setFilter(const DataFilter & coparator);
    const DataFilter & getFilter() const;

    void setReferenceValue(int referenceValue);
    int getReferenceValue() const;

    static bool nonZeroFilter(int val);
	virtual void process();
	virtual void reset();
	virtual QWidget* getConfigurationWidget();
	virtual void refreshConfiguration();

private:
	ExampleIntInputPin * inPinA;
	ExampleIntOutputPin * outPinA;
    DataFilter dataFilter;
};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORFILTER_H__