/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:24
    filename: exampleIntProcessorStatistic.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__


#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include <plugins/newVdf/INodeConfiguration.h>
#include <dflib/Pin.h>
#include <dflib/IDFPin.h>
#include <dflib/IDFNode.h>
#include <dflib/Node.h>
#include "examplePins.h"


class ExampleIntProccesorStatistic :  public df::ProcessingNode, public df::IDFProcessor
{
public:
	ExampleIntProccesorStatistic();
	virtual void reset();
	virtual void process();

private:
	ExampleIntInputPin * inPinA;
	ExampleStatsOutputPin * outPinA;
};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORSTATISTIC_H__