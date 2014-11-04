/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   9:34
    filename: exampleIntProcessorFlopSignMul.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__
#define HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__


#include <exampleWorkflowPlugin/exampleIntStatistics.h>
#include "Plugin.h"
#include "examplePins.h"
#include "dflib/Node.h"
#include "dflib/IDFNode.h"


class ExampleIntProccesorFlopSignMul : public df::ProcessingNode, public df::IDFProcessor
{
public:
	ExampleIntProccesorFlopSignMul();

public:
	virtual void reset();
	virtual void process();

private:
	ExampleIntInputPin * inPinA;
	ExampleIntInputPin * inPinB;
	ExampleIntOutputPin * outPinA;
	QWidget* widget;
};

#endif  //  HEADER_GUARD___EXAMPLEINTPROCESSORFLOPSIGNMUL_H__