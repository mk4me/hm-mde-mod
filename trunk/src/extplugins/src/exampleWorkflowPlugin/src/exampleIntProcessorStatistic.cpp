#include "PCH.h"
#include "exampleIntProcessorStatistic.h"

ExampleIntProccesorStatistic::ExampleIntProccesorStatistic()
{
	inPinA = new ExampleIntInputPin(this);
	outPinA = new StatsOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

void ExampleIntProccesorStatistic::process()
{
	IntsConstPtr vals = inPinA->getValue();
	if(vals->empty() == true)
	{
	    return;
	}

	ExampleIntStatisticsPtr stats(new ExampleIntStatistics());
	for(auto it = vals->begin(); it != vals->end(); it++){
	    stats->addSample(*it);
	}
	outPinA->setValue(stats);
}

void ExampleIntProccesorStatistic::reset()
{
}

