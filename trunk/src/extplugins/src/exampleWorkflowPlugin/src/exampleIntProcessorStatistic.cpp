#include "PCH.h"
#include "exampleIntProcessorStatistic.h"

ExampleIntProccesorStatistic::ExampleIntProccesorStatistic()
{
	inPinA = new ExampleIntInputPin(this);
	outPinA = new ExampleStatsOutputPin(this);
	addInputPin(inPinA);
	addOutputPin(outPinA);
}

void ExampleIntProccesorStatistic::process()
{
	IntsConstPtr vals = inPinA->value();
	if(vals->empty() == true)
	{
	    return;
	}

	ExampleIntStatisticsPtr stats(new ExampleIntStatistics());
	for(auto it = vals->begin(); it != vals->end(); it++){
	    stats->addSample(*it);
	}
	outPinA->value(stats);
}

void ExampleIntProccesorStatistic::reset()
{
}

