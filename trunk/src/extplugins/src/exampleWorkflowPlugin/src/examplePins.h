/********************************************************************
	created:	2013/03/25
	created:	25:3:2013   11:18
	filename: 	examplePins.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_EXAMPLE__EXAMPLEPINS_H__
#define HEADER_GUARD_EXAMPLE__EXAMPLEPINS_H__

#include <dflib/Pin.h>
#include <dflib/IDFPin.h>
#include <dflib/IDFNode.h>
#include <dflib/Node.h>

#include "Plugin.h"

class ExampleIntOutputPin : public df::OutputPin, public df::IDFOutput
{
public:
	ExampleIntOutputPin(df::ISourceNode * node);

public:
	const IntsConstPtr value() const;
	void value(IntsPtr val);
	virtual void reset();

private:
	IntsPtr val;
};


class ExampleIntInputPin : public df::InputPin, public df::IDFInput
{
public:
	ExampleIntInputPin(df::ISinkNode * node);
	
public:
	virtual void copyData(const df::IDFOutput * pin);
	virtual void reset();
	const IntsConstPtr value() const;

private:
	IntsPtr val;
};

class ExampleStatsOutputPin : public df::OutputPin, public df::IDFOutput
{
public:
	ExampleStatsOutputPin(df::ISourceNode * node);

public:
	const ExampleIntStatisticsConstPtr value() const;
	void value(ExampleIntStatisticsPtr val);
	virtual void reset();

private:
	ExampleIntStatisticsPtr val;
};

#endif
