/********************************************************************
    created:  2011/06/28
    created:  28:6:2011   8:20
    filename: exampleScalarChannelProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__
#define HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__

#include <corelib/IPlugin.h>
#include <plugins/c3d/C3DChannels.h>
#include <plugins/newVdf/INodeConfiguration.h>
#include <utils/DataChannelDescriptors.h>
#include <dflib/Pin.h>
#include <dflib/IDFPin.h>
#include <dflib/IDFNode.h>
#include <dflib/Node.h>


class ScalarChannelOutputPin : public df::OutputPin, public df::IDFOutput
{
public:
	ScalarChannelOutputPin(df::ISourceNode * node) : df::OutputPin(node) {}

	const ScalarChannelReaderInterfaceConstPtr value() const { return val; }
	void value(ScalarChannelReaderInterfaceConstPtr val) { this->val = val; }

	virtual void reset() { val = ScalarChannelReaderInterfaceConstPtr(); }

private:
	ScalarChannelReaderInterfaceConstPtr val;
};

class ScalarChannelInputPin : public df::InputPin, public df::IDFInput
{
public:
	ScalarChannelInputPin(df::ISinkNode * node) : df::InputPin(node) {}

public:
	virtual void copyData(const df::IDFOutput * pin) { val = dynamic_cast<const ScalarChannelOutputPin*>(pin)->value(); }
	virtual void reset() { val = ScalarChannelReaderInterfaceConstPtr(); }
	const ScalarChannelReaderInterfaceConstPtr value() const { return val; }

	virtual const bool pinCompatible( const df::IOutputPin * pin ) const
	{
		return dynamic_cast<const ScalarChannelOutputPin*>(pin);
	}

private:
	ScalarChannelReaderInterfaceConstPtr val;
};

// Przyk³ad prostego elementu przetwarzaj¹cego. Operuje on na danych typu scalar channel.
// Element prtzetwarzaj¹cy neguje dane skalarne - mamy tutaj przyk³ad jak pibieraæ dane z wejœcia,
// jak generowaæ dane wyjœciowe i umieszczaæ je na wyjœciu.
class ExampleScalarChannelProcessor : public df::ProcessingNode, public df::IDFProcessor, public vdf::INodeConfiguration
{
    //nadanie unikalnego identyfikatora elementowi przetwarzaj¹cemu
    //automatycznie dodaje metody getID i getDescription z interfejsu core::IIdentifable
    UNIQUE_ID("{04B67836-E8C8-4E07-A8D9-60E3089D2510}");
	CLASS_DESCRIPTION("ExampleScalarChannelProcessor", "ExampleScalarChannelProcessor");

public:
    //! Domyœlny konstruktor ustawaj¹cy element przetwarzj¹cy jako inverter wartoœci sygna³u
    //! \param scale Skala
    ExampleScalarChannelProcessor(double scale = -1);

    //! \param scale Nowa skala która stosujemy dla wartoœci kana³u
    void setScale(double scale);

    //! \return Skala
    double getScale() const;

	virtual void process();
	virtual void reset();

	virtual QWidget* getConfigurationWidget();
	virtual void refreshConfiguration();

private:
    //! Skala
	double scale;
	ScalarChannelOutputPin* outPinA;
	ScalarChannelInputPin* inPinA;
};

#endif  //  HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__