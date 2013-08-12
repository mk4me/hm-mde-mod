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
#include <plugins/newVdf/IDataFlowProvider.h>
#include <plugins/dfElements/DFPins.h>


// Przyk�ad prostego elementu przetwarzaj�cego. Operuje on na danych typu scalar channel.
// Element prtzetwarzaj�cy neguje dane skalarne - mamy tutaj przyk�ad jak pibiera� dane z wej�cia,
// jak generowa� dane wyj�ciowe i umieszcza� je na wyj�ciu.
class ExampleScalarChannelProcessor : public df::ProcessingNode, public df::IDFProcessor, public vdf::INodeConfiguration
{
    //nadanie unikalnego identyfikatora elementowi przetwarzaj�cemu
    //automatycznie dodaje metody getID i getDescription z interfejsu core::IIdentifable
    //UNIQUE_ID("{04B67836-E8C8-4E07-A8D9-60E3089D2510}");
	//CLASS_DESCRIPTION("ExampleScalarChannelProcessor", "ExampleScalarChannelProcessor");

public:
    //! Domy�lny konstruktor ustawaj�cy element przetwarzj�cy jako inverter warto�ci sygna�u
    //! \param scale Skala
    ExampleScalarChannelProcessor(double scale = -1);

    //! \param scale Nowa skala kt�ra stosujemy dla warto�ci kana�u
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
	ScalarOutputPin* outPinA;
	ScalarInputPin* inPinA;
};

#endif  //  HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__