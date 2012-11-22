/********************************************************************
    created:  2011/06/28
    created:  28:6:2011   8:20
    filename: exampleScalarChannelProcessor.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__
#define HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__

#include <core/Plugin.h>
#include <plugins/c3d/C3DChannels.h>

// Przyk³ad prostego elementu przetwarzaj¹cego. Operuje on na danych typu scalar channel.
// Element prtzetwarzaj¹cy neguje dane skalarne - mamy tutaj przyk³ad jak pibieraæ dane z wejœcia,
// jak generowaæ dane wyjœciowe i umieszczaæ je na wyjœciu.
class ExampleScalarChannelProcessor : public core::IDataProcessor
{
    //nadanie unikalnego identyfikatora elementowi przetwarzaj¹cemu
    //automatycznie dodaje metody getID i getDescription z interfejsu core::IIdentifable
    UNIQUE_ID("{04B67836-E8C8-4E07-A8D9-60E3089D2510}", "ExampleScalarChannelProcessor");

public:
    //! Domyœlny konstruktor ustawaj¹cy element przetwarzj¹cy jako inverter wartoœci sygna³u
    //! \param scale Skala
    ExampleScalarChannelProcessor(double scale = -1);

    //! \return Nazwa elementu przetwarzaj¹cego
    virtual const std::string & getName() const;

    //! \return Klon elementu przetwarzajacego - instancja która byæ mo¿e bêdzie mia³a kontakt z danymi i wykona ich przetwarzanie
    virtual ExampleScalarChannelProcessor* createClone() const;

    //! \brief Metoda przetwarzajaca, pobiera dane z wejœcia i zapisuje je na wyjœciu, klient decyduje o sposobie przetwarzania danych (kolejnoœci)
    //! \param input Obiekt z dostepem do danych wejœciowych - nie ma gwarancji ich istnienie, konieczna weryfikacja
    //! \param output Obiekt udostêpniaj¹cy sk³adowanie danych wyjœciowych, mo¿na zawsze swobodnie zapisywaæ
    virtual void process(core::IObjectSource* input, core::IObjectOutput* output);

    //! \param input Parametr przez który klient opisuje typy danych wejsciowych obs³ugiwane przez dany element przetwarzaj¹cy
    //! \brief ISTOTNE - kolejnoœæ opisu obowi¹zuje potem w danych wejœciowych metody process, powinna byæ sta³a dla ka¿dego biektu przetwarzj¹cego!!
    virtual void getInputInfo(std::vector<InputInfo>& input);

    //! \param output Parametr przez który klient opisuje typy danych wyjsciowych dostarczanych przez dany element przetwarzaj¹cy
    //! \brief ISTOTNE - kolejnoœæ opisu obowi¹zuje potem w danych wyjœciowych metody process, powinna byæ sta³a dla ka¿dego biektu przetwarzj¹cego!!
    virtual void getOutputInfo( std::vector<OutputInfo> & output );

    //! \return Widget z konfiguracj¹ elementu przetwarzaj¹cego, mo¿e byæ nullptr jeœli element nie ma konfiguracji
    virtual QWidget* getConfigurationWidget();

    //! \brief Metoda wywo³ywana po pe³nym cyklu danych w workflow, powinna przygotowaæ element przetwarzj¹cy na now¹ porcjê danych
    virtual void reset();


    //! \param scale Nowa skala która stosujemy dla wartoœci kana³u
    void setScale(double scale);

    //! \return Skala
    double getScale() const;

private:
    //! Skala
    double scale;
};

#endif  //  HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__