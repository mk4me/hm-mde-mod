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

// Przyk�ad prostego elementu przetwarzaj�cego. Operuje on na danych typu scalar channel.
// Element prtzetwarzaj�cy neguje dane skalarne - mamy tutaj przyk�ad jak pibiera� dane z wej�cia,
// jak generowa� dane wyj�ciowe i umieszcza� je na wyj�ciu.
class ExampleScalarChannelProcessor : public core::IDataProcessor
{
    //nadanie unikalnego identyfikatora elementowi przetwarzaj�cemu
    //automatycznie dodaje metody getID i getDescription z interfejsu core::IIdentifable
    UNIQUE_ID("{04B67836-E8C8-4E07-A8D9-60E3089D2510}", "ExampleScalarChannelProcessor");

public:
    //! Domy�lny konstruktor ustawaj�cy element przetwarzj�cy jako inverter warto�ci sygna�u
    //! \param scale Skala
    ExampleScalarChannelProcessor(double scale = -1);

    //! \return Nazwa elementu przetwarzaj�cego
    virtual const std::string & getName() const;

    //! \return Klon elementu przetwarzajacego - instancja kt�ra by� mo�e b�dzie mia�a kontakt z danymi i wykona ich przetwarzanie
    virtual ExampleScalarChannelProcessor* createClone() const;

    //! \brief Metoda przetwarzajaca, pobiera dane z wej�cia i zapisuje je na wyj�ciu, klient decyduje o sposobie przetwarzania danych (kolejno�ci)
    //! \param input Obiekt z dostepem do danych wej�ciowych - nie ma gwarancji ich istnienie, konieczna weryfikacja
    //! \param output Obiekt udost�pniaj�cy sk�adowanie danych wyj�ciowych, mo�na zawsze swobodnie zapisywa�
    virtual void process(core::IObjectSource* input, core::IObjectOutput* output);

    //! \param input Parametr przez kt�ry klient opisuje typy danych wejsciowych obs�ugiwane przez dany element przetwarzaj�cy
    //! \brief ISTOTNE - kolejno�� opisu obowi�zuje potem w danych wej�ciowych metody process, powinna by� sta�a dla ka�dego biektu przetwarzj�cego!!
    virtual void getInputInfo(std::vector<InputInfo>& input);

    //! \param output Parametr przez kt�ry klient opisuje typy danych wyjsciowych dostarczanych przez dany element przetwarzaj�cy
    //! \brief ISTOTNE - kolejno�� opisu obowi�zuje potem w danych wyj�ciowych metody process, powinna by� sta�a dla ka�dego biektu przetwarzj�cego!!
    virtual void getOutputInfo( std::vector<OutputInfo> & output );

    //! \return Widget z konfiguracj� elementu przetwarzaj�cego, mo�e by� nullptr je�li element nie ma konfiguracji
    virtual QWidget* getConfigurationWidget();

    //! \brief Metoda wywo�ywana po pe�nym cyklu danych w workflow, powinna przygotowa� element przetwarzj�cy na now� porcj� danych
    virtual void reset();


    //! \param scale Nowa skala kt�ra stosujemy dla warto�ci kana�u
    void setScale(double scale);

    //! \return Skala
    double getScale() const;

private:
    //! Skala
    double scale;
};

#endif  //  HEADER_GUARD___EXAMPLESCALARCHANNELPROCESSOR_H__