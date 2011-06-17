/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   11:36
    filename: exampleIntVisualizer.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTVISUALIZER_H__
#define HEADER_GUARD___EXAMPLEINTVISUALIZER_H__

#include <core/IVisualizer.h>

class QLineEdit;

class ExampleIntVisualizer : public core::IVisualizer
{
    UNIQUE_ID("{82978BE1-6G5F-4CE8-9CCE-302F82256A20}", "ExampleIntVisualizer");

public:

    class IntSerie : public core::IVisualizer::SerieBase
    {
    public:
        IntSerie(QLineEdit * widget);

    protected:

        virtual void setSerieName(const std::string & name);

        virtual void setSerieData(const core::ObjectWrapperConstPtr & data);

    public:
        QLineEdit * widget;
    };

public:
    //!    
    ExampleIntVisualizer() {};
    //!
    virtual ~ExampleIntVisualizer() {};

public:
    //! \see IVisualizer::getName
    virtual const std::string& getName() const;
    //! \see IVisualizer::create
    virtual core::IVisualizer* createClone() const;
    //! \see IVisualizer::getSlotInfo
    virtual void getInputInfo(std::vector<core::IInputDescription::InputInfo>& info);
    //! Nic nie robi.
    //! \see IVisualizer::update
    virtual void update(double deltaTime);
    //! \see IVisualizer::createWidget
    virtual QWidget* createWidget(std::vector<QObject*>& actions);
    //! \see IVisualizer::createIcon
    virtual QIcon* createIcon();
    //! \see IVisualizer::setUp
    virtual void setUp(core::IObjectSource* source);

    virtual int getMaxDataSeries() const;

    //! \return Seria danych ktora mozna ustawiac - nazwa i dane, nie zarzadza ta seria danych - czasem jej zycia, my zwalniamy jej zasoby!!
    virtual core::IVisualizer::SerieBase* createSerie(const core::ObjectWrapperConstPtr & data, const std::string & name = std::string());

    //! \param serie Seria danych do usuniêcia, nie powinien usuwac tej serii! Zarzadzamy nia my!!
    virtual void removeSerie(core::IVisualizer::SerieBase* serie);

    virtual void reset();

private:
    QWidget * widget;
};

#endif  //  HEADER_GUARD___EXAMPLEINTVISUALIZER_H__

