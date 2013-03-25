/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   11:36
    filename: exampleIntVisualizer.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTVISUALIZER_H__
#define HEADER_GUARD___EXAMPLEINTVISUALIZER_H__

#include <corelib/IVisualizer.h>

class QLineEdit;

class ExampleIntVisualizer : public plugin::IVisualizer
{
    UNIQUE_ID("{82978BE1-6G5F-4CE8-9CCE-302F82256A20}");
	CLASS_DESCRIPTION("Example Int Visualizer", "Example Int Visualizer");

public:

    class IntSerie : public plugin::IVisualizer::ISerie
    {
		friend class ExampleIntVisualizer;
    public:
        IntSerie(QLineEdit * widget);

    public:
		virtual void setName(const std::string & name);
		virtual const std::string getName() const;
		virtual void setData(const core::TypeInfo & requestedDataType, const core::ObjectWrapperConstPtr & data);
		virtual void update();
		virtual const core::ObjectWrapperConstPtr & getData() const;
		virtual const core::TypeInfo & getRequestedDataType() const;

    private:
        QLineEdit * widget;
        core::ObjectWrapperConstPtr data;
        std::string name;
		utils::TypeInfo requestedType;
    };

public:
    //!    
    ExampleIntVisualizer();;
    //!
    virtual ~ExampleIntVisualizer() {};
	virtual plugin::IVisualizer * create() const;
	virtual QWidget* createWidget();
	virtual QIcon* createIcon();
	virtual QPixmap takeScreenshot() const;
	virtual void update( double deltaTime );
	virtual ISerie* createSerie( const ISerie* serie );
	virtual ISerie* createSerie(const core::TypeInfo & requestedType, const core::ObjectWrapperConstPtr & data);
	virtual void removeSerie( ISerie* serie );
	virtual void setActiveSerie( ISerie * serie );
	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const;
	virtual void getSupportedTypes( core::TypeInfoList & supportedTypes ) const;
	virtual int getMaxDataSeries() const;

private:
    QWidget * widget;
	plugin::IVisualizer::ISerie* activeSerie;
};

#endif  //  HEADER_GUARD___EXAMPLEINTVISUALIZER_H__

