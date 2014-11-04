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
#include "exampleIntRemoteSource.h"
#include "corelib/AbstractSerie.h"

class QLineEdit;

class ExampleIntVisualizer : public plugin::IVisualizer
{
    UNIQUE_ID("{82978BE1-6G5F-4CE8-9CCE-302F82256A20}");
	CLASS_DESCRIPTION("Example Int Visualizer", "Example Int Visualizer");

public:

    class IntSerie : public plugin::AbstractSerie
    {
		friend class ExampleIntVisualizer;
    public:
        IntSerie(QListView * view);

    public:
		virtual void update();
		virtual void setupData(const core::VariantConstPtr & data);

    private:
        QListView * view;
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
	
	virtual plugin::IVisualizer::ISerie *createSerie(const utils::TypeInfo & requestedType, const core::VariantConstPtr & data);
	virtual plugin::IVisualizer::ISerie* createSerie(const ISerie* serie, const utils::TypeInfo & requestedType, const core::VariantConstPtr & data);
	virtual plugin::IVisualizer::ISerie *createSerie(const plugin::IVisualizer::ISerie*);
	
	virtual void removeSerie(ISerie* serie);
	virtual void setActiveSerie( ISerie * serie );
	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const;
	virtual plugin::IVisualizer::ISerie * getActiveSerie();

	virtual void getSupportedTypes(utils::TypeInfoList & supportedTypes) const;
	virtual int getMaxDataSeries() const;

private:
    QWidget * widget;
	plugin::IVisualizer::ISerie* activeSerie;
};

#endif  //  HEADER_GUARD___EXAMPLEINTVISUALIZER_H__

