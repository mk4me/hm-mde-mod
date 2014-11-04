/********************************************************************
    created:  2011/06/16
    created:  16:6:2011   12:17
    filename: exampleIntVisualizerStatistics.h
    author:   Mateusz Janiak
    
    purpose:  
*********************************************************************/
#ifndef HEADER_GUARD___EXAMPLEINTVISUALIZERSTATISTICS_H__
#define HEADER_GUARD___EXAMPLEINTVISUALIZERSTATISTICS_H__

#include <corelib/IVisualizer.h>
#include <QtCore/QObject>
#include <corelib/AbstractSerie.h>

class QTextEdit;

class ExampleIntVisualizerStatistics : public QObject, public plugin::IVisualizer
{
    Q_OBJECT;

    UNIQUE_ID("{82978BE1-6G5F-4C08-9CCE-302F82256A20}");
	CLASS_DESCRIPTION("Example Int Visualizer Statistics", "Example Int Visualizer Statistics")

public:
    class StatsSerie : public plugin::AbstractSerie
    {
		friend class ExampleIntVisualizerStatistics;
    public:
        StatsSerie(QTextEdit * widget);

    public:
		virtual void update();
		virtual void setupData(const core::VariantConstPtr & data);

	private:
		QTextEdit * widget;
    };

public:
    //!    
    ExampleIntVisualizerStatistics();
    //!
    virtual ~ExampleIntVisualizerStatistics() {};

public:
	virtual plugin::IVisualizer * create() const;
	virtual QWidget* createWidget();
	virtual QIcon* createIcon();
	virtual QPixmap takeScreenshot() const;
	virtual void update( double deltaTime );
	
	virtual plugin::IVisualizer::ISerie *createSerie(const utils::TypeInfo & requestedType, const core::VariantConstPtr & data);
	virtual plugin::IVisualizer::ISerie* createSerie(const ISerie* serie, const utils::TypeInfo & requestedType, const core::VariantConstPtr & data);
	virtual plugin::IVisualizer::ISerie *createSerie(const plugin::IVisualizer::ISerie*);
	

	virtual void removeSerie( ISerie* serie );
	virtual void setActiveSerie( ISerie * serie );
	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const;

	virtual plugin::IVisualizer::ISerie * getActiveSerie();

	virtual void getSupportedTypes(utils::TypeInfoList & supportedTypes) const;
	virtual int getMaxDataSeries() const;

private slots:
	void shuffle();

private:
	QWidget * widget;
	plugin::IVisualizer::ISerie* activeSerie;
};

#endif  //  HEADER_GUARD___EXAMPLEINTVISUALIZERSTATISTICS_H__

