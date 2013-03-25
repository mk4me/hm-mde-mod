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

class QLineEdit;

class ExampleIntVisualizerStatistics : public QObject, public plugin::IVisualizer
{
    Q_OBJECT;

    UNIQUE_ID("{82978BE1-6G5F-4C08-9CCE-302F82256A20}");
	CLASS_DESCRIPTION("Example Int Visualizer Statistics", "Example Int Visualizer Statistics")

public:
    class StatsSerie : public plugin::IVisualizer::ISerie
    {
		friend class ExampleIntVisualizerStatistics;
    public:
        StatsSerie(QLineEdit * widget);

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
    ExampleIntVisualizerStatistics();;
    //!
    virtual ~ExampleIntVisualizerStatistics() {};

public:
	virtual plugin::IVisualizer * create() const;
	virtual QWidget* createWidget();
	virtual QIcon* createIcon();
	virtual QPixmap takeScreenshot() const;
	virtual void update( double deltaTime );
	virtual plugin::IVisualizer::ISerie* createSerie( const ISerie* serie );
	virtual plugin::IVisualizer::ISerie* createSerie(const core::TypeInfo & requestedType, const core::ObjectWrapperConstPtr & data);
	virtual void removeSerie( ISerie* serie );
	virtual void setActiveSerie( ISerie * serie );
	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const;
	virtual void getSupportedTypes( core::TypeInfoList & supportedTypes ) const;
	virtual int getMaxDataSeries() const;

private slots:
	void shuffle();

private:
	QWidget * widget;
	plugin::IVisualizer::ISerie* activeSerie;
};

#endif  //  HEADER_GUARD___EXAMPLEINTVISUALIZERSTATISTICS_H__

