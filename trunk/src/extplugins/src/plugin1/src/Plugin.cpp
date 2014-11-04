//#define CORE_DISABLE_LOGGING
#include <corelib/IPlugin.h>
#include <corelib/PluginCommon.h>
#include <corelib/IIdentifiable.h>
#include <QtWidgets/QWidget>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QLabel>
#include <QtGui/QIcon>
#include <memory>
#include <fstream>
#include <utils/ClonePolicies.h>
#include <utils/ObjectWrapper.h>
#include <utils/PtrPolicyStd.h>
#include <utils/TypeInfo.h>
#include <corelib/AbstractSerie.h>


using namespace core;

DEFINE_WRAPPER(std::string, utils::PtrPolicyStd, utils::ClonePolicyCopyConstructor);

//! Prosty parser, wczytuje pliki do std::string
class SampleParser : public plugin::ISourceParser
{
private:
    utils::ObjectWrapperPtr object;

public:
    UNIQUE_ID("{55D3B9E4-1759-4BFE-B9CF-D5C25155D442}");
	CLASS_DESCRIPTION("Simple parser", "Simple parser");
    
public:
    SampleParser() : object(utils::ObjectWrapper::create<std::string>())
    {
        
    }

	//! \return pusty obiekt nowego parsera
    virtual IParser* create() const
    {
        return new SampleParser();
    }

	//! Parsowanie pliku 
	//! \param path poprawna œcie¿ka do pliku
    virtual void parse(const std::string& source)
    {
		core::Filesystem::Path path(source);
        std::ostringstream contents;
        std::ifstream file(path.string());

        while ( file ) {
            std::string s;
            std::getline(file, s);
            contents << s << std::endl;
        }

        boost::shared_ptr<std::string> str(new std::string(contents.str()));
        object->set(str);
    }

	//! Zwraca rozszerzenia, które s¹ obs³ugiwane przez parser 
	//! \param extensions kolecja z roszerzeniami
	virtual void acceptedExpressions( Expressions & expressions )  const
    {
        plugin::IParser::ExpressionDescription extDesc;
		extDesc.objectsTypes.push_back(typeid(std::string));
        extDesc.description = "C3D format";
        expressions[".*\\.c3d$"] = extDesc;
        extDesc.description = "AVI format";
        expressions[".*\\.avi$"] = extDesc;
		extDesc.description = "TXT format";
		expressions[".*\\.txt$"] = extDesc;
    }
    
	//! Zwraca obiekty dostarczone przez parser
	virtual void getObject(core::Variant& object, const core::VariantsVector::size_type idx) const
    {
		if (idx == 0) {
			object.set(this->object);
		}
    }

	//! Metoda zwalniaj¹ca sparsowane zasoby parsera
	virtual void reset()
	{
		object.reset();
	}

};

//! Prosty wizualizator 
class SampleVisualizer : public plugin::IVisualizer
{
    UNIQUE_ID("{693700C2-2EAE-4FC7-96D6-54D920D41A8F}");
	CLASS_DESCRIPTION("Sample visualizer", "Sample visualizer");

    std::auto_ptr<QLabel> widget;
	plugin::IVisualizer::ISerie * activeSerie;

public:
    SampleVisualizer() :
		activeSerie(nullptr)
    {
    }

	class SampleVisualizerSerie : public plugin::AbstractSerie
	{
	public:
		SampleVisualizerSerie(SampleVisualizer * visualizer)
			: visualizer(visualizer)
		{

		}

	public:
		virtual void update() {}

		virtual void setupData(const core::VariantConstPtr & data)
		{
			boost::shared_ptr<const std::string> str = data->get();
			visualizer->widget->setText(QString::fromStdString(*str));
		}

	private:
		SampleVisualizer* visualizer;
	};

	virtual void update( double deltaTime ) 
	{
	}
	
	virtual void setActiveSerie( plugin::IVisualizer::ISerie* serie )
	{
		this->activeSerie = serie;
	}
	
	virtual void removeSerie(plugin::IVisualizer::ISerie *serie)
	{
		widget->setText("serie removed");
	}
		
	virtual plugin::IVisualizer::ISerie * getActiveSerie()
	{
		return activeSerie;
	}

	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const
	{
		return activeSerie;
	}

	virtual IVisualizer* create() const
    {
        return new SampleVisualizer();
    }

	virtual plugin::IVisualizer::ISerie *createSerie(const utils::TypeInfo & requestedType, const core::VariantConstPtr & data)
	{
		plugin::AbstractSerie* ret = new SampleVisualizerSerie(this);
		ret->setName("Example serie");
		ret->setData(requestedType, data);

		return ret;
	}


	virtual ISerie* createSerie(const ISerie* serie, const utils::TypeInfo & requestedType, const core::VariantConstPtr & data)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}

	virtual plugin::IVisualizer::ISerie *createSerie(const plugin::IVisualizer::ISerie*)
	{
		throw std::logic_error("The method or operation is not implemented.");
	}


    virtual QWidget* createWidget()
    {
        widget.reset(new QLabel(nullptr));
        widget->setText("widget created");
        widget->setStyleSheet(  "QLabel { background: white; }" );
        return widget.get();
    }

    virtual QIcon* createIcon()
    {
        return new QIcon();
    }

	virtual QPixmap takeScreenshot() const
	{
		return QPixmap::grabWidget(widget.get());
	}

	virtual int getMaxDataSeries(void) const
	{
		return 7;
	}

	virtual void getSupportedTypes( utils::TypeInfoList & supportedTypes ) const
	{
		supportedTypes.push_back(typeid(std::string));
	}

	

};


CORE_PLUGIN_BEGIN("SamplePlugin", UID::GenerateUniqueID("{07FA085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(SampleParser)
CORE_PLUGIN_ADD_VISUALIZER(SampleVisualizer)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(std::string)
CORE_PLUGIN_END
