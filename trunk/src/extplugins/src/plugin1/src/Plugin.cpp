#define CORE_DISABLE_LOGGING
#include <corelib/IPlugin.h>
#include <corelib/PluginCommon.h>
#include <corelib/IIdentifiable.h>
#include <QtGui/QWidget>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QLabel>
#include <QtGui/QIcon>
#include <memory>
#include <fstream>
#include <utils/ClonePolicies.h>
#include <utils/ObjectWrapper.h>


using namespace core;

DEFINE_WRAPPER(std::string, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);


class SampleParser : public plugin::IParser, public plugin::ISourceParserCapabilities
{
private:
    utils::ObjectWrapperPtr object;

public:
    UNIQUE_ID("{55D3B9E4-1759-4BFE-B9CF-D5C25155D442}");
	CLASS_DESCRIPTION("Sample parser", "Sample parser");
    
public:
    SampleParser() : object(utils::ObjectWrapper::create<std::string>())
    {
        
    }

    virtual IParser* create() const
    {
        return new SampleParser();
    }

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

	virtual void acceptedExpressions( Expressions & expressions )  const
    {
        plugin::IParser::ExpressionDescription extDesc;
        extDesc.description = "C3D format";
        extDesc.types.insert(typeid(std::string));
        expressions[".*\\.c3d$"] = extDesc;
        extDesc.description = "AVI format";
        expressions[".*\\.avi$"] = extDesc;
    }
    
    virtual void getObjects(core::Objects& objects)
    {
        objects.insert(object);
    }
};

class SampleVisualizer : public plugin::IVisualizer
{
    UNIQUE_ID("{693700C2-2EAE-4FC7-96D6-54D920D41A8F}");
	CLASS_DESCRIPTION("Sample visualizer", "Sample visualizer");

    std::string name;
    std::auto_ptr<QLabel> widget;
	plugin::IVisualizer::ISerie * activeSerie;

public:
    SampleVisualizer() :
		name("SampleVisualizer"),
		activeSerie(nullptr)
    {
    }

	class SampleVisualizerSerie : public plugin::IVisualizer::ISerie
	{
	public:
		SampleVisualizerSerie(SampleVisualizer * visualizer)
			: visualizer(visualizer)
		{

		}

	public:
		virtual const core::TypeInfo & getRequestedDataType() const
		{
			return requestedType;
		}

		virtual void setName(const std::string & name)
		{
			this->name = name;
		}

        virtual const std::string getName() const
        {
            return name;
        }

		virtual void setData(const core::TypeInfo & requestedDataType, const core::ObjectWrapperConstPtr & data)
		{
            this->data = data;
			boost::shared_ptr<const std::string> str = data->get();
			visualizer->widget->setText(QString::fromStdString(*str));
			this->requestedType = requestedDataType;
		}

        virtual const core::ObjectWrapperConstPtr & getData() const
        {
            return data;
        }

		virtual void update() {}

	private:
		SampleVisualizer* visualizer;
        std::string name;
        core::ObjectWrapperConstPtr data;
		utils::TypeInfo requestedType;
	};

	virtual void update( double deltaTime ) 
	{
	}
	
	virtual void setActiveSerie( plugin::IVisualizer::ISerie* serie )
	{
		this->activeSerie = serie;
	}
	virtual const plugin::IVisualizer::ISerie * getActiveSerie() const
	{
		return activeSerie;
	}
		
	virtual IVisualizer* create() const
    {
        return new SampleVisualizer();
    }


    virtual plugin::IVisualizer::ISerie *createSerie(const plugin::IVisualizer::ISerie*)
    {
        throw std::runtime_error("Not implemented");
        return nullptr;
    }

	virtual plugin::IVisualizer::ISerie *createSerie(const core::TypeInfo & requestedType, const utils::ObjectWrapperConstPtr & data)
	{
		plugin::IVisualizer::ISerie* ret = new SampleVisualizerSerie(this);
		ret->setName("Example serie");
		ret->setData(requestedType, data);

		return ret;
	}

	virtual void removeSerie(plugin::IVisualizer::ISerie *serie)
	{
		widget->setText("serie removed");
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

	virtual void getSupportedTypes( core::TypeInfoList & supportedTypes ) const
	{
		supportedTypes.push_back(typeid(std::string));
	}

};


CORE_PLUGIN_BEGIN("SamplePlugin", UID::GenerateUniqueID("{07FA085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(SampleParser)
CORE_PLUGIN_ADD_VISUALIZER(SampleVisualizer)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(std::string)
CORE_PLUGIN_END
