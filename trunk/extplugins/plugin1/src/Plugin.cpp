#define CORE_DISABLE_LOGGING
#include <core/Plugin.h>
#include <core/PluginCommon.h>
#include <core/IIdentifiable.h>
#include <QtGui/QWidget>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QLabel>
#include <memory>
#include <fstream>
#include <core/IDataManager.h>
#include <utils/ClonePolicies.h>
#include <core/ObjectWrapper.h>


using namespace core;

CORE_DEFINE_WRAPPER(std::string, utils::PtrPolicyBoost, utils::ClonePolicyCopyConstructor);




class SampleParser : public core::IParser
{
private:
    core::ObjectWrapperPtr object;

public:
    UNIQUE_ID("{55D3B9E4-1759-4BFE-B9CF-D5C25155D442}", "Sample parser");
    
public:
    SampleParser()
    {
        object = core::ObjectWrapper::create<std::string>();
    }

    virtual IParser* create()
    {
        return new SampleParser();
    }

    virtual void parseFile(IDataManager* dataManager, const boost::filesystem::path& path)
    {
        std::ostringstream contents;
        std::ifstream file(path.string());

        while ( file ) {
            std::string s;
            std::getline(file, s);
            contents << s << std::endl;
        }

        boost::shared_ptr<std::string> str(new std::string(contents.str()));
        object->set(str);
        object->setName( path.filename().string() );
        object->setSource( path.string() );
    }
    
    virtual std::string getSupportedExtensions() const
    {
        return "*.c3d;*.avi";
    }
    
    virtual void getObjects(std::vector<ObjectWrapperPtr>& objects)
    {
        objects.push_back(object);
    }
};

class SampleVisualizer : public core::IVisualizer
{
    UNIQUE_ID("{693700C2-2EAE-4FC7-96D6-54D920D41A8F}", "Sample visualizer");

    std::string name;
    std::auto_ptr<QLabel> widget;

public:
    SampleVisualizer() :
    name("SampleVisualizer")
    {
    }

	class SampleVisualizerSerie : public SerieBase
	{
	public:
		SampleVisualizerSerie(SampleVisualizer * visualizer)
			: visualizer(visualizer)
		{

		}

	protected:
		virtual void setSerieName(const std::string & name)
		{
			//TODO
			//obecnie nazwy serii nie sa obslugiwane ale musimy pomyslec o tym i ewentualnie dodac!!
		}

		virtual void setSerieData(const core::ObjectWrapperConstPtr & data)
		{
			boost::shared_ptr<const std::string> str = data->get();
			visualizer->widget->setText( core::toString(*str) );
		}

	private:
		SampleVisualizer* visualizer;
	};

    virtual IVisualizer* createClone() const
    {
        return new SampleVisualizer();
    }

    virtual const std::string& getName() const
    {
        static std::string name = "SampleVisualizer";
        return name;
    }

	core::IVisualizer::SerieBase *createSerie(const ObjectWrapperConstPtr & data, const std::string & name)
	{
		core::IVisualizer::SerieBase * ret = new SampleVisualizerSerie(this);
		ret->setName(name);
		ret->setData(data);

		return ret;
	}

	virtual void removeSerie(core::IVisualizer::SerieBase *serie)
	{
		widget->setText("serie removed");
	}

    virtual QWidget* createWidget(std::vector<QObject*>& actions)
    {
        widget.reset(new QLabel(nullptr));
        widget->setText("widget created");
        widget->setStyleSheet(  "QLabel { background: white; }" );
        return widget.get();
    }

    virtual QIcon* createIcon()
    {
        return nullptr;
    }

	virtual int getMaxDataSeries(void) const
	{
		return 7;
	}

	virtual void getInputInfo( std::vector<core::IInputDescription::InputInfo>& info)
	{
		core::IInputDescription::InputInfo input;

		input.name = "model";
		input.type = typeid(std::string);
		input.required = true;
		input.modify = false;

		info.push_back(input);
	}

    virtual void setUp(IObjectSource* source) 
    {
		widget->setText("set up");
    }

    virtual void update(double deltaTime)
    {
    }
    virtual void reset()
    {

    }
};


CORE_PLUGIN_BEGIN("SamplePlugin", UID::GenerateUniqueID("{07FA085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(SampleParser)
CORE_PLUGIN_ADD_VISUALIZER(SampleVisualizer)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(std::string)
CORE_PLUGIN_END
