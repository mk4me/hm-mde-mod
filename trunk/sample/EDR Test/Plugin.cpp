#define CORE_DISABLE_LOGGING
#include <core/Plugin.h>
#include <core/IIdentifiable.h>
#include <QtGui/QWidget>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QLabel>
#include <memory>
#include <fstream>
//#include <plugins/video/Wrappers.h>
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

    virtual IVisualizer* create() const
    {
        return new SampleVisualizer();
    }

    virtual const std::string& getName() const
    {
        static std::string name = "SampleVisualizer";
        return name;
    }

    virtual QWidget* createWidget(std::vector<QObject*>& actions)
    {
        widget.reset(new QLabel(nullptr));
        widget->setText("aaaaa");
        widget->setStyleSheet(  "QLabel { background: white; }" );
        return widget.get();
    }

    virtual QIcon* createIcon()
    {
        return nullptr;
    }

    virtual void getInputInfo(int source, std::string& name, ObjectWrapper::Types& types)
    {
        if ( source == 0 ) {
            name = "file";
            types.push_back(typeid(std::string));
        }
    }

    virtual void setUp(IObjectSource* source) 
    {
        boost::shared_ptr<const std::string> str = source->getObject(0);
        widget->setText( core::toString(*str) );

//         // pobranie strumienia
//         stream = source->getObject<VideoStreamConstPtr>(0);
//         
//         // alokacja ramki
//         if ( picture.data ) {
//             picture.free();
//         }
//         picture = vidlib::Picture::create(stream->getWidth(), stream->getHeight(), vidlib::PixelFormatRGB24);
    }

    virtual void update(double deltaTime)
    {
    }
};

CORE_PLUGIN_BEGIN("SamplePlugin", UID::GenerateUniqueID("{07FA085C-B1EF-4D2F-8281-785D5EA5086F}"))
CORE_PLUGIN_ADD_PARSER(SampleParser)
CORE_PLUGIN_ADD_VISUALIZER(SampleVisualizer)
CORE_PLUGIN_ADD_OBJECT_WRAPPER(std::string)
CORE_PLUGIN_END