#include <core/IParser.h>

class ExampleIntParser : public core::IParser
{
    UNIQUE_ID("{A4B67836-E8C8-4E77-A8D9-60E3089D2510}", "ExampleIntParser");

public:

    ExampleIntParser();

    virtual IParser* create();
    
    virtual void parseFile(core::IDataManager* dataManager, const boost::filesystem::path& path);
    
    virtual std::string getSupportedExtensions() const;

    virtual void getObjects(std::vector<core::ObjectWrapperPtr>& objects);

private:

    core::ObjectWrapperPtr adapter;
};