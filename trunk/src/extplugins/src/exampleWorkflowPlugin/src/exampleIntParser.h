#include <core/IParser.h>

class ExampleIntParser : public core::IParser
{
    UNIQUE_ID("{A4B67836-E8C8-4E77-A8D9-60E3089D2510}", "ExampleIntParser");

public:

    ExampleIntParser();

    virtual IParser* create();
    
    virtual void parseFile(const core::Filesystem::Path& path);
    
    virtual void getSupportedExtensions(core::IParser::Extensions & extensions) const;

    virtual void getObjects(core::Objects & objects);

private:

    core::ObjectWrapperPtr adapter;
};