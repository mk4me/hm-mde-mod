#include "PCH.h"
#include <sstream>
#include <fstream>
#include "exampleIntParser.h"


ExampleIntParser::ExampleIntParser() : adapter(core::ObjectWrapper::create<Ints>())
{

}

core::IParser* ExampleIntParser::create()
{
    return new ExampleIntParser();
}

void ExampleIntParser::parseFile(core::IDataManager* dataManager, const boost::filesystem::path& path)
{
    using boost::lexical_cast;
    using boost::bad_lexical_cast;

    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

    std::ifstream inputFile(path.string());

    if(inputFile.is_open() == false){
        LOG_INFO(std::string("Problem while reading file - could not open file: ") + path.string());
        return;
    }

    std::string line;

    bool possiblyHeader = true;

    IntsPtr ints(new Ints());

    while(inputFile.eof() == false){
        std::getline(inputFile, line);

        Tokenizer tokens(line, boost::char_separator<char>(";"));
        for(Tokenizer::iterator it = tokens.begin(); it != tokens.end(); it++){
            
            try{
                int val = boost::lexical_cast<int>(*it);
                ints->push_back(val);
            }
            catch(bad_lexical_cast e){
                if(possiblyHeader == true){
                    possiblyHeader = false;
                    break;
                }else{
                    LOG_ERROR(std::string("Error parsing int source file: ") + e.what());
                    return;
                }
            }
        }

        if(possiblyHeader == true){
            possiblyHeader = false;
        }
    }

    adapter->set(ints);
}

std::string ExampleIntParser::getSupportedExtensions() const
{
    return "csv";
}

void ExampleIntParser::getObjects(std::vector<core::ObjectWrapperPtr>& objects)
{
    objects.push_back(adapter);
}