#include "PCH.h"
#include <sstream>
#include <fstream>
#include "exampleIntParser.h"
#include "Plugin.h"


ExampleIntParser::ExampleIntParser() :
	adapter(utils::ObjectWrapper::create<Ints>())
{
}

plugin::IParser* ExampleIntParser::create() const
{
    return new ExampleIntParser();
}

void ExampleIntParser::parse( const std::string & source )
{
    using boost::lexical_cast;
    using boost::bad_lexical_cast;

    typedef boost::tokenizer<boost::char_separator<char> > Tokenizer;

    std::ifstream inputFile(source);

    if(inputFile.is_open() == false){
        PLUGIN_LOG_INFO(std::string("Problem while reading file - could not open file: ") + source);
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
                    PLUGIN_LOG_ERROR(std::string("Error parsing int source file: ") + e.what());
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




void ExampleIntParser::acceptedExpressions( Expressions & expressions ) const
{
	plugin::IParser::ExpressionDescription expDesc;
	expDesc.description = "CSV format";
	expDesc.objectsTypes.push_back(typeid(Ints));
	expressions[".*\.csv$"] = expDesc;
}

void ExampleIntParser::getObject(core::Variant& object, const core::VariantsVector::size_type idx) const
{
	if (idx == 0) {
		object.set(adapter);
	}
}

void ExampleIntParser::reset()
{
	throw std::logic_error("The method or operation is not implemented.");
}


