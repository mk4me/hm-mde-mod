#include "PCH.h"
#include "exampleIntProcessorFlopSignMul.h"

ExampleIntProccesorFlopSignMul* ExampleIntProccesorFlopSignMul::createClone() const
{
    return new ExampleIntProccesorFlopSignMul();
}

const std::string & ExampleIntProccesorFlopSignMul::getName() const
{
    static const std::string name("ExampleIntProccesorFlopSignMul");
    return name;
}

void ExampleIntProccesorFlopSignMul::process(core::IObjectSource* input, core::IObjectOutput* output)
{
    const auto & inIntsA = input->getObjects(0);
    const auto & inIntsB = input->getObjects(1);
    auto & outInts = output->getObjects(0);

    if(inIntsA == nullptr || inIntsB == nullptr)
    {
        return;
    }

    int maxIndex = max(inIntsA->size(), inIntsB->size());

    for(int i = 0; i < maxIndex; i++){
        IntsConstPtr valsA = inIntsA->getObject(i)->get();
        IntsConstPtr valsB = inIntsB->getObject(i)->get();
        
        IntsPtr outVals(new Ints());

        int maxIndexLocal = max(valsA->size(), valsB->size());

        int sign = 1;

        for(int j = 0; j < maxIndexLocal; j++){
            int val = (*valsA)[i] * (*valsB)[i] * sign;

            if(val > 0){
                sign = -1;
            }else{
                sign = 1;
            }
            
            (*outVals)[i] = val;
        }

        core::ObjectWrapperPtr obj = core::ObjectWrapper::create<Ints>();
        obj->set(outVals);

        outInts->addObject(obj);
    }
}

void ExampleIntProccesorFlopSignMul::getInputInfo(std::vector<InputInfo>& info)
{
    InputInfo input;
    
    input.name = "INT_A";
    input.required = true;
    input.modify = false;
    input.type = typeid(Ints);

    info.push_back(input);

    input.name = "INT_B";
    input.required = true;
    input.modify = false;
    input.type = typeid(Ints);

    info.push_back(input);
}

void ExampleIntProccesorFlopSignMul::getOutputInfo( std::vector<OutputInfo> & output )
{
    OutputInfo info;
    info.name = "INTS_M";
    info.dependentInput.insert(0);
    info.dependentInput.insert(1);
    info.type = typeid(Ints);

    output.push_back(info);
}

QWidget* ExampleIntProccesorFlopSignMul::getConfigurationWidget()
{
    return nullptr;
}

void ExampleIntProccesorFlopSignMul::reset()
{

}