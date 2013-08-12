#include "PCH.h"
#include "exampleIntProcessorFlopSignMul.h"


void ExampleIntProccesorFlopSignMul::process()
{
    auto inIntsA = inPinA->getValue();
    auto inIntsB = inPinB->getValue();
    if(inIntsA->empty() == true || inIntsB->empty() == true)
    {
        return;
    }

    int maxIndex = (std::min)(inIntsA->size(), inIntsB->size());
	IntsPtr outVals(new Ints());
	int sign = 1;
    for(int i = 0; i < maxIndex; i++){
		int val = (*inIntsA)[i] * (*inIntsB)[i] * sign;
        if(val > 0){
            sign = -1;
        }else{
            sign = 1;
        }
        outVals->push_back(val);
    }
    outPinA->setValue(outVals);
}

void ExampleIntProccesorFlopSignMul::reset()
{
}

ExampleIntProccesorFlopSignMul::ExampleIntProccesorFlopSignMul()
{
	inPinA = new ExampleIntInputPin(this);
	inPinB = new ExampleIntInputPin(this);
	outPinA = new ExampleIntOutputPin(this);
	addInputPin(inPinA);
	addInputPin(inPinB);
	addOutputPin(outPinA);
}
