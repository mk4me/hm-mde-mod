#include "exampleScalarConfigurationWidget.h"
#include "exampleScalarChannelProcessor.h"
#include <utils/Debug.h>

ExampleScalarConfigurationWidget::ExampleScalarConfigurationWidget(ExampleScalarChannelProcessor * processor, QWidget * parent)
    : QWidget(parent), processor(processor)
{
    UTILS_ASSERT((processor != nullptr), "B³êdny element przetwarzajacy");
    setupUi(this);

    this->doubleSpinBox->blockSignals(true);
    this->doubleSpinBox->setValue(processor->getScale());
    this->doubleSpinBox->blockSignals(false);
}

ExampleScalarConfigurationWidget::~ExampleScalarConfigurationWidget()
{

}

void ExampleScalarConfigurationWidget::scaleChanged(double scale)
{
    processor->setScale(scale);
}