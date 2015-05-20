#include "calibWidget.h"

CalibWidget::CalibWidget(ECalibStage& refCalibStage) : _calibStage(refCalibStage)
{
	// QtSetup
    setupUi(this);

	// My setup
	_calibStage = CS_START;
	stageLabel->setText(QString("Stand straight, look forward.\nOperator should press BIND pose when ready."));
	bindButton->setEnabled(true);
	bowButton->setEnabled(false);
}


void CalibWidget::bindClickedSig()
{
	// Move to BIND pose (callibration module will now aquire bind poses)
	if (_calibStage == CS_START)
	{
		_calibStage = CS_BINDPOSE;
		stageLabel->setText(QString("Bow, look down.\nOperator should press BOW pose when ready."));
		bindButton->setEnabled(false);
		bowButton->setEnabled(false);
		//bowButton->setEnabled(true);
	}
	else
		return;
}

void CalibWidget::bowClickedSig()
{
	// Move to BOW pose (callibration module will now aquire bow poses)
	if (_calibStage == CS_BINDPOSE)
	{
		_calibStage = CS_BOWPOSE;
		stageLabel->setText(QString("Callibration finished.\nWindow will dissapear automatically."));
		bindButton->setEnabled(false);
		bowButton->setEnabled(false);
	}
	else
		return;
}

