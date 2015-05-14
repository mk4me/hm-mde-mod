#include "calibWidget.h"

CalibWidget::CalibWidget(bool *bindPoseFlag, bool *bowPoseFlag) : _bindPoseFlag(bindPoseFlag), _bowPoseFlag(bowPoseFlag)
{
	// QtSetup
    setupUi(this);

	// My setup
	// ...
}


void CalibWidget::bindClickedSig()
{
	// Set flag
	if (_bindPoseFlag)
		(*_bindPoseFlag) = true;
}
