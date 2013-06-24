#include "CustomQOSGWidget.h"
#include <QtCore/QMutexLocker>

CustomQOSGWidget::CustomQOSGWidget(QWidget * parent, const osg::GraphicsContext::Traits* traits,
	Qt::WindowFlags f) : osgui::QOsgDefaultWidget(parent, traits, f), sync(QMutex::NonRecursive)
{
	_camera->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR ); 
	_camera->setCullingMode(osg::CullSettings::NO_CULLING);
}

CustomQOSGWidget::~CustomQOSGWidget()
{

}

void CustomQOSGWidget::addOperation(Functor func)
{
	if(func.empty() == false){
		QMutexLocker lock(&sync);
		operations.push_back(func);
	}
}

void CustomQOSGWidget::paintEvent( QPaintEvent* event )
{
	if(operations.empty() == false){
		QMutexLocker lock(&sync);
		for(auto it = operations.begin(); it!= operations.end(); ++it){
			try{
				(*it)();
			}catch(...){

			}
		}

		std::list<Functor>().swap(operations);
	}

	osgui::QOsgDefaultWidget::paintEvent(event);
}