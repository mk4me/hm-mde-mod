#include "Plugin.h"

#include <vector>
#include <map>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <sstream>
#include <stack>
#include <iostream>
#include <list>

UTILS_PUSH_WARNINGS
#include <osg/Node>
#include <osg/AnimationPath>
#include <osg/Config>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Image>
#include <osg/ImageStream>
#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/Notify>
#include <osg/observer_ptr>
#include <osg/PositionAttitudeTransform>
#include <osg/PrimitiveSet>
#include <osg/Quat>
#include <osg/ref_ptr>
#include <osg/Referenced>
#include <osg/ShapeDrawable>
#include <osg/Timer>
#include <osg/Vec3>
#include <osg/Vec3d>

#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>

#include <osgWidget/Widget>
#include <osgWidget/Window>
#include <osgWidget/Label>
#include <osgWidget/Box>
#include <osgWidget/Canvas>
#include <osgWidget/Table>

#include <osgDB/ReadFile>

UTILS_POP_WARNINGS

#include <QtGui/QDialog>
#include <QtGui/QMessageBox>
#include <QtCore/QString>
#include <QtCore/QtCore>
#include <QtGui/QtGui>
#include <QtOpenGL/QtOpenGL>
#include <QtGui/QAction>
#include <QtGui/QWidget>

#include <boost/lexical_cast.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/any.hpp>
#include <boost/range.hpp>
#include <boost/iterator.hpp>

#include <utils/Debug.h>