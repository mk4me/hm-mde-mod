#include "MainWindow.h"
#include <QtCore/QTextCodec>
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	//potrzebujemy polskich znaków
	//QTextCodec::setCodecForCStrings( QTextCodec::codecForName("Windows-1250") );

	MainWindow w;
	w.show();
	return a.exec();
}

