#include "stdafx.h"
#include "stitcher.hpp"
#include <QtWidgets/QApplication>
#include <QtGui>

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	Stitcher stitcher;

	stitcher.show();
	
	return app.exec();
}
