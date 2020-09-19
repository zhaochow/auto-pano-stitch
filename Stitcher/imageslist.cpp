#include "stdafx.h"
#include "imageslist.hpp"

ImagesList::ImagesList(QWidget *parent)
	: QListWidget(parent)
{
}

ImagesList::~ImagesList()
{
}

void ImagesList::clearData()
{
	clear();
	images_names.clear();
}

void ImagesList::dragEnterEvent(QDragEnterEvent *e)
{
	if (e->mimeData()->hasUrls()) {
		e->acceptProposedAction();
	}
}

void ImagesList::dragMoveEvent(QDragMoveEvent *e)
{
	e->acceptProposedAction();
}

void ImagesList::dropEvent(QDropEvent * e)
{
	for (const QUrl &url : e->mimeData()->urls()) {
		QString fileName = url.toLocalFile();
		addItem(new QListWidgetItem(fileName, this));
		images_names.push_back(fileName.toStdString());
	}
}