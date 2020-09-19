#include "stdafx.h"
#include "stitcher.hpp"
#include "stitching_dev.hpp"

Stitcher::Stitcher(QWidget *parent)
	: QWidget(parent)
{
	setMinimumSize(1280, 720);

	gridLayout = new QGridLayout(this);
	
	view = new QGraphicsView(this);
	scene = new QGraphicsScene(view);
	view->setScene(scene);
	view->setDragMode(QGraphicsView::ScrollHandDrag);
	item = new QGraphicsPixmapItem();
	scene->addItem(item);

	imagesList = new ImagesList(this);
	imagesList->setAcceptDrops(true);
	stitchButton = new QPushButton("Stitch images", this);
	connect(stitchButton, SIGNAL(clicked()), SLOT(stitchImages()));
	clearButton = new QPushButton("Clear", this);
	connect(clearButton, SIGNAL(clicked()), SLOT(clearLists()));
	panoList = new QListWidget(this);
	connect(panoList, SIGNAL(currentRowChanged(int)), SLOT(displaySelectedPano(int)));

	// Layout
	gridLayout->addWidget(view, 0, 0, -1, 1);
	gridLayout->addWidget(imagesList, 0, 1, 1, -1);
	gridLayout->addWidget(stitchButton, 1, 1);
	gridLayout->addWidget(clearButton, 1, 2);
	gridLayout->addWidget(panoList, 2, 1, 1, -1);
	gridLayout->setColumnStretch(0, 4);
	gridLayout->setColumnStretch(1, 1);
	gridLayout->setColumnStretch(2, 1);
	gridLayout->setRowStretch(0, 2);
	gridLayout->setRowStretch(1, 1);
	gridLayout->setRowStretch(2, 1);

	setLayout(gridLayout);
}

void Stitcher::stitchImages()
{
	panoList->clear();
	Stitching stitcher;
	stitcher.stitch(imagesList->images_names);

	std::vector<cv::String> panoramasNames = stitcher.panoramas_names;
	addPanosToList(panoramasNames);
}

void Stitcher::addPanosToList(const std::vector<cv::String> &panoramasNames)
{
	if (panoramasNames.size() != 0)
	{
		QString currentPath = QDir::currentPath() + QString("/");
		for (size_t i = 0; i < panoramasNames.size(); ++i)
		{
			QString filename = currentPath + QString::fromStdString(panoramasNames[i]);
			QListWidgetItem *listItem = new QListWidgetItem(filename, panoList);
			panoList->addItem(listItem);
		}
		panoList->setCurrentRow(0);
	}
}

void Stitcher::displaySelectedPano(int currentRow)
{
	if (currentRow == -1) // Clear scene if no item selected
	{
		scene->clear();
		item = new QGraphicsPixmapItem();
		scene->addItem(item);
	}
	else
	{
		item->setPixmap(QPixmap(panoList->item(currentRow)->text()));
		scene->setSceneRect(item->boundingRect());
	}
}

void Stitcher::clearLists()
{
	imagesList->clearData();
	panoList->clear();
}
