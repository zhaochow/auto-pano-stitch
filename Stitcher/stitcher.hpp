#pragma once

#include <QtWidgets/QWidget>
#include "imageslist.hpp"

class Stitcher : public QWidget
{
	Q_OBJECT

public:
	Stitcher(QWidget *parent = Q_NULLPTR);

private:
	// Add panmoramas to the list of panoramas
	void addPanosToList(const std::vector<cv::String> &panoramasNames);

	QGridLayout *gridLayout;

	QGraphicsView *view;
	QGraphicsScene *scene;
	QGraphicsPixmapItem *item;

	ImagesList *imagesList;
	QPushButton *stitchButton;
	QPushButton *clearButton;
	QListWidget *panoList;

private slots:
	// Stitch images in the list of images
	void stitchImages();

	// Display currently selected panorama
	void displaySelectedPano(int currentRow);

	// Clear all items in the lists of images and panoramas
	void clearLists();
};
