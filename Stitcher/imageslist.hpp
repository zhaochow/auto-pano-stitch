#pragma once

#include <QListWidget>
#include "stitching_dev.hpp"

class ImagesList : public QListWidget
{
	Q_OBJECT

public:
	ImagesList(QWidget *parent = Q_NULLPTR);
	~ImagesList();

	// Clear all items
	void clearData();

	std::vector<cv::String> images_names;

private:
	// Accept files when drag enters
	void dragEnterEvent(QDragEnterEvent * e) override;

	// See dragEnterEvent
	void dragMoveEvent(QDragMoveEvent * e) override;
	
	// Store file path when dropping
	void dropEvent(QDropEvent * e) override;
};
