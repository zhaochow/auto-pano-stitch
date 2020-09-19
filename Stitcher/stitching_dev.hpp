#pragma once
#ifndef stitching_hpp
#define stitching_hpp

#include "opencv2/imgcodecs.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"

class Stitching
{
public:
	Stitching() {};
	~Stitching() {};

	// Stitch input images (need image file path)
	int stitch(const std::vector<cv::String> &images_names);

	// Performs Feature Detection
	void featuresDetection();

	// Performs Feature Matching
	void featuresMatching();

	// Performs Image Matching
	int imagesMatching();

	// Estimates all camera parameters and warping scale (initial homography estimation, bundle adjustment and wave correction)
	void estimateCameraParameters(std::vector<cv::detail::CameraParams> &cameras, float &warped_image_scale);

	// Performs Surface Warping (spherical)
	void surfaceWarping(const std::vector<cv::detail::CameraParams> &cameras, float warped_image_scale);
	
	// Performs Exposure Compensation (only feed compensator, actual compensation is performed later)
	// Using default compensator of OpenCV
	void exposureCompensation();

	// Performs Seam Finding (using default seam finder of OpenCV)
	void seamFinding();
	
	// Performs Blending (using default multi-band blender of OpenCV with 5 bands)
	void blending(float blend_strength = 5);

	std::vector<cv::String> panoramas_names;

private:
	int nb_images_original;
	std::vector<cv::String> images_names;
	std::vector<cv::Mat> panoramas;

	std::vector<cv::Mat> images;
	int nb_images;

	std::vector<cv::detail::ImageFeatures> features;
	std::vector<cv::detail::ImageFeatures> features_work;

	std::vector<cv::detail::MatchesInfo> pairwise_matches;
	std::vector<cv::detail::MatchesInfo> pairwise_matches_work;

	std::vector<int> indices;
	std::vector<int> indices_removed;
	float conf_thresh = 1.f;

	std::vector<cv::Point> corners;
	std::vector<cv::Size> sizes;
	std::vector<cv::UMat> masks_warped;
	std::vector<cv::UMat> masks_warped_work;
	std::vector<cv::UMat> images_warped;

	cv::Ptr<cv::detail::ExposureCompensator> compensator;
};

#endif // stitching_hpp
