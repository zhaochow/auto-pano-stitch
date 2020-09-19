#include "stdafx.h"
#include "stitching_dev.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/warpers.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

int Stitching::stitch(const std::vector<cv::String> &images_names)
{
	nb_images_original = images_names.size();

	// Read input images
	for (int i = 0; i < nb_images_original; ++i)
	{
		Mat img = imread(images_names[i]);
		if (img.empty())
			return -1;
		this->images_names.push_back(images_names[i]);
		images.push_back(img);
		indices.push_back(i);
	}
	nb_images = images.size();

	// Check if have enough images
	if (nb_images < 2)
		return -1;

	featuresDetection();
	featuresMatching();

	while (imagesMatching() == 0) // If 2 or more images match
	{
		vector<CameraParams> cameras;
		float warped_image_scale;
		estimateCameraParameters(cameras, warped_image_scale);
		surfaceWarping(cameras, warped_image_scale);
		exposureCompensation();
		seamFinding();
		blending();

		if (indices_removed.size() >= 2) // If 2 or more images were discarded, prepare for potential next panorama
		{
			indices = indices_removed;
			indices_removed.clear();

			for (int i = 0; i < indices.size(); ++i)
			{
				features_work.push_back(features[indices[i]]);
				for (int j = 0; j < indices.size(); ++j)
					pairwise_matches_work.push_back(pairwise_matches[indices[i] * nb_images_original + indices[j]]);
			}
		}
		else
			break;
	}	

	// Write stitched panorama(s) to file
	for (int i = 0; i < panoramas.size(); ++i)
	{
		string fileName = "panorama" + to_string(i + 1) + ".jpg";
		panoramas_names.push_back(fileName);

		imwrite(fileName, panoramas[i]);
	}

	return 0;
}

void Stitching::featuresDetection()
{
	Ptr<Feature2D> finder = ORB::create(); // Create an ORB features finder

	Mat img;
	features.resize(nb_images);

	for (int i = 0; i < nb_images; ++i)
	{
		img = images[i].clone();
		computeImageFeatures(finder, img, features[i]); // Extract features in image
		features[i].img_idx = i; // Tag features with image index
	}
	img.release();

	for (int i = 0; i < features.size(); ++i)
		features_work.push_back(features[i]); // Copy features for future work
}

void Stitching::featuresMatching()
{
	// Create features matcher which returns the best of the 2 nearest neighbours
	Ptr<FeaturesMatcher> matcher = makePtr<BestOf2NearestMatcher>();

	(*matcher)(features, pairwise_matches); // Match features
	matcher->collectGarbage(); // Free unused memory allocated

	for (int i = 0; i < pairwise_matches.size(); ++i)
		pairwise_matches_work.push_back(pairwise_matches[i]); // Copy matches for future work
}

int Stitching::imagesMatching()
{
	// Leave only images that are from the same panorama with high confidence (defined by a threshold)
	// ATTENTION features_work, pairwise_matches_work are modified
	vector<int> indices_work = leaveBiggestComponent(features_work, pairwise_matches_work, conf_thresh); // ascending order indices to work with

	vector<Mat> images_subset;
	int j = 0;
	for (size_t i = 0; i < indices.size(); ++i)
	{
		if (j < indices_work.size() && i == indices_work[j]) // If index i is present in indices_work
		{
			images_subset.push_back(imread(images_names[indices[i]])); // Copy image for future work
			++j;
		}
		else // Image i to discard
		{
			indices_removed.push_back(indices[i]);
		}
	}
	nb_images = static_cast<int>(images_subset.size());
	images.clear();
	for (size_t i = 0; i < nb_images; ++i)
		images.push_back(images_subset[i]);
	indices.clear();
	images_subset.clear();

	// Check if we still have enough images
	if (nb_images < 2)
		return -1;
	return 0;
}

void Stitching::estimateCameraParameters(vector<CameraParams> &cameras, float &warped_image_scale)
{
	// Initial homography estimation
	Ptr<Estimator> estimator = makePtr<HomographyBasedEstimator>();

	(*estimator)(features_work, pairwise_matches_work, cameras);

	for (size_t i = 0; i < cameras.size(); ++i)
	{
		Mat R;
		cameras[i].R.convertTo(R, CV_32F);
		cameras[i].R = R; // Convert R to 32 bit float (necessary for later)
	}

	// Bundle adjustment
	Ptr<BundleAdjusterBase> adjuster = makePtr<detail::BundleAdjusterRay>();

	(*adjuster)(features_work, pairwise_matches_work, cameras);

	features_work.clear();
	pairwise_matches_work.clear();

	// Find median focal length
	vector<double> focals;
	for (size_t i = 0; i < cameras.size(); ++i)
		focals.push_back(cameras[i].focal);

	sort(focals.begin(), focals.end()); // Sort into ascending order
	
	// Compute scale for warping
	if (focals.size() % 2 == 1)
		warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
	else
		warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

	// Wave correction (horizontal)
	vector<Mat> rmats;
	for (size_t i = 0; i < cameras.size(); ++i)
		rmats.push_back(cameras[i].R.clone());
	waveCorrect(rmats, WAVE_CORRECT_HORIZ);
	for (size_t i = 0; i < cameras.size(); ++i)
		cameras[i].R = rmats[i];
}

void Stitching::surfaceWarping(const vector<CameraParams> &cameras, float warped_image_scale)
{
	corners.resize(nb_images);
	sizes.resize(nb_images);
	masks_warped.resize(nb_images);
	images_warped.resize(nb_images);
	vector<UMat> masks(nb_images);

	// Prepare images masks
	for (int i = 0; i < nb_images; ++i)
	{
		masks[i].create(images[i].size(), CV_8U);
		masks[i].setTo(Scalar::all(255));
	}

	// Warp images and their masks
	cv::Ptr<cv::WarperCreator> warper_creator = makePtr<cv::SphericalWarper>();
	Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale));

	for (int i = 0; i < nb_images; ++i)
	{
		Mat K;
		cameras[i].K().convertTo(K, CV_32F); // Needed for later use

		corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]); // Get top left corner
		sizes[i] = images_warped[i].size();

		warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
		masks_warped_work.push_back(masks_warped[i].clone()); // Copy masks for future work
	}
	images.clear();
}

void Stitching::exposureCompensation()
{
	compensator = ExposureCompensator::createDefault(ExposureCompensator::GAIN_BLOCKS);
	compensator->feed(corners, images_warped, masks_warped_work);
}

void Stitching::seamFinding()
{
	vector<UMat> images_warped_f(nb_images);
	for (int i = 0; i < nb_images; ++i)
		images_warped[i].convertTo(images_warped_f[i], CV_32F);

	cv::Ptr<cv::detail::SeamFinder> seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
	seam_finder->find(images_warped_f, corners, masks_warped_work); // masks_warped_work are updated
}

void Stitching::blending(float blend_strength)
{
	Mat img_warped, img_warped_s;
	Mat dilated_mask, seam_mask, mask_warped;
	Ptr<Blender> blender;
	
	for (int img_idx = 0; img_idx < nb_images; ++img_idx)
	{
		img_warped = images_warped[img_idx].getMat(ACCESS_READ).clone();
		mask_warped = masks_warped[img_idx].getMat(ACCESS_READ).clone(); // Need original warped mask
		
		compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped); // Compensate exposure

		img_warped.convertTo(img_warped_s, CV_16S); // Necessary for later

		// Get mask for the current image (taking into account Seam Finding step)
		dilate(masks_warped_work[img_idx], dilated_mask, Mat());
		resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
		mask_warped = seam_mask & mask_warped;

		if (!blender) // Only creates blender once
		{
			// Use multi-band blending, same as default in OpenCV
			blender = Blender::createDefault(Blender::MULTI_BAND);
			Size dst_sz = resultRoi(corners, sizes).size();
			float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
			MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
			mb->setNumBands(static_cast<int>(ceil(log(blend_width) / log(2.)) - 1.));
			blender->prepare(corners, sizes);
		}

		// Feed the blender with the current image
		blender->feed(img_warped_s, mask_warped, corners[img_idx]);
	}
	corners.clear();
	sizes.clear();
	masks_warped.clear();
	masks_warped_work.clear();
	images_warped.clear();

	Mat result, result_mask;
	blender->blend(result, result_mask);

	result.convertTo(result, CV_8U); // Convert back to 8 bit since values 0->255
	panoramas.push_back(result); // Store result
}