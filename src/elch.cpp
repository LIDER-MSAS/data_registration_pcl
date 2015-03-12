#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <iostream>
#include <string>

#include <vector>
#include "dataFramework\data_model.hpp"
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;

data_model model;
data_model modelAfterElch;
std::vector<std::string> cloudIds;


Eigen::Vector3f getOrigin (int id )
{
	Eigen::Affine3f mm;
	Eigen::Vector3f origin;
	model.getAffine(cloudIds[id], mm.matrix());
	origin = mm * Eigen::Vector3f(0,0,0);
	return origin;
}



bool
	loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last)
{
	static double min_dist = -1;
	int state = 0;

	for (int i = end-1; i > 0; i--)
	{
		Eigen::Vector3f cstart, cend;
		//TODO use pose of scan
		cstart = getOrigin(i);
		cend = getOrigin(end);
		//pcl::compute3DCentroid (*(clouds[i].second), cstart);
		//pcl::compute3DCentroid (*(clouds[end].second), cend);
		Eigen::Vector3f diff = cend - cstart;

		double norm = diff.norm ();

		//std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

		if (state == 0 && norm > dist)
		{
			state = 1;
			//std::cout << "state 1" << std::endl;
		}
		if (state > 0 && norm < dist)
		{
			state = 2;
			//std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
			if (min_dist < 0 || norm < min_dist)
			{
				min_dist = norm;
				first = i;
				last = end;
			}
		}
	}
	//std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
	if (min_dist > 0 && (state < 2 || end == int (clouds.size ()) - 1)) //TODO
	{
		min_dist = -1;
		return true;
	}
	return false;
}

int
	main (int argc, char **argv)
{
	if(argc < 3)
	{
		printf("usage: elch input.xml output.xml -d 0.1 -r 0.1 -i 100 -l 3.0\n");
		printf("-d maxCorrespondenceDistance\n");
		printf("-r RANSACOutlierRejectionThreshold\n");
		printf("-i maximumICPIterations\n");
		printf("-l loopdetectiondistance\n");
		//printf("-d threshold_distance_between_indexes\n");
		//printf("-r threshold_determine_correspondences\n");
		//printf("-o threshold_overlap\n");

		return -1;
	}

	std::string input_file_name(argv[1]);
	std::string output_file_name(argv[2]);


	double maxCorrespondenceDistance = 0.1;
	pcl::console::parse_argument (argc, argv, "-d", maxCorrespondenceDistance);

	double RANSACOutlierRejectionThreshold = 0.1;
	pcl::console::parse_argument (argc, argv, "-r", RANSACOutlierRejectionThreshold);

	int maximumICPIterations = 100;
	pcl::console::parse_argument (argc, argv, "-i", maximumICPIterations);

	double loopdetectiondistance = 3.0;
	pcl::console::parse_argument (argc, argv, "-l", loopdetectiondistance);

	pcl::registration::ELCH<PointType> elch;
	pcl::IterativeClosestPoint<PointType, PointType>::Ptr icp (new pcl::IterativeClosestPoint<PointType, PointType>);
	icp->setMaximumIterations (maximumICPIterations);
	icp->setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	icp->setRANSACOutlierRejectionThreshold (RANSACOutlierRejectionThreshold);
	elch.setReg (icp);

	model.loadFile(input_file_name);
	model.getAllScansId(cloudIds);
	CloudVector clouds;
	std::vector<CloudPtr> cloudsBeforeELCH;

	
	std::string dataPath;
	model.getDataSetPath(dataPath);
	modelAfterElch.setDataSetPath(dataPath);



	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		model.getPointcloudName(cloudIds[i], fn);
		modelAfterElch.setPointcloudName(cloudIds[i], fn);
		model.getAffine(cloudIds[i], tr);

		fn= model.getFullPathOfPointcloud(cloudIds[i]);

		CloudPtr pc (new Cloud);
		pcl::io::loadPCDFile (fn, *pc);
		pcl::transformPointCloud(*pc, *pc, tr);
		CloudPtr pc2 (new Cloud);

		*pc2 = *pc;
		cloudsBeforeELCH.push_back(pc2);
		clouds.push_back (CloudPair (cloudIds[i], pc));
		std::cout << "loading file: " << fn << " size: " << pc->size () << std::endl;
		elch.addPointCloud (clouds[i].second);
	}

	int first = 0, last = 0;

	for (size_t i = 0; i < clouds.size (); i++)
	{
		if (loopDetection (int (i), clouds, loopdetectiondistance, first, last))
		{
			std::cout << "Loop between " << first << " (" << clouds[first].first << ") and " << last << " (" << clouds[last].first << ")" << std::endl;
			elch.setLoopStart (first);
			elch.setLoopEnd (last);
			elch.compute ();
		}
	}

	//for (size_t i = 0; i < clouds.size (); i++)
	//{
	//
	//std::string result_filename (clouds[i].first);
	//result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
	//pcl::io::savePCDFileBinary ("elch"+result_filename+".pcd", *(clouds[i].second));
	//std::cout << "saving result to " << "elch"+result_filename<< std::endl;
	//}

	for (size_t i = 0; i < clouds.size (); i++)
	{

		//std::string result_filename (clouds[i].first);
		//result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
		//pcl::io::savePCDFileBinary ("elch"+result_filename+".pcd", *(clouds[i].second));
		//std::cout << "saving result to " << "elch"+result_filename<< std::endl;
		/// compute transformation
		Eigen::Matrix4f tr;
		Eigen::Matrix4f tr2;
		model.getAffine(cloudIds[i], tr2);
		pcl::registration::TransformationEstimationSVD<PointType,PointType> est;
		est.estimateRigidTransformation(*cloudsBeforeELCH[i], *(clouds[i].second), tr);

		Eigen::Matrix4f fin = tr*tr2;
		modelAfterElch.setAffine(cloudIds[i], fin);
	}
	modelAfterElch.saveFile(output_file_name);
	return 0;
}
