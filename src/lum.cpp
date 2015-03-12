#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <iostream>
#include <string>
#include <vector>
#include "dataFramework\data_model.hpp"
#include <Eigen\Eigen>

typedef pcl::PointXYZ PointType;



data_model model;
data_model modelAfterLum;

std::vector<std::string> cloudIds;
std::vector<pcl::PointCloud<PointType>::Ptr> clouds;

Eigen::Vector3f getOrigin (int id )
{
	Eigen::Affine3f mm;
	Eigen::Vector3f origin;
	model.getAffine(cloudIds[id], mm.matrix());
	origin = mm * Eigen::Vector3f(0,0,0);
	return origin;
}


int main (int argc, char **argv)
{

	if(argc < 3)
	{
		printf("usage: lum input.xml output.xml -l 10 -c 0.001 -t 3.0 -d 50 -r 1.5 -o 0.3\n");
		printf("-l lumIter\n");
		printf("-c convergenceThreshold\n");
		printf("-t threshold_diff_norm\n");
		printf("-r threshold_determine_correspondences\n");
		printf("-o threshold_overlap\n");

		return -1;
	}

	std::string input_file_name(argv[1]);
	std::string output_file_name(argv[2]);

	int lumIter = 100;
	double convergenceThreshold = 0.0001;
	double threshold_diff_norm = 3.0;
	double threshold_determine_correspondences =1.5;
	double threshold_overlap = 0.3;

	pcl::console::parse_argument (argc, argv, "-l", lumIter);
	pcl::console::parse_argument (argc, argv, "-c", convergenceThreshold);
	pcl::console::parse_argument (argc, argv, "-t", threshold_diff_norm);
	pcl::console::parse_argument (argc, argv, "-r", threshold_determine_correspondences);
	pcl::console::parse_argument (argc, argv, "-o", threshold_overlap);
	
	std::cout << "threshold_diff_norm = " << threshold_diff_norm << std::endl;
	std::cout << "threshold_determine_correspondences = " << threshold_determine_correspondences << std::endl;
	std::cout << "threshold_overlap = " << threshold_overlap << std::endl;
	std::cout << "lumIter = " << lumIter << std::endl;
	std::cout << "convergenceThreshold = " << convergenceThreshold << std::endl;

	model.loadFile(input_file_name);
	pcl::registration::LUM<PointType> lum;
	lum.setMaxIterations (lumIter);
	lum.setConvergenceThreshold (convergenceThreshold);

	
	std::string dataPath;
	model.getDataSetPath(dataPath);
	modelAfterLum.setDataSetPath(dataPath);

	model.getAllScansId(cloudIds);
	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		model.getPointcloudName(cloudIds[i], fn);
		modelAfterLum.setPointcloudName(cloudIds[i], fn);
		model.getAffine(cloudIds[i], tr);
		
		fn= model.getFullPathOfPointcloud(cloudIds[i]);

		pcl::PointCloud<PointType>::Ptr pc (new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile (fn, *pc);
		pcl::transformPointCloud(*pc, *pc, tr);
		clouds.push_back(pc);
		std::cout << "loading file: " << fn << " size: " << pc->size () << std::endl;
		lum.addPointCloud (clouds[i]);
	}


	for (int i=0; i< clouds.size(); i++)
	{
		for (int j=0; j< i; j++)
		{
			Eigen::Vector3f origin1 = getOrigin (i);
			Eigen::Vector3f origin2 = getOrigin (j);
			Eigen::Vector3f diff = origin1 - origin2;
			float dist =diff.norm();
	
			if(diff.norm () < threshold_diff_norm)
			{
			  pcl::registration::CorrespondenceEstimation<PointType, PointType> ce;
			  ce.setInputTarget (clouds[i]);
			  ce.setInputCloud (clouds[j]);
			  pcl::CorrespondencesPtr corr (new pcl::Correspondences);
			  ce.determineCorrespondences (*corr, threshold_determine_correspondences);

			  if (corr->size () > (float(clouds[j]->size()) * threshold_overlap) )
			  {				  
				lum.setCorrespondences (j, i, corr);
				std::cout << "add connection between " << i << " (" << cloudIds[i] << ") and " << j << " (" << cloudIds[j] << ")" << std::endl;
			  }
			}

		}
	}

	lum.compute ();

	for(size_t i = 0; i < lum.getNumVertices (); i++)
	{
		//std::cout << i << ": " << lum.getTransformation (i) (0, 3) << " " << lum.getTransformation (i) (1, 3) << " " << lum.getTransformation (i) (2, 3) << std::endl;
		Eigen::Affine3f tr2;
		model.getAffine(cloudIds[i], tr2.matrix());
		Eigen::Affine3f tr = lum.getTransformation(i);
		
		Eigen::Affine3f final = tr * tr2;
		modelAfterLum.setAffine(cloudIds[i], final.matrix());
	}
	
	modelAfterLum.saveFile(output_file_name);

	return 0;
}