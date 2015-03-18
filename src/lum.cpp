#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <iostream>
#include <string>
#include <vector>
#include "dataFramework/data_model.hpp"
#include <Eigen/Eigen>

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
	int lumIter = 100;
	double convergenceThreshold = 0.000001;
	double threshold_diff_norm = 10;
	double threshold_determine_correspondences = 0.3;
	double threshold_overlap = 0.6;

	if(argc < 3)
	{
		std::cout << "Usage:\n";
		std::cout << argv[0] << "input.xml output.xml parameters\n";
		std::cout << " -i\tSets the maximum number of LUM iterations. Deafult: "<<lumIter<<std::endl;
		std::cout << " -c\tSets the convergence threshold. Use -c 0 to do maximum number of iterations. Deafult: "<<convergenceThreshold<<std::endl;
		std::cout << " -d\tSets the maximum distance between scans to consider a link. Deafult: "<<threshold_diff_norm<<std::endl;
		std::cout << " -r\tSets the radius for the nearest neighbor search. Deafult: "<<threshold_determine_correspondences<<std::endl;
		std::cout << " -o\tSets the minimum overlap between scans to consider a link. -o 0.6 means 60% overlap. Deafult: "<<threshold_overlap<<std::endl;

		return -1;
	}

	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");

	if(xml_indices.size()!=2)
	{
		return -2;
	}

	std::string input_file_name(argv[xml_indices[0]]);
	std::string output_file_name(argv[xml_indices[1]]);


	pcl::console::parse_argument (argc, argv, "-i", lumIter);
	pcl::console::parse_argument (argc, argv, "-c", convergenceThreshold);
	pcl::console::parse_argument (argc, argv, "-d", threshold_diff_norm);
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


	for (int i=0; i<clouds.size(); i++)
	{
		for (int j=0; j<i; j++)
		{
			Eigen::Vector3f origin1 = getOrigin (i);
			Eigen::Vector3f origin2 = getOrigin (j);
			Eigen::Vector3f diff = origin1 - origin2;
			float dist = diff.norm();

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

	lum.compute();

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
