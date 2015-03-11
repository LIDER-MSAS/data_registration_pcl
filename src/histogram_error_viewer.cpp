
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "dataFramework\data_model.hpp"

typedef pcl::PointXYZ PointT;
pcl::visualization::PCLVisualizer p;
std::vector<pcl::PointCloud<PointT>::Ptr> pcs;
std::vector<std::string> pcsName;


int currentlyHighlitedScan =0;

void createMetascan (data_model &model, pcl::PointCloud<PointT> &pc)
{
	std::cout <<"creating metascan\n";
	std::vector<std::string> ids;
	model.getAllScansId(ids);

	for (int i=0; i < ids.size();i ++)
	{
		std::string fn = model.getFullPathOfPointcloud(ids[i]);
		std::cout <<"loading "<<fn <<"\n";
		Eigen::Matrix4f tr;
		model.getAffine(ids[i], tr);
		pcl::PointCloud<PointT> tmp;
		pcl::io::loadPCDFile(fn, tmp);
		pcl::transformPointCloud(tmp,tmp,tr);
		pc +=tmp;
	}
}



int main (int argc, char** argv)
{
	std::string modelSourceFn = argv[1];
	std::string modelTargetFn = argv[2];

	std::cout <<"input model " << modelSourceFn <<"\n";
	std::cout <<"output model " << modelTargetFn <<"\n";


	data_model modelSource;
	data_model modelTarget;

	modelSource.loadFile(modelSourceFn);
	modelTarget.loadFile(modelTargetFn);

	pcl::PointCloud<PointT> metascanSource;
	pcl::PointCloud<PointT> metascanTarget;

	createMetascan(modelSource, metascanSource);
	createMetascan(modelTarget, metascanTarget);

	pcl::io::savePCDFile ("metaSource.pcd", metascanSource);
	pcl::io::savePCDFile ("metaTarget.pcd", metascanTarget);


	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanSource_color_handler (metascanSource.makeShared(), 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanTarget_color_handler (metascanTarget.makeShared(), 0, 255, 0);


	p.addPointCloud<PointT>(metascanSource.makeShared(),metascanSource_color_handler,"sorce");
	p.addPointCloud<PointT>(metascanTarget.makeShared(),metascanTarget_color_handler,"target");
	

	p.spin();

	return 0;
}

