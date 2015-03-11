// icp_overlap_viewer
#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include "dataFramework\data_model.hpp"
#include <pcl/registration/icp.h>

//#define ICP_OVERLAP1
#define ICP_OVERLAP2

#ifdef ICP_OVERLAP1

typedef pcl::PointXYZ PointT;
pcl::visualization::PCLVisualizer p;
std::vector<pcl::PointCloud<PointT>::Ptr> pcs;
std::vector<std::string> pcsName;
data_model dSets;
int currentlyHighlitedScan =0;

template <typename PointSource, typename PointTarget, typename Scalar = float> class myIcp : public  pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >
{
public:
	pcl::CorrespondencesPtr getCorrespondeces()
	{
		return correspondences_;
	}
};
 
 
void loadPointcloud (pcl::PointCloud<PointT> &scan, std::string id)
{
	std::string fn = dSets.getFullPathOfPointcloud(id);
	std::cout <<"file name "<< fn << "\n";
	pcl::io::loadPCDFile(fn, scan);
	Eigen::Matrix4f mat;
	dSets.getAffine(id, mat);
	std::cout <<"marix transform: \n "<< mat << "\n";
	pcl::transformPointCloud(scan,scan,mat);
}
 
int main (int argc, char** argv)
{
	if(argc < 3)
	{
		printf("usage: icp_overlap_viewer.exe input.xml scan_id1 scan_id2 -d 0.5 -r 0.1\n");
		printf("-d maxCorrespondenceDistance\n");
		printf("-r RANSACOutlierRejectionThreshold\n");
	}

	double maxCorrespondenceDistance = 0.5;
	pcl::console::parse_argument (argc, argv, "-d", maxCorrespondenceDistance);

	double RANSACOutlierRejectionThreshold = 0.1;
	pcl::console::parse_argument (argc, argv, "-r", RANSACOutlierRejectionThreshold);
	/// loadpointclouds
 
	std::string model_file = argv[1];
	std::string scan_id1 = argv[2];
	std::string scan_id2 = argv[3];
	std::cout << model_file <<"\n";
	std::cout << scan_id1 <<"\n";
	std::cout << scan_id2 <<"\n";
 
	dSets.loadFile(model_file);
	p.setWindowName(model_file);
	
	pcl::PointCloud<PointT> scan1;
	pcl::PointCloud<PointT> scan2;
	pcl::PointCloud<PointT> scanAlign;
 
	loadPointcloud (scan1, scan_id1);
	loadPointcloud (scan2, scan_id2);
 
	pcl::visualization::PointCloudColorHandlerCustom<PointT> handler_scan1 (scan1.makeShared(), 0,0,255);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> handler_scan2 (scan2.makeShared(), 255,0,0);
 
	
	myIcp<PointT, PointT> *icp;
	icp = new myIcp<PointT, PointT>();
 
	icp->setMaximumIterations (0);
	icp->setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	icp->setRANSACOutlierRejectionThreshold (RANSACOutlierRejectionThreshold);
 
	icp->setInputTarget (scan1.makeShared());
	icp->setInputSource (scan2.makeShared());
 
	icp->align(scanAlign);
 
	pcl::Correspondences corr = *(icp->getCorrespondeces());
	p.addPointCloud(scan1.makeShared(), handler_scan1 , "scan1");
	p.addPointCloud(scan2.makeShared(), handler_scan2 , "scan2");
	for (int i=0; i< corr.size(); i++)
	{
		int i1= corr[i].index_match;
		int i2= corr[i].index_query;
		std::stringstream ss;
		ss << i;
		if (i1 >0 && i2)
		{
			p.addLine<PointT> (scan1.points[i1], scan2.points[i2], ss.str());
		}
	}
	std::cout << "correspondeces size " << corr.size()<< "\n";
	std::cout << "size scan1 " << scan1.size()<< "\n";
	std::cout << "size scan2 " << scan2.size()<< "\n";
	float overlap = 100.0f*corr.size()/scan2.size();
	std::stringstream ss;
	ss << std::setprecision(2) << overlap;
	std::cout << "overlap : " << ss.str() << " %\n";
	
	p.spin();
 
	return 0;
}

#endif

#ifdef ICP_OVERLAP2

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

#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  //for abs()
#include"dataFramework\data_model.hpp"
#include <Eigen\Eigen>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/lum.h>

typedef pcl::PointXYZ PointType;
using namespace std;
using namespace pcl::visualization;


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

void addEdgeToPlot(PCLPlotter * p, int index_i, int index_j, float overlap)
{
  std::vector<double> ax;
  std::vector<double> ay;
  
  ax.push_back(index_i);
  ax.push_back(index_j);
	
  ay.push_back(overlap);
  ay.push_back(overlap);


  //addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())
  
  stringstream ss;
  ss << "overlap: "<< index_i << "-" << index_j << ":" << overlap;
	string str = ss.str();


	p->addPlotData(ax,ay,str.c_str());
  
  //p->setXRange(-50,50);
  //p->setYRange(-50,50);
}

int main (int argc, char **argv)
{
	
	if(argc < 3)
	{
		printf("usage: icp_overlap_viewer.exe input.xml -d 0.5\n");
		printf("-d threshold_determine_correspondences\n");
		

		return -1;
	}

	PCLPlotter *plotter = new PCLPlotter ("My Plotter");
    plotter->setShowLegend (true);


	std::string input_file_name(argv[1]);
	double threshold_determine_correspondences = 0.5;
	pcl::console::parse_argument (argc, argv, "-d", threshold_determine_correspondences);



	model.loadFile(input_file_name);
	//pcl::registration::LUM<PointType> lum;
	//lum.setMaxIterations (0);
	//lum.setConvergenceThreshold (0);



	model.getAllScansId(cloudIds);
	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		//model.getPointcloudName(cloudIds[i], fn);
		fn = model.getFullPathOfPointcloud(cloudIds[i]);

		modelAfterLum.setPointcloudName(cloudIds[i], fn);
		model.getAffine(cloudIds[i], tr);

		pcl::PointCloud<PointType>::Ptr pc (new pcl::PointCloud<PointType>);
		pcl::io::loadPCDFile (fn, *pc);
		pcl::transformPointCloud(*pc, *pc, tr);
		clouds.push_back(pc);
		std::cout << "loading file: " << fn << " size: " << pc->size () << std::endl;
		//lum.addPointCloud (clouds[i]);
	}


	for (int i=1; i< clouds.size(); i++)
	{
		int j = i - 1;
		
			  pcl::registration::CorrespondenceEstimation<PointType, PointType> ce;
			  ce.setInputTarget (clouds[i]);
			  ce.setInputCloud (clouds[j]);
			  pcl::CorrespondencesPtr corr (new pcl::Correspondences);
			  ce.determineCorrespondences (*corr, threshold_determine_correspondences);

			  printf("i:%d j:%d corr: %d clouds[i]:%d clouds[j]:%d overlap:%.2f\n", i,j, corr->size (), clouds[i]->size(), clouds[j]->size(), float(corr->size ())/float(clouds[j]->size())*100.0);
			  addEdgeToPlot(plotter,i, j, float(corr->size ())/float(clouds[j]->size())*100.0);
	}

	plotter->setXRange(0,clouds.size());
    plotter->setYRange(0,100);
	plotter->spin();

	return 0;
}

#endif