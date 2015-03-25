
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
	ss << "overlap: "<<cloudIds[ index_j] << " - " << cloudIds[index_i]<< " :" <<std::setprecision(2) <<  overlap;
	string str = ss.str();
	p->addPlotData(ax,ay,str.c_str(),vtkChart::BAR);

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