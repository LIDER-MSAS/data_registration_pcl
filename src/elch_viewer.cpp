#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  //for abs()
#include"dataFramework\data_model.hpp"

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/lum.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/registration/elch.h>
#include <Eigen\Eigen>

#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;

using namespace std;
using namespace pcl::visualization;


typedef pcl::PointXYZ PointType;

data_model model;
data_model modelAfterLum;
data_model modelAfterElch;

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

void createTrayectory (data_model &tr1, std::vector<double> &x, std::vector<double> &y)
{
	std::vector<std::string> ids;
	tr1.getAllScansId(ids);

	x.resize(ids.size());
	y.resize(ids.size());

	for (int i =0 ; i < ids.size(); i++)
	{
		Eigen::Vector3f f = Eigen::Vector3f(0,0,0);
		Eigen::Affine3f affine;

		tr1.getAffine(ids[i],affine.matrix());
		Eigen::Vector3f out = affine * f;
		//std::cout << out.x()<<"\t" << out.y()<<"\n";

		x[i]=static_cast<double>(out.x());
		y[i]=static_cast<double>(out.y());

	}
}

void addTrajectory(std::string xmlName, PCLPlotter * p)
{
	data_model tr1;
	tr1.loadFile(xmlName);
	std::vector<double> ax;
	std::vector<double> ay;
	createTrayectory(tr1, ax,ay);

	//addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())
	p->addPlotData(ax,ay,xmlName.c_str());

	p->setXRange(-50,50);
	p->setYRange(-50,50);


}

void addEdgeToPlot(PCLPlotter * p, double x1, double y1, double x2, double y2, int index_i, int index_j)
{
	std::vector<double> ax;
	std::vector<double> ay;

	ax.push_back(x1);
	ax.push_back(x2);

	ay.push_back(y1);
	ay.push_back(y2);

	//addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())

	stringstream ss;
	ss << "edge: "<< index_i << "(" << x1 << "," << y1 << ")" << " " << index_j << "(" << x2 << "," << y2 << ")";
	string str = ss.str();

	p->addPlotData(ax,ay,str.c_str());

	p->setXRange(-50,50);
	p->setYRange(-50,50);
}

bool loopDetection (int end, const CloudVector &clouds, double dist, int &first, int &last)
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

int main (int argc, char * argv [])
{
	double maxCorrespondenceDistance = 0.1;
	double RANSACOutlierRejectionThreshold = 0.1;
	int maximumICPIterations = 100;
	double loopdetectiondistance = 3.0;

	if(argc < 2)
	{
		std::cout << "Usage:\n";
		std::cout << argv[0] <<" inputModel.xml parameters\n";
		std::cout << " -d\tSets the maximum distance threshold between two correspondent points in source <-> target.\tDefault: " << maxCorrespondenceDistance << std::endl;
		std::cout << " -r\tSets the inlier distance threshold for the internal RANSAC outlier rejection loop.\tDefault: " << RANSACOutlierRejectionThreshold << std::endl;
		std::cout << " -i\tSets the maximum number of iterations the internal optimization should run for.\tDefault: " << maximumICPIterations << std::endl;
		std::cout << " -l\tSets the maximum distance between scans to consider a loop.\tDefault: " << loopdetectiondistance << std::endl;

		return -1;
	}

	PCLPlotter *plotter = new PCLPlotter ("ELCH Viewer");
	plotter->setShowLegend (true);
	
	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");
	
	if(xml_indices.size()!=1)
	{
		return -2;
	}

	std::string input_file_name(argv[xml_indices[0]]);

	addTrajectory(input_file_name, plotter);
	
	pcl::console::parse_argument (argc, argv, "-d", maxCorrespondenceDistance);
	pcl::console::parse_argument (argc, argv, "-r", RANSACOutlierRejectionThreshold);
	pcl::console::parse_argument (argc, argv, "-i", maximumICPIterations);
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
	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		fn = model.getFullPathOfPointcloud(cloudIds[i]);
		modelAfterElch.setPointcloudName(cloudIds[i], fn);
		model.getAffine(cloudIds[i], tr);

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
			Eigen::Vector3f origin1 = getOrigin (first);
			Eigen::Vector3f origin2 = getOrigin (last);

			addEdgeToPlot(plotter, origin1.x(), origin1.y(), origin2.x(), origin2.y(), first, last);

			elch.compute ();
		}
	}

	plotter->spin();

	return 1;
}