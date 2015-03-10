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


void addTrayectory(std::string xmlName, PCLPlotter * p)
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


int main (int argc, char * argv [])
{
	if(argc < 2)
	{
		printf("usage: graph_viewer xxx.xml -t 3.0 -d 50 -c 1.5 -o 0.3\n");
		printf("-t threshold_diff_norm\n");
		printf("-d threshold_distance_between_indexes\n");
		printf("-c threshold_determine_correspondences\n");
		printf("-o threshold_overlap\n");

		return -1;
	}
	
	PCLPlotter *plotter = new PCLPlotter ("My Plotter");
    plotter->setShowLegend (true);

	double threshold_diff_norm = 3.0;
	pcl::console::parse_argument (argc, argv, "-t", threshold_diff_norm);

	int threshold_distance_between_indexes = 50;
	pcl::console::parse_argument (argc, argv, "-d", threshold_distance_between_indexes);
	
	double threshold_determine_correspondences =1.5;
	pcl::console::parse_argument (argc, argv, "-c", threshold_determine_correspondences);

	double threshold_overlap = 0.3;
	pcl::console::parse_argument (argc, argv, "-o", threshold_overlap);
	
	model.loadFile(argv[1]);

	pcl::registration::LUM<PointType> lum;
	//lum.setMaxIterations (lumIter);
	//lum.setConvergenceThreshold (convergenceThreshold);

	model.getAllScansId(cloudIds);
	for (size_t i = 0; i < cloudIds.size (); i++)
	{
		std::string fn;
		Eigen::Matrix4f tr;

		model.getPointcloudName(cloudIds[i], fn);
		fn = model.getFullPathOfPointcloud(cloudIds[i]);
		model.getAffine(cloudIds[i], tr);

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
				//lum.setCorrespondences (j, i, corr);
				std::cout << "add connection between " << i << " (" << cloudIds[i] << ") and " << j << " (" << cloudIds[j] << ")" << std::endl;

				///////////dodajemy origines////////////
				double x1 = origin1.x();
				double y1 = origin1.y();
				double x2 = origin2.x();
				double y2 = origin2.y();

				addEdgeToPlot(plotter, x1, y1, x2 ,y2, i, j);
			  }
			}

		}
	}


	printf("-t threshold_diff_norm %f\n", threshold_diff_norm);
	printf("-d threshold_distance_between_indexes %d\n", threshold_distance_between_indexes);
	printf("-c threshold_determine_correspondences %f\n", threshold_determine_correspondences);
	printf("-o threshold_overlap %f\n", threshold_overlap);

	plotter->spin();
	
     return 1;
}