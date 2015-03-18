
#include<pcl/visualization/pcl_plotter.h>

#include<iostream>
#include<vector>
#include<utility>
#include<math.h>  
#include"dataFramework/data_model.hpp"
using namespace std;
using namespace pcl::visualization;



void createPlot (data_model &tr1, std::string resultName,std::vector<double> &x, std::vector<double> &y, double &maxFitnessScore)
{
	std::vector<std::string> ids;
	tr1.getAllScansId(ids);

	x.resize(ids.size());
	y.resize(ids.size());

	//double maxFitnessScore = -1000.0;

	for (int i =0 ; i < ids.size(); i++)
	{
		x[i]=i;
		float t =0.0f;
		tr1.getResult(ids[i],resultName,t);
		y[i] = t;
		if(t > maxFitnessScore)maxFitnessScore = t;
	}

	//p->setXRange(0,ids.size());
  //p->setYRange(-50,50);
}


void addTrajectory(std::string xmlName, std::string resultName, PCLPlotter * p)
{
  data_model tr1;
  tr1.loadFile(xmlName);
  std::vector<double> ax;
  std::vector<double> ay;
  double maxFitnessScore = -1000.0;
  createPlot(tr1,resultName, ax,ay, maxFitnessScore);

  //addPlotData (std::vector< double > const &array_x, std::vector< double >const &array_y, char const *name="Y Axis", int type=vtkChart::LINE, std::vector< char > const &color=std::vector< char >())
  p->addPlotData(ax,ay,xmlName.c_str());
  p->setXRange(0,ax.size());
	p->setYRange(0,maxFitnessScore);

  
}
int main (int argc, char * argv [])
{
  

  std::cout <<"usage:\n";
  std::cout << argv[0] <<" MODEL1.XML MODEL2.XML paramName\n";

  PCLPlotter *plotter = new PCLPlotter ("Trayectory plotter");
  std::string paramName = argv[argc-1];
  plotter->setShowLegend (true);
  std::cout << "argc " << argc << "\n";
  for (int i=1; i < argc-1 ; i++)
  {
	  std::string model = argv[i];
	  std::cout << "loading model " << model << "\n";
	  addTrajectory(model,paramName,plotter);
  }

  plotter->spin();

  return 1;
}
