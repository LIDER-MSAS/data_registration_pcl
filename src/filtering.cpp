#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework\data_model.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/time.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int		MeanK=50;
float	StddevMulThresh=1.0f;

int main (int argc, char** argv)
{
	std::cout << "Usage:\n";
	std::cout << argv[0] << " input_model.xml output_model.xml parameters\n";
	std::cout << " -k <number>\t\tSet the number of nearest neighbors to use for mean distance estimation. Default: " << MeanK << std::endl;
	std::cout << " -s <multiplier>\tSet the standard deviation multiplier for the distance threshold calculation. Default: " << StddevMulThresh << std::endl;

	if(argc<3)
	{
		return -1;
	}

	std::string param_inputModel = argv[1];
	std::string param_outputModel = argv[2];
	std::string param_MeanK;
	std::string param_StddevMulThresh;
	

	int _k;
	float _s;
	if(pcl::console::parse_argument (argc, argv, "-k", _k)!=-1)
	{
		MeanK=_k;
	}
	if(pcl::console::parse_argument (argc, argv, "-s", _s)!=-1)
	{
		StddevMulThresh=_s;
	}
	
	//Generate text from values of parameters
	std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
	ss << StddevMulThresh;
	param_StddevMulThresh=ss.str();
	//Reset the string to be empty
	ss.str("");
	//clear any error flags that may be set
	ss.clear();
	ss << MeanK;
	param_MeanK=ss.str();
	
	std::cout << "number of NN to use for mean distance estimation = " << param_MeanK << "\n";
	std::cout << "standard deviation multiplier = " << param_StddevMulThresh << "\n";

	/// models
	data_model inputModel;
	data_model outputModel;

	if(!inputModel.loadFile(param_inputModel))
	{
		std::cout << "Error loading: " << param_inputModel << std::endl;
		return -2;
	}
	std::cout << "Loaded: " << param_inputModel << " correctly.\n";

	/// create new path for data
	boost::filesystem::path pathinputXML(param_inputModel);
	boost::filesystem::path pathouputXML(param_outputModel);
	boost::filesystem::path pathOfNewDataDirectory = pathouputXML.parent_path();

	
	//generate absolute path
	std::string relativePathToData = "data_filtered_" + param_MeanK + "_" + param_StddevMulThresh;
	pathOfNewDataDirectory/=relativePathToData;

	// create new directory for data
	std::cout <<"creating directory with path :" << pathOfNewDataDirectory <<"\n";
	if(!boost::filesystem::create_directories(pathOfNewDataDirectory))
	{
		std::cout<<"Could not create dir: "<< pathOfNewDataDirectory << std::endl;
		return -3;
	}

	// save relative path to model's data
	outputModel.setDataSetPath(relativePathToData);

	// loads clouds ids in input model
	std::vector <std::string> cloud_ids;
	inputModel.getAllScansId(cloud_ids);

	// sets some info about algorithm
	outputModel.setAlgorithmName("Statistical Outlier Removal");
	outputModel.addAlgorithmParam("MeanK",  param_MeanK);
	outputModel.addAlgorithmParam("StddevMulThresh",  param_StddevMulThresh);
	
	std::cout <<"pointclouds count: "<< cloud_ids.size() <<"\n";

	double totalTime=0;

	for (int i=0; i < cloud_ids.size(); i++)
	{
		//we take full path of pointcloud in input model ...
		boost::filesystem::path inputFn (inputModel.getFullPathOfPointcloud(cloud_ids[i]));
		// ... and get only filename for ouput
		std::string outputFn = (pathOfNewDataDirectory/(inputFn.filename())).string();

		pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>());
		std::cout <<"loading pointcloud:" << inputFn<<"\n";
		pcl::io::loadPCDFile(inputFn.string(), *inputCloud);

		// Create the filtering object
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (inputCloud);
		sor.setMeanK (MeanK);
		sor.setStddevMulThresh (StddevMulThresh);

		//Measure the computation time
		pcl::StopWatch sw;
		sw.reset();
		sor.filter (*outputCloud);
		double exTime = sw.getTime();

		//Increase total computation time
		totalTime+=exTime;

		std::cout <<"saving pointcloud :" << outputFn<<"\n";
		pcl::io::savePCDFile(outputFn, *outputCloud);

		//rewrite some data from input model to ouput model -
		///TODO copying
		Eigen::Matrix4f tr;
		inputModel.getAffine(cloud_ids[i], tr);
		outputModel.setAffine(cloud_ids[i], tr);
		outputModel.setPointcloudName(cloud_ids[i], inputFn.filename().string());

		//save results
		outputModel.setResult(cloud_ids[i], "size_before_filtering",inputCloud->height*inputCloud->width);
		outputModel.setResult(cloud_ids[i], "size_after_filtering",outputCloud->height*outputCloud->width);
		outputModel.setResult(cloud_ids[i], "computation_time", exTime);
		outputModel.setResult(cloud_ids[i], "cummulative_filter_time", totalTime);

	}
	

	outputModel.saveFile(param_outputModel);

}