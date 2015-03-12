#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework\data_model.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/time.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

float       leaf_size = 0.1f;
std::string default_field ("z");
double      default_filter_min = -std::numeric_limits<double>::max ();
double      default_filter_max = std::numeric_limits<double>::max ();

int main (int argc, char** argv)
{
	//print_info ("Downsample a model using pcl::VoxelGrid \n USAGE: subsampling leaf_size input_model output_model\n");

	std::cout << "Usage:\n";
	std::cout << argv[0] << " input_model.xml output_model.xml parameters\n";
	std::cout << " -l <leaf_size>\t\tSet the voxel grid leaf size. Default: " << leaf_size << std::endl;
	
	if(argc<3)
	{
		return -1;
	}

	std::string param_inputModel = argv[1];
	std::string param_outputModel = argv[2];
	std::string param_leaf_size;

	pcl::console::parse_argument (argc, argv, "-l", leaf_size);

	//Generate text from values of parameters
	std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
	ss << leaf_size;
	param_leaf_size=ss.str();

	std::cout << "voxel grid leaf size = " << param_leaf_size << "\n";

	/// models
	data_model inputModel;
	data_model outputModel;
	inputModel.loadFile(param_inputModel);
	
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
	std::string relativePathToData = "data_subsampled_"+param_leaf_size;
	pathOfNewDataDirectory/=relativePathToData;

	/// create new directory for data
	std::cout <<"creating directory (if does not exist):" << pathOfNewDataDirectory << std::endl;
	if(!boost::filesystem::create_directories(pathOfNewDataDirectory))
	{
		if(!boost::filesystem::is_directory(pathOfNewDataDirectory))
		{
			std::cout<<"Could not create dir: "<< pathOfNewDataDirectory << std::endl;
			return -3;
		}
	}

	/// save relative path to model's data
	outputModel.setDataSetPath(relativePathToData);
	
	/// loads clouds ids in input model
	std::vector <std::string> cloud_ids;
	inputModel.getAllScansId(cloud_ids);
	
	/// sets some info about algorithm
	outputModel.setAlgorithmName("Voxel grid subsampling");
	outputModel.addAlgorithmParam("leaf_size",  param_leaf_size);
	
	std::cout <<"pointclouds count: "<< cloud_ids.size() <<"\n";
	
	double totalTime=0;
	
	for (int i=0; i < cloud_ids.size(); i++)
	{
		//we take full path of pointcloud in input model ...
		boost::filesystem::path inputFn (inputModel.getFullPathOfPointcloud(cloud_ids[i]));
		// ... and get only filename for ouput
		std::string outputFn = (pathOfNewDataDirectory/(inputFn.filename())).string();

		pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2());
		pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2());
		std::cout <<"loading pointcloud:" << inputFn<<"\n";
		pcl::io::loadPCDFile(inputFn.string(), *inputCloud);

		VoxelGrid<pcl::PCLPointCloud2> grid;
		grid.setInputCloud (inputCloud);
		grid.setLeafSize (leaf_size, leaf_size, leaf_size);

		pcl::StopWatch sw;
		sw.reset();
		grid.filter (*outputCloud);
		double exTime = sw.getTime();

		std::cout << "Size before: " << inputCloud->height*inputCloud->width << " Size after: "<<outputCloud->height*outputCloud->width << std::endl;

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
		outputModel.setResult(cloud_ids[i], "size_before_subsample",inputCloud->height*inputCloud->width);
		outputModel.setResult(cloud_ids[i], "size_after_subsample",outputCloud->height*outputCloud->width);
		outputModel.setResult(cloud_ids[i], "execution_time", exTime);
		outputModel.setResult(cloud_ids[i], "cummulative_filter_time", totalTime);

	}


	outputModel.saveFile(param_outputModel);

}