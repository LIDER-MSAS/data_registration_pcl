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

float       default_leaf_size = 0.01f;
std::string default_field ("z");
double      default_filter_min = -std::numeric_limits<double>::max ();
double      default_filter_max = std::numeric_limits<double>::max ();

int main (int argc, char** argv)
{
  print_info ("Downsample a model using pcl::VoxelGrid \n USAGE: filtering leaf_size input_model output_model\n");

  std::string param_leafSize = argv[1];
  std::string param_inputModel = argv[2];
  std::string param_outputModel = argv[3];

  float leafSize = boost::lexical_cast<float, std::string>(param_leafSize);
  /// models
  data_model inputModel;
  data_model outputModel;
  inputModel.loadFile(param_inputModel);

  /// create new path for data
  boost::filesystem::path pathinputXML(param_inputModel);
  boost::filesystem::path pathouputXML(param_outputModel);

  boost::filesystem::path pathOfNewDataFolder = pathinputXML.parent_path();
  std::string relativePathToData = "data_subsampled"+boost::lexical_cast<std::string,float>(leafSize);
  pathOfNewDataFolder/=relativePathToData;

  /// create new folder for data
  std::cout <<"creating folder with path :" << pathOfNewDataFolder <<"\n";
  boost::filesystem::create_directory(pathOfNewDataFolder);

  /// save relative path to model's data
  outputModel.setDataSetPath(relativePathToData);
  
  /// loads clouds ids in input model
  std::vector <std::string> cloud_ids;
  inputModel.getAllScansId(cloud_ids);

  /// sets some info about algorithm
  outputModel.setAlgorithmName("Voxel grid subsampling");
  outputModel.addAlgorithmParam("leaf_size",  param_leafSize);

  std::cout <<"pointclouds count to filter "<< cloud_ids.size() <<"\n";
  for (int i=0; i < cloud_ids.size(); i++)
  {
	  //we take full path of pointcloud in input model ...
	  boost::filesystem::path inputFn (inputModel.getFullPathOfPointcloud(cloud_ids[i]));
	  // ... and get only filename for ouput
	  std::string outputFn = (pathOfNewDataFolder/(inputFn.filename())).string();

	  pcl::PCLPointCloud2::Ptr inputCloud (new pcl::PCLPointCloud2());
	  pcl::PCLPointCloud2::Ptr outputCloud (new pcl::PCLPointCloud2());
	  std::cout <<"loading pointcloud :" << inputFn<<"\n";
	  pcl::io::loadPCDFile(inputFn.string(), *inputCloud);

	  VoxelGrid<pcl::PCLPointCloud2> grid;
	  grid.setInputCloud (inputCloud);
	  grid.setLeafSize (leafSize, leafSize, leafSize);

	  pcl::StopWatch sw;
	  sw.reset();
	  grid.filter (*outputCloud);
	  double exTime = sw.getTime();
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

  }
  /// set new data repository path (relative to output model)
  
  outputModel.saveFile(param_outputModel);

}