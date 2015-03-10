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

float       default_leaf_size = 0.01f;
std::string default_field ("z");
double      default_filter_min = -std::numeric_limits<double>::max ();
double      default_filter_max = std::numeric_limits<double>::max ();

int main (int argc, char** argv)
{
  print_info ("Filtering a model using pcl::StatisticalOutlierRemoval \n USAGE: filtering MeanK StddevMulThresh input_model output_model\n");
  
  std::string param_MeanK = argv[1];
  std::string param_StddevMulThresh = argv[2];
  std::string param_inputModel = argv[3];
  std::string param_outputModel = argv[4];
  
  int MeanK = boost::lexical_cast<int, std::string>(param_MeanK);
  float StddevMulThresh = boost::lexical_cast<float, std::string>(param_StddevMulThresh);
  /// models
  data_model inputModel;
  data_model outputModel;
  inputModel.loadFile(param_inputModel);

  /// create new path for data
  boost::filesystem::path pathinputXML(param_inputModel);
  boost::filesystem::path pathouputXML(param_outputModel);

  boost::filesystem::path pathOfNewDataDirectory = pathinputXML.parent_path();
  std::string relativePathToData = "data_filtered_"+param_MeanK+"_"+param_StddevMulThresh;
  pathOfNewDataDirectory/=relativePathToData;

  /// create new directory for data
  std::cout <<"creating directory with path :" << pathOfNewDataDirectory <<"\n";
  boost::filesystem::create_directory(pathOfNewDataDirectory);

  /// save relative path to model's data
  outputModel.setDataSetPath(relativePathToData);
  
  /// loads clouds ids in input model
  std::vector <std::string> cloud_ids;
  inputModel.getAllScansId(cloud_ids);

  /// sets some info about algorithm
  outputModel.setAlgorithmName("Statistical Outlier Removal");
  outputModel.addAlgorithmParam("MeanK",  param_MeanK);
  outputModel.addAlgorithmParam("StddevMulThresh",  param_StddevMulThresh);

  std::cout <<"pointclouds count to filter "<< cloud_ids.size() <<"\n";

  for (int i=0; i < cloud_ids.size(); i++)
  {
	  //we take full path of pointcloud in input model ...
	  boost::filesystem::path inputFn (inputModel.getFullPathOfPointcloud(cloud_ids[i]));
	  // ... and get only filename for ouput
	  std::string outputFn = (pathOfNewDataDirectory/(inputFn.filename())).string();

	  pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud (new pcl::PointCloud<pcl::PointXYZ>());
	  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud (new pcl::PointCloud<pcl::PointXYZ>());
	  std::cout <<"loading pointcloud :" << inputFn<<"\n";
	  pcl::io::loadPCDFile(inputFn.string(), *inputCloud);

	  // Create the filtering object
	  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	  sor.setInputCloud (inputCloud);
	  sor.setMeanK (MeanK);
	  sor.setStddevMulThresh (StddevMulThresh);

	  pcl::StopWatch sw;
	  sw.reset();
	  sor.filter (*outputCloud);
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
	  outputModel.setResult(cloud_ids[i], "size_before_filtering",inputCloud->height*inputCloud->width);
	  outputModel.setResult(cloud_ids[i], "size_after_filtering",outputCloud->height*outputCloud->width);
	  outputModel.setResult(cloud_ids[i], "execution_time", exTime);

  }
  /// set new data repository path (relative to output model)
  
  outputModel.saveFile(param_outputModel);

}