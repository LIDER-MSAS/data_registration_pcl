/*
 * Software License Agreement (BSD License)
 *
 *  Data Registration Framework - Mobile Spatial Assistance System
 *  Copyright (c) 2014-2015, Institute of Mathematical Machines
 *  http://lider.zms.imm.org.pl/
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Institute of Mathematical Machines nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework/data_model.hpp>
#include <boost/filesystem.hpp>
#include <pcl/common/time.h>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int		MeanK=50;
float	StddevMulThresh=1.0f;

int main (int argc, char** argv)
{
	if(argc<3)
	{
		std::cout << "Usage:\n";
		std::cout << argv[0] << " input_model.xml output_model.xml parameters\n";
		std::cout << " -k <number>\t\tSet the number of nearest neighbors to use for mean distance estimation. Default: " << MeanK << std::endl;
		std::cout << " -s <multiplier>\tSet the standard deviation multiplier for the distance threshold calculation. Default: " << StddevMulThresh << std::endl;

		return -1;
	}

	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");

	if(xml_indices.size()!=2)
	{
		return -2;
	}


	std::string param_inputModel = argv[xml_indices[0]];
	std::string param_outputModel = argv[xml_indices[1]];
	std::string param_MeanK;
	std::string param_StddevMulThresh;
	

	pcl::console::parse_argument (argc, argv, "-k", MeanK);
	pcl::console::parse_argument (argc, argv, "-s", StddevMulThresh);
	
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
		return -3;
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
	std::cout <<"creating directory (if does not exist):" << pathOfNewDataDirectory <<"\n";
	if(!boost::filesystem::create_directories(pathOfNewDataDirectory))
	{
		if(!boost::filesystem::is_directory(pathOfNewDataDirectory))
		{
			std::cout<<"Could not create dir: "<< pathOfNewDataDirectory << std::endl;
			return -4;
		}
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
		double time = sw.getTime();

		std::cout << "Size before: " << inputCloud->size() << " Size after: "<<outputCloud->size() << std::endl;

		//Increase total computation time
		totalTime+=time;

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
		outputModel.setResult(cloud_ids[i], "computation_time", time);
		outputModel.setResult(cloud_ids[i], "cummulative_filter_time", totalTime);

	}
	

	outputModel.saveFile(param_outputModel);

}
