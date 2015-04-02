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
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <dataFramework/data_model.hpp>
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
	if(argc<3)
	{
		std::cout << "Usage:\n";
		std::cout << argv[0] << " input_model.xml output_model.xml parameters\n";
		std::cout << " -l <leaf_size>\t\tSet the voxel grid leaf size. Default: " << leaf_size << std::endl;
	
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

		// Create the filtering object
		VoxelGrid<pcl::PCLPointCloud2> grid;
		grid.setInputCloud (inputCloud);
		grid.setLeafSize (leaf_size, leaf_size, leaf_size);

		//Measure the computation time
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
