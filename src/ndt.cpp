/*
 * Software License Agreement (BSD License)
 *
 *  Data Registration Framework - Mobile Spatial Assistance System
 *  Copyright (c) 2014-2015, Institute of Mathematical Machine
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
 *   * Neither the name of Institute of Mathematical Machine nor the names of its
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

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <vector>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>

#include "dataFramework/data_model.hpp"

#include "dataFramework/viewerLog.hpp"
#include <pcl/common/time.h>


//convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;
data_model inputXML;
data_model outputXML;
std::vector<std::string> indices;

pcl::PointCloud<PointT> metascan;
std::vector<std::string> pcNames;

Eigen::Affine3f lastGlobalOdom;
Eigen::Affine3f lastFit;
Eigen::Affine3f currentFitCandidate;
Eigen::Affine3f currentlyAssignedGlobalOdom;
Eigen::Affine3f currentInitialFit;

std::string currentFileName;
std::string outputXMLFn;

int currentPointcloud =0;
pcl::PointCloud<PointT>::Ptr currentlyRegisteredPc;
bool registration_accepted = false;
std::vector<std::string> msg;
bool isUseMetascan = false;

int ndt_iter = 100;
float ndt_res =1.0f;
float ndt_step_size = 0.1f;
float ndt_trans_eps = 0.01f;
float cumulative_align_time = 0.0f;

bool registerNDT(pcl::PointCloud<PointT> &metascan, pcl::PointCloud<PointT> &scan, Eigen::Affine3f &metascanToScan, std::string cloudId)
{
	pcl::NormalDistributionsTransform<PointT, PointT> * ndt = new pcl::NormalDistributionsTransform<PointT, PointT>();

	ndt->setMaximumIterations (ndt_iter);
	ndt->setResolution (ndt_res);
	ndt->setStepSize (ndt_step_size);
	ndt->setTransformationEpsilon (ndt_trans_eps);
	ndt->setInputTarget (metascan.makeShared());
	ndt->setInputSource (scan.makeShared());
	pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
	
	pcl::StopWatch sw;
	sw.reset();
	ndt->align (*tmp);
	double time = sw.getTime();

	std::cout << "After NDT:\n";
	std::cout << ndt->getFinalTransformation () << std::endl;
	metascanToScan = ndt->getFinalTransformation();
	outputXML.setResult(cloudId, "FitnessScore", ndt->getFitnessScore());
	cumulative_align_time +=time;
	outputXML.setResult(cloudId, "AlignTime", time);
	outputXML.setResult(cloudId, "CumulativeAlignTime", cumulative_align_time);
	
	return true;
}
void loadNextPc()
{
	currentPointcloud++;
	if (currentPointcloud < indices.size())
	{
		std::cout << "loading pointcloud "<<indices[currentPointcloud]<<"\n";

		//inputXML.getPointcloudName(indices[currentPointcloud],currentFileName);	

		currentFileName= inputXML.getFullPathOfPointcloud(indices[currentPointcloud]);
		currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

		pcl::io::loadPCDFile(currentFileName, *currentlyRegisteredPc);

		currentlyAssignedGlobalOdom.setIdentity();
		inputXML.getAffine(indices[currentPointcloud],currentlyAssignedGlobalOdom.matrix()),

		currentlyRegisteredPc->sensor_origin_ = Eigen::Vector4f(0,0,0,0);
		currentlyRegisteredPc->sensor_orientation_ = Eigen::Quaternionf::Identity();

		Eigen::Affine3f odometryIncrement = lastGlobalOdom.inverse()*currentlyAssignedGlobalOdom;
		std::cout << "odometry increment matrix :\n";
		std::cout<< odometryIncrement.matrix()<<"\n";
		currentInitialFit = lastFit * odometryIncrement ; 
		pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc,currentInitialFit );
	}
	else
	{
		std::cout <<"There is no more pointclouds in given model \n";

	}
}
void registerScan()
{
	Eigen::Affine3f tr;
	registration_accepted = false;

	bool res = registerNDT(metascan, *currentlyRegisteredPc, tr, indices[currentPointcloud]);

	if (res)
	{
		pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc, tr);
		currentFitCandidate =   tr * currentInitialFit;
	}
	else
	{
		std::cout <<"registration failed\n";
	}
}
void accept()
{
	registration_accepted = true;
	if (isUseMetascan)
	{
		/// if metascan approach is not used metascan become last registered pc
		metascan += *currentlyRegisteredPc;		
	}
	else
	{
		metascan = *currentlyRegisteredPc;
	}
	lastGlobalOdom = currentlyAssignedGlobalOdom;
	lastFit = currentFitCandidate;

	outputXML.setAffine(indices[currentPointcloud], lastFit.matrix());
	std::string cloud_fn;
	inputXML.getPointcloudName(indices[currentPointcloud] ,cloud_fn);
	outputXML.setPointcloudName(indices[currentPointcloud], cloud_fn);
	std::cout <<"saving model to " << outputXMLFn<<"\n";
	outputXML.saveFile(outputXMLFn);
}

int main (int argc, char** argv)
{
	if(argc<3)
	{
		std::cout <<"USAGE:\n";
		std::cout <<argv[0]<<" parameters inputModel.xml outputModel.xml\n";
		std::cout <<" -r\tSets the voxel grid resolution.\tDefault: " << ndt_res << std::endl;
		std::cout <<" -i\tSets the maximum number of iterations the internal optimization should run for.\tDefault: " << ndt_iter << std::endl;
		std::cout <<" -eps\tSets the transformation epsilon.\tDefault: " << ndt_trans_eps << std::endl;
		std::cout <<" -s\tSets the newton line search maximum step length.\tDefault: " << ndt_step_size << std::endl;
		std::cout <<" -m\tSets the usage of metascan.\tDefault:" << isUseMetascan << std::endl;
	
		return -1;
	}

	pcl::console::parse_argument (argc, argv, "-r", ndt_res);
	pcl::console::parse_argument (argc, argv, "-i", ndt_iter);
	pcl::console::parse_argument (argc, argv, "-eps", ndt_trans_eps);
	pcl::console::parse_argument (argc, argv, "-s", ndt_step_size);
	pcl::console::parse_argument (argc, argv, "-m", isUseMetascan);
	
	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");


	std::string inputXMLFn = argv[xml_indices[0]];
	outputXMLFn = argv[xml_indices[1]];

	std::cout << "Input model  :"<< inputXMLFn <<"\n";
	std::cout << "Output model :"<< outputXMLFn <<"\n";


	currentlyRegisteredPc = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);


	inputXML.loadFile(inputXMLFn);
	inputXML.getAllScansId(indices);

	//generate some data to report
	outputXML.setAlgorithmName("NDT");
	outputXML.addAlgorithmParam("ndt_iter",ndt_iter);
	outputXML.addAlgorithmParam("ndt_res",ndt_res);
	outputXML.addAlgorithmParam("ndt_step_size",ndt_step_size);
	outputXML.addAlgorithmParam("ndt_step_size",ndt_step_size);
	outputXML.addAlgorithmParam("isUseMetascan",isUseMetascan);


	std::string dataPath;
	inputXML.getDataSetPath(dataPath);
	outputXML.setDataSetPath(dataPath);


	pcl::PointCloud<PointT> tmp;
	
	lastGlobalOdom.setIdentity();
	lastFit.setIdentity();
	std::string fn;
	fn = inputXML.getFullPathOfPointcloud(indices[0]);
	std::string pcdFileNameOnly;
	inputXML.getPointcloudName(indices[0], pcdFileNameOnly); 
	pcl::io::loadPCDFile(fn,tmp);
	inputXML.getAffine(indices[0],lastGlobalOdom.matrix()),
	pcl::transformPointCloud(tmp,tmp, lastGlobalOdom);
	lastFit = lastGlobalOdom;
	metascan += tmp;
	metascan.sensor_origin_ = Eigen::Vector4f(0,0,0,0);
	metascan.sensor_orientation_ = Eigen::Quaternionf::Identity();

	outputXML.setPointcloudName(indices[0],pcdFileNameOnly);
	outputXML.setResult(indices[0], "FitnessScore", 0.0f);
	outputXML.setResult(indices[0], "AlignTime", 0.0f);
	outputXML.setResult(indices[0], "CummulativeAlignTime", 0.0f);
	outputXML.setAffine(indices[0], lastGlobalOdom.matrix());


	for (int i=0; i<indices.size()-1; i++)
	{
		loadNextPc();
		registerScan();
		accept();	
	}
}
/* ]--- */