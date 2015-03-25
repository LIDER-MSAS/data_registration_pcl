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


#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "dataFramework/data_model.hpp"

typedef pcl::PointXYZ PointT;
pcl::visualization::PCLVisualizer p;
std::vector<pcl::PointCloud<PointT>::Ptr> pcs;
std::vector<std::string> pcsName;

int currentlyHighlitedScan =0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if ((event.getKeySym()=="l") && event.keyUp())
	{
		std::cout << "will save metascan\n";
		pcl::PointCloud<PointT>::Ptr metascan (new pcl::PointCloud<PointT>);
		for (int i=0; i < pcs.size(); i++)
		{
			*metascan+=*pcs[i];
		}
		pcl::io::savePCDFile("metascan.pcd",*metascan);

	}

	if (event.getKeySym()=="2" && event.keyUp())
	{
		std::cout <<" will show all scans in one colors\n";
		p.removeAllPointClouds();
		for (int i=0; i < pcs.size(); i++)
		{	
			p.addPointCloud(pcs[i], pcsName[i]);
		}		
	}
	if (event.getKeySym()=="3" && event.keyUp())
	{
		std::cout <<" will show all scans in multi colors\n";
		p.removeAllPointClouds();
		for (int i=0; i < pcs.size(); i++)
		{	
			char colR = rand()%255;
			char colG = rand()%255;
			char colB = rand()%255;
			pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (pcs[i], colR, colG, colB);
			p.addPointCloud(pcs[i], source_cloud_color_handler, pcsName[i]);
		}		
	}
	if ((event.getKeySym()=="a"  || event.getKeySym()=="z" ) && event.keyUp())
	{
		if (event.getKeySym()=="a")
		{
			currentlyHighlitedScan++;
			if (currentlyHighlitedScan >= pcs.size()) currentlyHighlitedScan =pcs.size()-1;
		}
		if (event.getKeySym()=="z")
		{
			currentlyHighlitedScan--;
			if (currentlyHighlitedScan <=0) currentlyHighlitedScan =0;
		}

		std::cout <<" will show highlight scan :"<<pcsName[currentlyHighlitedScan] << " in colors\n";
		p.removeAllPointClouds();
		for (int i=0; i < pcs.size(); i++)
		{	
			if ( i== currentlyHighlitedScan)
			{
				pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (pcs[i], 255,0,0);
				p.addPointCloud(pcs[i], source_cloud_color_handler, pcsName[i]);
				p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, pcsName[i]);
			}
			else
			{
				p.addPointCloud(pcs[i], pcsName[i]);
			}
		}

		p.updateCamera ();
	}



}


int main (int argc, char** argv)
{



	/// loadpointclouds

	if (argc !=2 && argc !=3)
	{
		std::cout << "USAGE:\n";
		std::cout << argv[0]<<" xml_model_file \n";
		std::cout <<"or\n";
		std::cout << argv[0]<<" xml_model_file folder_with_pcd\n";
		return -1;
	}
	std::string model_file = argv[1];
	

	p.setWindowName(model_file);
	data_model dSets;
	dSets.loadFile(model_file);
	std::vector<std::string> indices;
	dSets.getAllScansId(indices);

	for (int i=0; i< indices.size(); i++)
	{
		std::string fn;
		dSets.getPointcloudName(indices[i], fn);
		std::cout << indices[i]<<"\t"<<fn<<"\n";
	}
	p.addCoordinateSystem(5);
	for (int i=0; i < indices.size(); i++)
	{
		pcl::PointCloud<PointT>::Ptr pc (new pcl::PointCloud<PointT>);
		std::string fn;
		Eigen::Matrix4f transform;
		fn = dSets.getFullPathOfPointcloud(indices[i]);
		bool isOkTr = dSets.getAffine(indices[i], transform);
		if (isOkTr)
		{
			std::cout <<"============\n";
			std::cout <<"adding pc     :"<< indices[i]<<"\n";
			std::cout <<"filen name    :"<<fn <<"\n";
			std::cout <<"with transform: \n"<< transform<<"\n";
			pcl::io::loadPCDFile<PointT>(fn, *pc);
			pc->sensor_origin_ = Eigen::Vector4f(0,0,0,0);
			pc->sensor_orientation_ = Eigen::Quaternionf::Identity();

			pcl::transformPointCloud(*pc,*pc, transform);
			p.addPointCloud(pc, indices[i]);
			pcs.push_back(pc);
			pcsName.push_back(indices[i]);
		}		
	}


	p.registerKeyboardCallback (keyboardEventOccurred, (void*)&p);
	p.spin();

	return 0;
}

