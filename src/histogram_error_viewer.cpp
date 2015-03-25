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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include<pcl/visualization/pcl_plotter.h>
typedef pcl::PointXYZRGBNormal PointT;
pcl::visualization::PCLVisualizer p;



int currentlyHighlitedScan =0;

void createMetascan (data_model &model, pcl::PointCloud<PointT> &pc)
{
	std::cout <<"creating metascan\n";
	std::vector<std::string> ids;
	model.getAllScansId(ids);

	for (int i=0; i < ids.size();i ++)
	{
		std::string fn = model.getFullPathOfPointcloud(ids[i]);
		std::cout <<"loading "<<fn <<"\n";
		Eigen::Matrix4f tr;
		model.getAffine(ids[i], tr);
		pcl::PointCloud<PointT> tmp;
		pcl::io::loadPCDFile(fn, tmp);
		pcl::transformPointCloud(tmp,tmp,tr);
		pc +=tmp;
	}
}



int main (int argc, char** argv)
{
	
	bool skip_registration = false;
	
	pcl::PointCloud<PointT> submetascanTarget;
	pcl::PointCloud<PointT> alignedSourceMetascan;

	if (!skip_registration)
	{
		std::string modelSourceFn = argv[1];
		std::string modelTargetFn = argv[2];

		std::cout <<"input model " << modelSourceFn <<"\n";
		std::cout <<"output model " << modelTargetFn <<"\n";


		data_model modelSource;
		data_model modelTarget;

		modelSource.loadFile(modelSourceFn);
		modelTarget.loadFile(modelTargetFn);

		pcl::PointCloud<PointT> metascanSource;
		pcl::PointCloud<PointT> metascanTarget;

		createMetascan(modelSource, metascanSource);
		createMetascan(modelTarget, metascanTarget);
		// saving metascan - in lcoal cooridanate system - for manula registration in CloudCompare
		pcl::io::savePCDFile ("metaSource.pcd", metascanSource);
		pcl::io::savePCDFile ("metaTarget.pcd", metascanTarget);



		Eigen::Affine3f SourceGlobalMatrix, TargetGlobalMatrix;
		SourceGlobalMatrix = Eigen::Affine3f::Identity();
		TargetGlobalMatrix = Eigen::Affine3f::Identity();
		modelSource.getGlobalModelMatrix(SourceGlobalMatrix.matrix());
		modelTarget.getGlobalModelMatrix(TargetGlobalMatrix.matrix());	
		pcl::transformPointCloudWithNormals(metascanSource,metascanSource, SourceGlobalMatrix);
		pcl::transformPointCloudWithNormals(metascanTarget,metascanTarget, TargetGlobalMatrix);

		// subsample data for final registration

		std::cout << "filtering\n";
		pcl::PointCloud<PointT> submetascanSource;
	
		pcl::VoxelGrid<PointT> sor;
		
		sor.setInputCloud(metascanSource.makeShared());
		sor.setLeafSize(0.125,0.125,0.125);
		sor.filter(submetascanSource);


		sor.setInputCloud(metascanTarget.makeShared());
		sor.setLeafSize(0.125,0.125,0.125);
		sor.filter(submetascanTarget);
	
		// icp 
		std::cout << "icp\n";
		
		pcl::IterativeClosestPoint <PointT,PointT> icp;
		icp.setInputSource(submetascanSource.makeShared());
		icp.setInputTarget(submetascanTarget.makeShared());
		icp.setMaxCorrespondenceDistance(0.3);
		icp.setMaximumIterations(200);
		icp.align(alignedSourceMetascan);
		std::cout <<"icp fitness score " << icp.getFitnessScore() <<"\n";

	
		pcl::io::savePCDFile("submetascanTarget.pcd",submetascanTarget);
		pcl::io::savePCDFile("alignedSourceMetascan.pcd",alignedSourceMetascan);
	}
	else
	{
		std::cout << "registration skipped - remove submetascanTarget.pcd and alignedSourceMetascan.pcd to prevent skipping \n";
		pcl::io::loadPCDFile("submetascanTarget.pcd",submetascanTarget);
		pcl::io::loadPCDFile("alignedSourceMetascan.pcd",alignedSourceMetascan);
	}


	// kdtrees
	
	pcl::search::KdTree<PointT>::Ptr treeAlignedSourceMetascan (new pcl::search::KdTree<PointT> ());
	
	std::vector <double> distances;
	std::vector <double> normalAngles;

	double radius = 0.5;

	treeAlignedSourceMetascan->setInputCloud(alignedSourceMetascan.makeShared());
	for (int i=0; i < submetascanTarget.size(); i++)
	{
		PointT p = submetascanTarget[i];
		Eigen::Vector4f pN = submetascanTarget[i].getNormalVector4fMap();
		std::vector<int> indices;
		std::vector<float> sqrdistance;
		treeAlignedSourceMetascan->nearestKSearch(p,1, indices, sqrdistance);
		if (sqrdistance.size() >0)
		{
			int i = indices[0];
			Eigen::Vector4f  qN = alignedSourceMetascan[i].getNormalVector4fMap();
			if (sqrdistance[0]<(0.5*0.5))
			{
				
				distances.push_back(sqrt(sqrdistance[0]));

				
			}

			if (sqrdistance[0]<(0.7*0.7))
			{
				float c = qN.dot(pN);
				float ang = 180.0*acos(c)/M_PI;
				if (ang == ang ) normalAngles.push_back(ang);
				

			}
		}
	}


	std::cout << "distances size " << distances.size() <<"\n";
	std::cout << "normalAngles size " << normalAngles.size() <<"\n";

	//vis
	pcl::visualization::PCLPlotter  plotter1;
	plotter1.addHistogramData(distances,  500);
	
	pcl::visualization::PCLPlotter  plotter2;
	plotter2.addHistogramData(normalAngles,  500);
	
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanSource_color_handler (submetascanTarget.makeShared(), 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanTarget_color_handler (alignedSourceMetascan.makeShared(), 0, 255, 0);
	p.addPointCloud<PointT>(submetascanTarget.makeShared(),metascanSource_color_handler,"sorce");
	p.addPointCloud<PointT>(alignedSourceMetascan.makeShared(),metascanTarget_color_handler,"target");
	
	plotter1.setTitle("NN-distance Histogram");
	plotter1.setXTitle("Distance [m]");
	plotter1.setYTitle("Point count");

	plotter2.setTitle("Normal Histogram");
	plotter2.setXTitle("Angle");
	plotter2.setYTitle("Point count");




	while (1)
	{

		p.spinOnce();
		plotter1.spinOnce();
		plotter2.spinOnce();	
	}
	return 0;
}

