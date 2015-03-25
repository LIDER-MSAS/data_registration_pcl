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

#include <pcl/console/parse.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/correspondence_estimation.h>
#include "dataFramework/data_model.hpp"
#include <pcl/registration/icp.h>


typedef pcl::PointXYZ PointT;
pcl::visualization::PCLVisualizer p;
std::vector<pcl::PointCloud<PointT>::Ptr> pcs;
std::vector<std::string> pcsName;
data_model dSets;
int currentlyHighlitedScan =0;

template <typename PointSource, typename PointTarget, typename Scalar = float> class myIcp : public  pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >
{
public:
	pcl::CorrespondencesPtr getCorrespondeces()
	{
        return pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >::correspondences_;
	}
};
 
 
void loadPointcloud (pcl::PointCloud<PointT> &scan, std::string id)
{
	std::string fn = dSets.getFullPathOfPointcloud(id);
	std::cout <<"file name "<< fn << "\n";
	pcl::io::loadPCDFile(fn, scan);
	Eigen::Matrix4f mat;
	dSets.getAffine(id, mat);
	std::cout <<"marix transform: \n "<< mat << "\n";
	pcl::transformPointCloud(scan,scan,mat);
}



int main (int argc, char** argv)
{
	if(argc < 3)
	{
		printf("usage: icp_overlap_viewer.exe input.xml scan_id1 scan_id2 -d 0.5 -r 0.1\n");
		printf("-d maxCorrespondenceDistance\n");
		printf("-r RANSACOutlierRejectionThreshold\n");
	}

	double maxCorrespondenceDistance = 0.5;
	pcl::console::parse_argument (argc, argv, "-d", maxCorrespondenceDistance);

	double RANSACOutlierRejectionThreshold = 0.1;
	pcl::console::parse_argument (argc, argv, "-r", RANSACOutlierRejectionThreshold);
	/// loadpointclouds
 
	std::string model_file = argv[1];
	std::string scan_id1 = argv[3];
	std::string scan_id2 = argv[2];
	std::cout << model_file <<"\n";
	std::cout << scan_id1 <<"\n";
	std::cout << scan_id2 <<"\n";
 
	dSets.loadFile(model_file);
	p.setWindowName(model_file);
	
	pcl::PointCloud<PointT> scan1;
	pcl::PointCloud<PointT> scan2;
	pcl::PointCloud<PointT> scanAlign;
 
	loadPointcloud (scan1, scan_id1);
	loadPointcloud (scan2, scan_id2);
 
	pcl::visualization::PointCloudColorHandlerCustom<PointT> handler_scan1 (scan1.makeShared(), 0,0,255);
	pcl::visualization::PointCloudColorHandlerCustom<PointT> handler_scan2 (scan2.makeShared(), 255,0,0);
 
	
	myIcp<PointT, PointT> *icp;
	icp = new myIcp<PointT, PointT>();
 
	Eigen::Vector4f centroid1;
	Eigen::Vector4f centroid2;

	pcl::PointXYZ pcentroid1;
	pcl::PointXYZ pcentroid2;

	
	pcl::compute3DCentroid(scan1, centroid1);
	pcl::compute3DCentroid(scan2, centroid2);

	
	pcentroid1.x = centroid1.x();
	pcentroid1.y = centroid1.y();
	pcentroid1.z = centroid1.z();

	pcentroid2.x = centroid2.x();
	pcentroid2.y = centroid2.y();
	pcentroid2.z = centroid2.z();

	icp->setMaximumIterations (0);
	icp->setMaxCorrespondenceDistance (maxCorrespondenceDistance);
	icp->setRANSACOutlierRejectionThreshold (RANSACOutlierRejectionThreshold);
 
	icp->setInputTarget (scan1.makeShared());
	icp->setInputSource (scan2.makeShared());
 
	icp->align(scanAlign);
 
	pcl::Correspondences corr = *(icp->getCorrespondeces());
	p.addPointCloud(scan1.makeShared(), handler_scan1 , "scan1");
	p.addPointCloud(scan2.makeShared(), handler_scan2 , "scan2");
	p.addSphere<pcl::PointXYZ>(pcentroid1,0.5,"centroid1",0);
	p.addSphere<pcl::PointXYZ>(pcentroid2,0.5,"centroid2",0);

	for (int i=0; i< corr.size(); i++)
	{
		int i1= corr[i].index_match;
		int i2= corr[i].index_query;
		std::stringstream ss;
		ss << i;
		if (i1 >0 && i2)
		{
			p.addLine<PointT> (scan1.points[i1], scan2.points[i2], ss.str());
		}
	}
	std::cout << "correspondeces size " << corr.size()<< "\n";
	std::cout << "size scan1 " << scan1.size()<< "\n";
	std::cout << "size scan2 " << scan2.size()<< "\n";
	float overlap = 100.0f*corr.size()/scan2.size();
	std::stringstream ss;
	ss << std::setprecision(2) << overlap;
	std::cout << "overlap : " << ss.str() << " %\n";
	p.setBackgroundColor(1,1,1);
	p.spin();
 
	return 0;
}
