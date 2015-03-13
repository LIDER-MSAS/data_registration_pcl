#include<vector>
#include<iostream>
#include<utility>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl/visualization/pcl_visualizer.h>
#include<pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/common/transforms.h>
//...

using namespace std;

typedef pcl::PointXYZ PointT;
pcl::visualization::PCLVisualizer p;

int main ()
{

	pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr planeInliners (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr planeInlinersDistance (new pcl::PointCloud<pcl::PointXYZI>);


	pcl::io::loadPCDFile("plane.pcd", *plane);
	std::vector<int> inliers;

	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (plane));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.1);
    ransac.computeModel();
    ransac.getInliers(inliers);
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);

	//pcl::copyPointCloud<pcl::PointXYZ>(*plane, inliers, *planeInliners);

	std::vector<double> distances;
	model_p->getDistancesToModel(coeff,distances);
	

	
	//pcl::visualization::PointCloudColorHandlerCustom<PointT> handler_plane (plane, 0, 255, 0);
	Eigen::Vector3f n (coeff[0],coeff[1],coeff[2]);
	Eigen::Quaternionf q ;
	q.setFromTwoVectors(n,Eigen::Vector3f(0,0,1));
	Eigen::Quaternionf q2 ;
	q2.setFromTwoVectors(n,Eigen::Vector3f(0,1,0));


	Eigen::Affine3f rot = Eigen::Affine3f::Identity();
	rot.rotate(q2*q);
	pcl::transformPointCloud(*plane,*plane,rot);

	for (int i =0; i < distances.size(); i++)
	{
		PointT p  = (*plane)[i];
	
		pcl::PointXYZI pi;
		pi.x = p.x;
		pi.y = p.y;
		pi.z = p.z;
		pi.intensity = p.x;
		planeInlinersDistance->push_back(pi);

	}
	//p.addPointCloud(plane,handler_plane,"plane");
	
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(planeInlinersDistance,"intensity");
	//p.addCoordinateSystem(10);
	p.addPointCloud(planeInlinersDistance, intensity_distribution);
	//pcl::io::savePCDFile("pc.pcd", *intensity_distribution);

	p.setBackgroundColor(0xff,0xff,0xff);
	p.spin();
  return 0;
}