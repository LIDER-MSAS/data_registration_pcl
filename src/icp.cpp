

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/parse.h>

#include <pcl/registration/icp.h>

#include "dataFramework\data_model.hpp"

#include "dataFramework\viewerLog.hpp"
#include <pcl/common/time.h>


using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

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
pcl::visualization::PCLVisualizer p;
logViewer* logger;
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

double icp_CorrespondenceDistance = 0.15;
double icp_RANSACOutlierRejectionThreshold = 0.15;
int icp_MaximumIterations = 100;

bool auto_reg = true;
bool registerICP(pcl::PointCloud<PointT> &metascan, pcl::PointCloud<PointT> &scan, Eigen::Affine3f &metascanToScan, std::string cloudId)
{
	std::cout <<"invoking ICP on scan "<< cloudId<< " \n";

	pcl::IterativeClosestPoint<PointT, PointT> *icp;
	icp = new pcl::IterativeClosestPoint<PointT, PointT>();

	icp->setMaximumIterations (icp_MaximumIterations);
	icp->setMaxCorrespondenceDistance (icp_CorrespondenceDistance);
	icp->setRANSACOutlierRejectionThreshold (icp_RANSACOutlierRejectionThreshold);

	icp->setInputTarget (metascan.makeShared());
	icp->setInputSource (scan.makeShared());

	pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
	pcl::StopWatch sw;
	sw.reset();
	icp->align (*tmp);
	float executionTime = sw.getTime();
	std::cout << icp->getFinalTransformation () << std::endl;
	metascanToScan = icp->getFinalTransformation();
	outputXML.setResult(cloudId, "FitnessScore", icp->getFitnessScore());
	outputXML.setResult(cloudId, "AlignTime", executionTime);
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
		logger->addMessage("loaded pointcloud , press r to register");
	}
	else
	{
		std::cout <<"There is no mode pointclouds in given model \n";

	}
}
void registerScan()
{
	Eigen::Affine3f tr;
	registration_accepted = false;
	//bool res = registerNDT(metascan, *currentlyRegisteredPc, tr);
	bool res;

	res= registerICP(metascan, *currentlyRegisteredPc, tr, indices[currentPointcloud]);

	if (res)
	{
		pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc, tr);
		currentFitCandidate =   tr * currentInitialFit;
		logger->addMessage("registration OK, press a to accept");
	}
	else
	{
		std::cout <<"registration failed\n";
		logger->addMessage("registration NOT ok, l to load next scan, or r for next iterations of registration");
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
	logger->addMessage("registration accepted");
	outputXML.setAffine(indices[currentPointcloud], lastFit.matrix());
	std::string cloud_fn;
	inputXML.getPointcloudName(indices[currentPointcloud] ,cloud_fn);
	outputXML.setPointcloudName(indices[currentPointcloud], cloud_fn);
	std::cout <<"saving model to " << outputXMLFn<<"\n";
	outputXML.saveFile(outputXMLFn);
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	if (event.getKeySym()=="l" && event.keyUp())
	{
		loadNextPc();
	}
	if (event.getKeySym()=="r" && event.keyUp())
	{
		registerScan();
	}
	if (event.getKeySym()=="a" && event.keyUp())
	{
		accept();		
	}
	if (event.getKeySym()=="q"&& event.keyUp())
	{
		loadNextPc();
		registerScan();
		accept();		
	}

	p.removeAllPointClouds();
	pcl::visualization::PointCloudColorHandlerCustom<PointT> metascanH(metascan.makeShared(), 0, 0, 255);
	p.addPointCloud<PointT> (metascan.makeShared(),metascanH,"metascan" );

	pcl::visualization::PointCloudColorHandlerCustom<PointT> scanH(currentlyRegisteredPc, 255, 0, 0);
	p.addPointCloud<PointT> (currentlyRegisteredPc,scanH, "scan");
}

int main (int argc, char** argv)
{
	logger =new logViewer(&p,25);

	std::cout <<"USAGE:\n";
	std::cout <<argv[0]<<" parameters inputModel.xml outputModel.xml\n";
	std::cout <<" -d sets icp_CorrespondenceDistance\n";
	std::cout <<" -r sets icp_RANSACOutlierRejectionThreshold\n";
	std::cout <<" -i sets icp_MaximumIterations\n";
	std::cout <<" -m sets usage of metascan\n";
	std::cout <<" -u shows 3d viewer for semi-automatic registration\n";

	pcl::console::parse_argument (argc, argv, "-d", icp_CorrespondenceDistance);

	pcl::console::parse_argument (argc, argv, "-r", icp_RANSACOutlierRejectionThreshold);

	pcl::console::parse_argument (argc, argv, "-i", icp_MaximumIterations);
	pcl::console::parse_argument (argc, argv, "-m", isUseMetascan);
	pcl::console::parse_argument (argc, argv, "-u", auto_reg);
	


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
	outputXML.setAlgorithmName("ICP");
	outputXML.addAlgorithmParam("icp_CorrespondenceDistance",icp_CorrespondenceDistance);
	outputXML.addAlgorithmParam("icp_RANSACOutlierRejectionThreshold",icp_RANSACOutlierRejectionThreshold);
	outputXML.addAlgorithmParam("icp_MaximumIterations",icp_MaximumIterations);
	outputXML.addAlgorithmParam("isUseMetascan",isUseMetascan);
	outputXML.addAlgorithmParam("auto_reg",auto_reg);


	std::string dataPath;
	inputXML.getDataSetPath(dataPath);
	outputXML.setDataSetPath(dataPath);


	pcl::PointCloud<PointT> tmp;
	
	lastGlobalOdom.setIdentity();
	lastFit.setIdentity();
	std::string fn;
	fn = inputXML.getFullPathOfPointcloud(indices[0]);
	pcl::io::loadPCDFile(fn,tmp);
	inputXML.getAffine(indices[0],lastGlobalOdom.matrix()),
	pcl::transformPointCloud(tmp,tmp, lastGlobalOdom);
	lastFit = lastGlobalOdom;
	metascan += tmp;
	metascan.sensor_origin_ = Eigen::Vector4f(0,0,0,0);
	metascan.sensor_orientation_ = Eigen::Quaternionf::Identity();


	p.registerKeyboardCallback (keyboardEventOccurred, (void*)&p);
	if (auto_reg)
	{
		//p.close();
		for (int i=0; i<indices.size()-1; i++)
		{
			loadNextPc();
			registerScan();
			accept();	
		}
	}

	if (!auto_reg) p.spin();
}
/* ]--- */