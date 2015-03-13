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

template <typename PointSource, typename PointTarget, typename Scalar = float> class myIcp : public  pcl::IterativeClosestPoint <PointSource, PointTarget, Scalar >
{
public:
	pcl::CorrespondencesPtr getCorrespondeces()
	{
		return correspondences_;
	}
};

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
float  cummulativeTime =0;
double icp_CorrespondenceDistance = 0.15;
double icp_RANSACOutlierRejectionThreshold = 0.15;
int icp_MaximumIterations = 100;


bool registerICP(pcl::PointCloud<PointT> &metascan, pcl::PointCloud<PointT> &scan, Eigen::Affine3f &metascanToScan, std::string cloudId)
{
	std::cout <<"invoking ICP on scan "<< cloudId<< " \n";

	myIcp<PointT, PointT> *icp;
	icp = new myIcp<PointT, PointT>();	//TODO: change to smart pointer

	icp->setMaximumIterations (icp_MaximumIterations);
	icp->setMaxCorrespondenceDistance (icp_CorrespondenceDistance);
	icp->setRANSACOutlierRejectionThreshold (icp_RANSACOutlierRejectionThreshold);

	icp->setInputTarget (metascan.makeShared());
	icp->setInputSource (scan.makeShared());

	pcl::PointCloud<PointT>::Ptr tmp (new pcl::PointCloud<PointT>);
	pcl::StopWatch sw;
	sw.reset();
	icp->align (*tmp);
	double time = sw.getTime();
	std::cout << icp->getFinalTransformation () << std::endl;
	metascanToScan = icp->getFinalTransformation();

	if (scan.size() != 0)
	{
		float overlap = 100.0f *  icp->getCorrespondeces()->size()/scan.size();
		outputXML.setResult(cloudId, "overlap", overlap);

	}
	outputXML.setResult(cloudId, "FitnessScore", icp->getFitnessScore());
	printf("FitnessScore: %f\n", icp->getFitnessScore());
	outputXML.setResult(cloudId, "AlignTime", time);
	cummulativeTime += time;
	outputXML.setResult(cloudId, "CummulativeAlignTime", cummulativeTime);

	delete icp;	//TODO: remove this

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


typedef pcl::PointXYZ PointType;

void registerScan()
{
	Eigen::Affine3f tr;
	registration_accepted = false;

	bool res;

	printf("trying register...\n");


	res= registerICP(metascan, *currentlyRegisteredPc, tr, indices[currentPointcloud]);


	printf("metascan.size(): %d   currentlyRegisteredPc->size():%d \n", metascan.size(), currentlyRegisteredPc->size() );

	if (res)
	{
		printf("registration ok\n");
		pcl::transformPointCloud(*currentlyRegisteredPc,*currentlyRegisteredPc, tr);
		currentFitCandidate =   tr * currentInitialFit;

	}
	else
	{
		printf("registration NOK %s \n", (indices[currentPointcloud]).c_str() ); //tu by sie przydala nazwa skanu

	
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
		std::cout << "Usage:\n";
		std::cout << argv[0] <<" inputModel.xml outputModel.xml parameters\n";
		std::cout << " -d\tSets the maximum distance threshold between two correspondent points in source <-> target.\
If the distance is larger than this threshold, the points will be ignored in the alignment process.\tDefault: " << icp_CorrespondenceDistance << std::endl;
		std::cout << " -r\tSets the inlier distance threshold for the internal RANSAC outlier rejection loop.\
The method considers a point to be an inlier, if the distance between the target data index and the \
transformed source index is smaller than the given inlier distance threshold.\tDefault: " << icp_RANSACOutlierRejectionThreshold << std::endl;
		std::cout << " -i\tSets the maximum number of iterations the internal optimization should run for.\tDefault: " << icp_MaximumIterations << std::endl;
		std::cout << " -m\tSets the usage of metascan.\tDefault: " << isUseMetascan << std::endl;

		pcl::console::parse_argument (argc, argv, "-d", icp_CorrespondenceDistance);
		pcl::console::parse_argument (argc, argv, "-r", icp_RANSACOutlierRejectionThreshold);
		pcl::console::parse_argument (argc, argv, "-i", icp_MaximumIterations);
		pcl::console::parse_argument (argc, argv, "-m", isUseMetascan);
	
		return -1;
	}


	std::vector<int> xml_indices;
	xml_indices = pcl::console::parse_file_extension_argument (argc, argv, ".xml");
	
	if(xml_indices.size()!=2)
	{
		return -2;
	}

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


	std::string dataPath;
	inputXML.getDataSetPath(dataPath);
	outputXML.setDataSetPath(dataPath);


	//add first scan to result;



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
	outputXML.setResult(indices[0], "overlap", 0.0f);
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
