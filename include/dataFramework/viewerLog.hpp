#ifndef __VIEWER_LOG_HPP
#define __VIEWER_LOG_HPP
#include <pcl/visualization/pcl_visualizer.h>
#include <vector>

class logViewer
{
public:
	logViewer(pcl::visualization::PCLVisualizer* _viewer, int _msgCount);
	void addMessage (std::string text);
private:
	int msgCount;
	pcl::visualization::PCLVisualizer* viewer;
	std::vector<std::string> msgs;
};

#endif