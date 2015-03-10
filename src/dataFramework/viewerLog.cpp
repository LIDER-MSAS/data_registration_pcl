
#include "viewerlog.hpp"


logViewer::logViewer(pcl::visualization::PCLVisualizer* _viewer, int _msgCount)
{
	viewer = _viewer;
	msgCount = _msgCount;
	for (int i =0; i < _msgCount; i++)
	{
		std::stringstream ss;
		ss<<"text"<<i;
		viewer->addText("...",10, i*15, 10,0,1,0, ss.str());
	}
	msgs.resize(_msgCount);
}
void logViewer::addMessage (std::string text)
{
	msgs.insert(msgs.begin(),text);
	for (int i=0; i< 25; i++)
	{
		std::stringstream ss;
		ss<<"text"<<i;
		if (msgs[i].size()==0)msgs[i]=".."; 
		viewer->updateText(msgs[i],10, 20+i*15, 10,0,1,0, ss.str());
		viewer->spinOnce();
	}
}