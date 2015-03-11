#include <iostream>
#include "dataFramework\data_model.hpp"
#include <boost\lexical_cast.hpp>

int main (int argc, char** argv)
{
	if (argc != 3)
	{
		std::cerr<< "USAGE :" << argv[0]<<" xmlModel.xml scanNumber \n";
		return -666;
	}

	data_model tr;
	std::string fn = argv[1];
	std::string s_scanNo  = argv[2];
	std::cout << "opening file "<< fn <<"\n";
	tr.loadFile(fn);

	std::vector<std::string> ids;
	tr.getAllScansId(ids);
	int scanNo = boost::lexical_cast<int, std::string> (s_scanNo);
	std::cout <<"scan name "<< ids[scanNo]<<"\n";


		return 666;
}