
#include "csv.h"

#include <fstream>

void    InitDFile()
{
	std::ofstream output("./uwbsim.csv", std::ios::trunc);
	output << "id,tagId,baseId,dist\n";
}
