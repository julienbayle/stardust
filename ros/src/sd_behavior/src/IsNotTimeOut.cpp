// Exemple de base de behaviorTree.cpp pour test
#include "IsNotTimeOut.h"
#include <ctime>

	static unsigned long debut;
	static unsigned long duree;

	BT::NodeStatus RobotNodes::IsNotTimeOut()
	{
		//1000.0 * (c_end-c_start) / CLOCKS_PER_SEC 
		unsigned long temps = 1000.00 * (std::clock() - debut) / CLOCKS_PER_SEC;
                        if(debut == 0)
			{	
				duree = 1000;
				debut = std::clock();
				std::cout << "En cours depuis " << debut << " temps système" << std::endl;
				return BT::NodeStatus::SUCCESS;
			}
			else if(temps >= duree)
			{
				std::cout << "Fin du Timer à " << temps/10 << " sec." << std::endl;
				return BT::NodeStatus::FAILURE;
			}
			else
			{
				std::cout << "Timer en cours depuis " << temps/10 << " sec."  << std::endl;
				return BT::NodeStatus::SUCCESS;
			}		
	}
