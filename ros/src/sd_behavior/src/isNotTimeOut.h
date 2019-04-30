#ifndef ISNOTTIMEOUT_H
#define ISNOTTIMEOUT_H

#include <behaviortree_cpp/condition_node.h>

#include <string>

namespace sd_behavior
{

class isNotTimeOut
{
        private :
                std::string message;

	public :
		isNotTimeOut();
};

}
#endif // ISNOTTIMEOUT_H
