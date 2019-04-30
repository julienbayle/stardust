using namespace RobotsNode;

BT::NodeStatus CheckBattery()
{
	std::cout << "[ Battery: OK ]" << std::endl;
}

class GripperInterface
{
	public:
		GripperInterface(): _open(true) {}
		
		NodeStatus open() {
			_open = true;
			std::cout << "GripperInteface::open" << std::endl;
			return NodeStatus::SUCCESS;
		}

		NodeStatus close(){
			_open = false;
			std::cout << "GripperInteface::close" << std::endl;
			return NodeStatus::SUCCESS;
		}
	private:
		bool _open; //shared information
}
