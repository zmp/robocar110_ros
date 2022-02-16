#include <ros/ros.h>
#include <string>
#include <vector>

std::vector<std::string> nodes;

void noMaster()
{
	for (const auto &node : nodes)
	{
		system(("pkill -f __name:=\\\\b" + node + "\\\\b").data());
	}
}

int main(int argc, char *argv[])
{
	nodes = std::vector<std::string>(argv + 1, argv + argc);

	ros::init(argc, argv, "rc110_master_hold");
	ros::NodeHandle nh;

	ros::Rate rate(10);

	bool connected = true;
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		if (ros::master::check())
		{
			if (!connected) connected = true;
		}
		else if (connected)
		{
			connected = false;
			noMaster();
		}
	}
	return EXIT_FAILURE;
}