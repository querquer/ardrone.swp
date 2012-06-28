#include <ros/ros.h>
#include "ardrone_sdk.h"

int main(int argc, char** argv)
{
      std::ostringstream out;
      out << ZAP_CHANNEL_VERT;
      ROS_INFO("bla: %s", out.str().c_str());
}
