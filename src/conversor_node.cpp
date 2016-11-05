#include <stdlib.h>
#include "ConversorNode.h"

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "conversor_node");
  ConversorNode node(new ros::NodeHandle());
  node.spin();
  return EXIT_SUCCESS;
}
