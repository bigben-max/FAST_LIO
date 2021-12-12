#include "laserMapping.h"

int main(int argc, char **argv) {

  std::cout << "laser_mapping " << std::endl;
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  
  //------------------------------------------------------------------------------------------------------
  odometry::laserMapping laser_mapping;
  laser_mapping.init();
  laser_mapping.initRos(nh);
  laser_mapping.work();

  return 0;
}
