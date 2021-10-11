
/** @file

    This class preprocess PointCloud2 for constructing GridMap

*/


#include <sensor_msgs/PointCloud2.h>
#include <zzz_driver_msgs/RigidBodyStateStamped.h>
#include <tf2_msgs/TFMessage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>

#include "gridmapconstructor.h"

//headers in Autowae Health Checker
//#include <health_checker/node_status_publisher.h>

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace rsgrid_map
{
class rsgridmap
{
public:
  rsgridmap(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~rsgridmap();

  void subscribePointcloud2(const sensor_msgs::PointCloud2ConstPtr &PointCloud2Msg);
  void subscribeEgostate(const zzz_driver_msgs::RigidBodyStateStamped EgoStateMsg);
  void subscribeTf(const tf2_msgs::TFMessage tf);
  void subscribeObj(const zzz_perception_msgs::TrackingBoxArray TrackingBoxArrayMsg);

private:

  void ConstructGridMap();
  
  //point cloud data preprocess. 
  //TODO: the preprocess function need optimizing
  //1 point just in front of the car (seems the point reflected by the top of the car) is filtered but sometimes it failed
  //2 to decrease the number of points, points with a height(z) too high or too low will be filtered but it may fail when slope 
  void transformPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr rawPointData_buffer,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPointData_buffer);
  void filterPassThrough(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                         const std::string &axis,
                         const float &limitMin,
                         const float &limitMax);
  void filterVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                         pcl::PointCloud<pcl::PointXYZI>::Ptr &output);
  void filterCondition(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr &output);                      
  
  // msg buffer
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawDataBuffer;
  Eigen::Vector3d EgoPoseBuffer;
  Eigen::Vector3d EgoVelocityBuffer;
  Eigen::Vector3d EgoEulerBuffer;
  Eigen::Vector3d TFtranslationBuffer;
  Eigen::Matrix3d TFrotationBuffer;
  Eigen::Quaterniond EgoQuaBuffer;
  
  //the node runs only when all msgs have been received.
  bool ego_pose_received ;
  bool tf_received ;
  bool obs_received;
  
  ros::Subscriber pointcloud2_subscriber;
  ros::Subscriber ego_pose_subscriber; 
  ros::Subscriber tf_subscriber; 
  ros::Subscriber obs_subscriber;
  ros::Publisher grid_map_publisher;
  ros::Publisher grid_map_obs_publisher;

  GridMapConstructor Constructor;
  std::vector<rsgrid_map::ObsGrid> obs_vector;

};

}  // namespace rsgrid_map

