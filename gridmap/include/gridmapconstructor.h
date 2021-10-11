/** @file

    This class construct gridmap by points and objs detected by CNN

*/
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <zzz_perception_msgs/GridMapMsgStamped.h>
#include <zzz_perception_msgs/TrackingBoxArray.h>
#include <zzz_perception_msgs/TrackingBox.h>
#include <algorithm>
#include <deque>

#ifndef PI
#define PI 3.14159265358979323846
#endif
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*PI/180.)
#endif

#define MAX_SLOPE 0.07
#define MAX_GROUND_THRE 1.0

#define NUM_OF_GRID 360
#define ANGLE_RESOLUTION  360./NUM_OF_GRID
#define MAX_DISTANCE  25.0

#define DISTANCEONEOBJTHRE  0.3
#define DISTANCE_MAX_OBJ  DISTANCEONEOBJTHRE/sin(DEG2RAD(ANGLE_RESOLUTION))

#define PLANE_SEGMENT_THRE  0.3
#define GROUND_HEIGHT_THRE  0.4
#define LOWEST_HANGOVER_DRIVABLE  1.5
#define PLANE_NUM_THRE  2

#define COS2RES  std::cos(2*DEG2RAD(ANGLE_RESOLUTION))

#define HALF_CAR_WIDTH  1.7/2.
#define HALF_CAR_LENGTH  4./2.
#define CAR_CENTER_HEIGHT  1.

namespace rsgrid_map
{

#ifndef PointPole
struct PointPole{
    double distance;
    double z;
    
};
#endif

#ifndef PlaneGrid
struct PlaneGrid{
    double min_z;
    double max_z;
    double min_d;
    //int index;
};
#endif

#ifndef GroundPoint
struct GroundPoint{
    double distance;
    double z;
    int index;
};
#endif

#ifndef ObsGrid
struct ObsGrid{
    double x_car;
    double y_car;
    double distance;
    double alpha;
    double theta;
    double length;
    double angle;
    int class_id;
};
#endif

#ifndef ObsPole
struct ObsPole{
    double min_d;
    double max_d;
};
#endif



class GridMapConstructor{

private:
    
    void convert_pc2_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr& rawPoints);
    void distance_check();
    
    double pole_check(int index);
    bool grid_check(std::deque<rsgrid_map::PointPole>::iterator be, std::deque<rsgrid_map::PointPole>::iterator en, 
                                double* gz, bool* ig, double deltad, double* mind);
    bool distance_obs_check(int index, double distance);

    double distances[NUM_OF_GRID];
    double distances_obs[NUM_OF_GRID];
    std::vector<std::vector<rsgrid_map::PointPole>> poles; 
    std::vector<std::vector<rsgrid_map::ObsPole>> obs_in_poles; 
    

public:

    GridMapConstructor();
    ~GridMapConstructor();

    //this function receives obs info and convert them in polar coordinate
    void Receive_Obs_Info(std::vector<rsgrid_map::ObsGrid>::iterator be, 
                          std::vector<rsgrid_map::ObsGrid>::iterator en);
    //this function construct grid map by both the pointcloud and the obs info                     
    void Construct_GridMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& rawPoints);
    //this function convert grid map to msg. Both the grid map with and without obj info. 
    //output_obs means the grid map considering the obj, while the output means the grid map without the obj info. 
    void getGridMapMessage(zzz_perception_msgs::GridMapMsgStampedPtr& output ,
                            zzz_perception_msgs::GridMapMsgStampedPtr& output_obs);

};
}