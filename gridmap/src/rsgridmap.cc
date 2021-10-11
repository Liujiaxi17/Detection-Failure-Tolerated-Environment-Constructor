
#include "rsgridmap.h"
//#include "gridmapconstructor.h"
#include <zzz_perception_msgs/TrackingBoxArray.h>
#include <zzz_perception_msgs/GridMapMsgStamped.h>
#include <zzz_driver_msgs/RigidBodyStateStamped.h>

#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/PointCloud2.h>

#include <math.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>

//#include <ctime>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>


#define FILTER_DIS MAX_DISTANCE + 0.5 
#define CONSIDER_OBS_RANGE PI/2

#define EXPENSION_TIME 1.2
#define DELTAT 0.2
 
namespace rsgrid_map{

rsgridmap::rsgridmap(ros::NodeHandle node, ros::NodeHandle private_nh):rawDataBuffer(new pcl::PointCloud<pcl::PointXYZI>),Constructor(){
    //ROS_INFO("rsgridmap constructed!!!!!!!");
    std::string input_pointcloud2_topic;
    std::string input_egopose_topic;
    std::string input_tf_topic;
    std::string input_obs_topic;
    
    private_nh.param("input_pointcloud2_topic",input_pointcloud2_topic,std::string("/middle/rslidar_points"));
    private_nh.param("input_egopose_topic",input_egopose_topic,std::string("/zzz/navigation/ego_pose"));
    private_nh.param("input_tf_topic",input_tf_topic,std::string("/tf"));
    private_nh.param("input_obs_topic",input_obs_topic,std::string("/zzz/perception/objects_tracked"));
    
    
    std::string output_gridmap_topic;
    std::string output_gridmap_obs_topic;
    private_nh.param("output_gridmap_topic",output_gridmap_topic,std::string("/zzz/perception/grid_map"));
    private_nh.param("output_gridmap_obs_topic",output_gridmap_obs_topic,std::string("/zzz/perception/grid_map_obs"));
    
    pointcloud2_subscriber = node.subscribe(input_pointcloud2_topic,10,&rsgridmap::subscribePointcloud2,
                                        (rsgridmap*)this);
    ego_pose_subscriber = node.subscribe(input_egopose_topic,10,&rsgridmap::subscribeEgostate,
                                        (rsgridmap*)this);
    tf_subscriber = node.subscribe(input_tf_topic,10,&rsgridmap::subscribeTf,
                                        (rsgridmap*)this);
    obs_subscriber = node.subscribe(input_obs_topic,10,&rsgridmap::subscribeObj,
                                        (rsgridmap*)this);
    grid_map_publisher = node.advertise<zzz_perception_msgs::GridMapMsgStamped>(output_gridmap_topic,10);
    grid_map_obs_publisher = node.advertise<zzz_perception_msgs::GridMapMsgStamped>(output_gridmap_obs_topic,10);


    ego_pose_received = false;
    tf_received = false;
    obs_received = false;

}

rsgridmap::~rsgridmap(){};


  void rsgridmap::subscribePointcloud2(const sensor_msgs::PointCloud2ConstPtr &PointCloud2Msg){
    pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudIn(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr TransformedData(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredData_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredData_zx(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredData_zxy(new pcl::PointCloud<pcl::PointXYZI>); 
     
    if (ego_pose_received && tf_received){
        pcl::fromROSMsg(*PointCloud2Msg,*PointCloudIn);
        rsgridmap::transformPoint(PointCloudIn,TransformedData);
        filterPassThrough(TransformedData, FilteredData_z, "z",0.1f,2.3f);
        filterPassThrough(FilteredData_z, FilteredData_zx, "x",-FILTER_DIS,FILTER_DIS);
        filterPassThrough(FilteredData_zx, FilteredData_zxy, "y",-FILTER_DIS,FILTER_DIS);
        filterCondition(FilteredData_zxy,rawDataBuffer);
        rsgridmap::ConstructGridMap();
    } 
  }
   
  void rsgridmap::subscribeEgostate(const zzz_driver_msgs::RigidBodyStateStamped EgoStateMsg){
    ego_pose_received = true;
    EgoPoseBuffer << EgoStateMsg.state.pose.pose.position.x,EgoStateMsg.state.pose.pose.position.y,
                    EgoStateMsg.state.pose.pose.position.z;
    Eigen::Quaterniond q_temp(EgoStateMsg.state.pose.pose.orientation.w,
          EgoStateMsg.state.pose.pose.orientation.x,
          EgoStateMsg.state.pose.pose.orientation.y,
          EgoStateMsg.state.pose.pose.orientation.z);
    EgoEulerBuffer = q_temp.toRotationMatrix().eulerAngles(2,1,0).transpose();
    if (std::abs(EgoEulerBuffer[1])>1.5 && std::abs(EgoEulerBuffer[1])<4.5){ //when transform Matrix or Quaternion to euler angle it may cause problem
      EgoEulerBuffer[0]=EgoEulerBuffer[0]+PI;
      EgoEulerBuffer[1]=EgoEulerBuffer[1]+PI;
      EgoEulerBuffer[2]=EgoEulerBuffer[2]+PI;
    }
    EgoVelocityBuffer << EgoStateMsg.state.twist.twist.linear.x,EgoStateMsg.state.twist.twist.linear.y,
                        EgoStateMsg.state.twist.twist.linear.z;
  }

  void rsgridmap::subscribeTf(const tf2_msgs::TFMessage tf){
    tf_received = true;
    TFtranslationBuffer << tf.transforms[0].transform.translation.x,tf.transforms[0].transform.translation.y,
                           tf.transforms[0].transform.translation.z;
    Eigen::Quaterniond q_temp(tf.transforms[0].transform.rotation.w,
          tf.transforms[0].transform.rotation.x,
          tf.transforms[0].transform.rotation.y,
          tf.transforms[0].transform.rotation.z);
    EgoQuaBuffer = q_temp;
    TFrotationBuffer = q_temp.toRotationMatrix();
  }

  void rsgridmap::subscribeObj(const zzz_perception_msgs::TrackingBoxArray TrackingBoxArrayMsg){
    obs_vector.clear();
    if (!ego_pose_received){return;}
    const zzz_perception_msgs::TrackingBoxArray  output;
    double obs_x, obs_y, obs_distance;
    double ego_x, ego_y, obs_x_car, obs_y_car, obs_theta, obs_alpha;//theta is angle of position, alpha is orientation
    double obs_length, obs_angle;
    double v_rx,v_ry,ego_vx,ego_vy, obs_sideslip;
    double obs_length_x, obs_length_y;
    double delta_p;
    int class_id;
    ego_x = EgoPoseBuffer[0];
    ego_y = EgoPoseBuffer[1];
    ego_vx = EgoVelocityBuffer[0];
    ego_vy = EgoVelocityBuffer[1];
    Eigen::Quaterniond q_temp;
    Eigen::Vector3d obs_euler;
    for (size_t i = 0; i < TrackingBoxArrayMsg.targets.size();i++){
      //if ((i+1)%2==0){continue;}   //manualy increase miss detection rate 
      v_rx = TrackingBoxArrayMsg.targets[i].twist.twist.linear.x - ego_vx;
      v_ry = TrackingBoxArrayMsg.targets[i].twist.twist.linear.y - ego_vy;
      obs_x = TrackingBoxArrayMsg.targets[i].bbox.pose.pose.position.x - ego_x;
      obs_y = TrackingBoxArrayMsg.targets[i].bbox.pose.pose.position.y - ego_y;
      obs_distance = std::sqrt(obs_x*obs_x+obs_y*obs_y);
      
      q_temp = Eigen::Quaterniond(TrackingBoxArrayMsg.targets[i].bbox.pose.pose.orientation.w,
                                  TrackingBoxArrayMsg.targets[i].bbox.pose.pose.orientation.x,
                                  TrackingBoxArrayMsg.targets[i].bbox.pose.pose.orientation.y,
                                  TrackingBoxArrayMsg.targets[i].bbox.pose.pose.orientation.z);
      obs_euler = q_temp.toRotationMatrix().eulerAngles(2,1,0).transpose();
      if (std::abs(obs_euler[1])>1.5 && std::abs(obs_euler[1])<4.5){ //when transform Matrix or Quaternion to euler angle it may cause problem
          obs_euler[0] = obs_euler[0]+PI;
          obs_euler[1] = obs_euler[1]+PI;
          obs_euler[2] = obs_euler[2]+PI;
      }
      //calculate sideslip and enlarge
      obs_sideslip = obs_euler[0] - std::atan2(v_ry,v_rx);
      delta_p = std::sqrt(v_rx*v_rx+v_ry*v_ry)*DELTAT;
      obs_x = obs_x + v_rx * DELTAT/2;
      obs_y = obs_y + v_ry * DELTAT/2;
      obs_length_x = TrackingBoxArrayMsg.targets[i].bbox.dimension.length_x + delta_p*std::abs(std::cos(obs_sideslip));
      obs_length_y = TrackingBoxArrayMsg.targets[i].bbox.dimension.length_y + delta_p*std::abs(std::sin(obs_sideslip));
  
      if (obs_distance > MAX_DISTANCE){        continue;      }
      //I dont know why the transform angle has an error of Pi/60. May there are some errors happen when transforming
      obs_x_car = obs_x * std::cos(EgoEulerBuffer[0]+PI/60) + obs_y * std::sin(EgoEulerBuffer[0]+PI/60);
      obs_y_car = -obs_x * std::sin(EgoEulerBuffer[0]+PI/60) + obs_y * std::cos(EgoEulerBuffer[0]+PI/60);
      obs_theta = std::atan2(obs_y_car,obs_x_car);


      if (std::abs(obs_theta) > CONSIDER_OBS_RANGE){  continue; }

      obs_alpha = obs_euler[0] - EgoEulerBuffer[0];
      obs_length = std::sqrt(obs_length_x*obs_length_x+obs_length_y*obs_length_y)/2.0;// /2.0
      obs_angle = std::atan2(obs_length_y,obs_length_x);
      class_id = TrackingBoxArrayMsg.targets[i].classes[0].classid;
      rsgrid_map::ObsGrid tempObs = {obs_x_car,obs_y_car,obs_distance,obs_alpha,obs_theta,obs_length*EXPENSION_TIME,obs_angle,class_id};
      //if (class_id == 2||class_id==3){continue;}
      obs_vector.push_back(tempObs);
    }
    Constructor.Receive_Obs_Info(obs_vector.begin(),obs_vector.end());
    return;
  }

  void rsgridmap::filterPassThrough(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr &output,
                                          const std::string &axis,
                                          const float &limitMin,
                                          const float &limitMax)
  {
    if(input->empty())
        return;

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(axis);
    pass.setFilterLimits(limitMin, limitMax);
    pass.setNegative(false); //true -->filter limit points
    pass.filter(*output);
  }

  void rsgridmap::filterVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
  {
    if(input->empty())
        return;
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setInputCloud(input);
    voxel.setLeafSize(0.05f,0.05f,0.05f);
    voxel.filter(*output);
  }

  void rsgridmap::filterCondition(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr &output)
  {
    pcl::ConditionalRemoval<pcl::PointXYZI> conditional_removal;
    pcl::ConditionOr<pcl::PointXYZI>::Ptr condition_or(new pcl::ConditionOr<pcl::PointXYZI>());
    condition_or->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::GT,4.0)));
    condition_or->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("x",pcl::ComparisonOps::LT,0.0)));
    condition_or->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::GT,2.2)));
    condition_or->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>("y",pcl::ComparisonOps::LT,-2.2)));
    conditional_removal.setCondition(condition_or);
    conditional_removal.setInputCloud(input);
    conditional_removal.filter(*output);

  }                                  

  void rsgridmap::transformPoint(const pcl::PointCloud<pcl::PointXYZI>::Ptr rawPointData_buffer,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr transformedPointData_buffer )
  {
  
    Eigen::Matrix3d Ego_matrix;
    Ego_matrix << std::cos(EgoEulerBuffer[0]+PI/60),std::sin(EgoEulerBuffer[0]+PI/60),0.,
                  -std::sin(EgoEulerBuffer[0]+PI/60),std::cos(EgoEulerBuffer[0]+PI/60),0.,
                  0.,0.,1.;

    Eigen::Matrix3d sum_rotation = Ego_matrix * TFrotationBuffer;
    Eigen::Vector3d sum_transition = Ego_matrix*(TFtranslationBuffer-EgoPoseBuffer);

    Eigen::Matrix4d Transform1;
    Transform1 << sum_rotation(0,0),sum_rotation(0,1),sum_rotation(0,2),sum_transition(0),
                  sum_rotation(1,0),sum_rotation(1,1),sum_rotation(1,2),sum_transition(1),
                  sum_rotation(2,0),sum_rotation(2,1),sum_rotation(2,2),sum_transition(2),
                  0.,0.,0.,1.;
    pcl::transformPointCloud (*rawPointData_buffer,*transformedPointData_buffer,Transform1);
  }


  void rsgridmap::ConstructGridMap(){

    Constructor.Construct_GridMap(rawDataBuffer);
    zzz_perception_msgs::GridMapMsgStampedPtr GridMapMsgPtr(new zzz_perception_msgs::GridMapMsgStamped);
    zzz_perception_msgs::GridMapMsgStampedPtr GridMapObsMsgPtr(new zzz_perception_msgs::GridMapMsgStamped);

    GridMapMsgPtr->g_map.quaternion.w = rsgridmap::EgoQuaBuffer.w();
    GridMapMsgPtr->g_map.quaternion.x = rsgridmap::EgoQuaBuffer.x();
    GridMapMsgPtr->g_map.quaternion.y = rsgridmap::EgoQuaBuffer.y();
    GridMapMsgPtr->g_map.quaternion.z = rsgridmap::EgoQuaBuffer.z();

    GridMapMsgPtr->g_map.position.x = rsgridmap::EgoPoseBuffer.x();
    GridMapMsgPtr->g_map.position.y = rsgridmap::EgoPoseBuffer.y();
    GridMapMsgPtr->g_map.position.z = rsgridmap::EgoPoseBuffer.z();

    GridMapObsMsgPtr->g_map.quaternion.w = rsgridmap::EgoQuaBuffer.w();
    GridMapObsMsgPtr->g_map.quaternion.x = rsgridmap::EgoQuaBuffer.x();
    GridMapObsMsgPtr->g_map.quaternion.y = rsgridmap::EgoQuaBuffer.y();
    GridMapObsMsgPtr->g_map.quaternion.z = rsgridmap::EgoQuaBuffer.z();

    GridMapObsMsgPtr->g_map.position.x = rsgridmap::EgoPoseBuffer.x();
    GridMapObsMsgPtr->g_map.position.y = rsgridmap::EgoPoseBuffer.y();
    GridMapObsMsgPtr->g_map.position.z = rsgridmap::EgoPoseBuffer.z();

    
    Constructor.getGridMapMessage(GridMapMsgPtr,GridMapObsMsgPtr);
    grid_map_publisher.publish(GridMapMsgPtr);
    grid_map_obs_publisher.publish(GridMapObsMsgPtr);
    ROS_INFO("gridmap construct finished");
  }

}