#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <deque>
#include "gridmapconstructor.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*PI/180.)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./PI)
#endif

#define MIN(a,b) (a>b?b:a)
#define MAX(a,b) (a>b?a:b)

//#include <ctime>

#define VehicleExpensionMin 0.8
#define VehicleExpensionMax 0.25
#define HumanExpensionMin 0.35
#define HumanExpensionMax 0.0
#define TruckExpensionMin 0.5
#define TruckExpensionMax 0.3
#define CyclistExpensionMin 0.2
#define CyclistExpensionMax 0.2
#define OtherExpensionMin 0.5
#define OtherExpensionMax 0.0

#define VehicleAngleAdd 2.5
#define VehicleAngleMin 2.5
#define HumanAngleAdd 3.5
#define HumanAngleMin 3.5
#define TruckAngleAdd 4.5
#define TruckAngleMin 4.5
#define CyclistAngleAdd 3.5
#define CyclistAngleMin 3.5
#define OtherAngleAdd 3
#define OtherAngleMin 3

#define VehicleId 1
#define HumanId 2
#define TruckId 49
#define CyclistId 3


namespace rsgrid_map
{

bool compare_d(rsgrid_map::PointPole &p1,rsgrid_map::PointPole &p2){
    return p1.distance < p2.distance;
}

bool compare_z(rsgrid_map::PointPole &p1,rsgrid_map::PointPole &p2){
    return p1.z < p2.z;
}

double eu_distance(double x, double y){
    return std::sqrt(x*x+y*y);
}

double point_to_line(double x_1, double y_1, double x_2, double y_2, double k){
    // calculate two lines' cross: y=kx and line(x1,y1)(x2,y2) , return the distance between cross point and 0
    if (std::abs(k*(x_1-x_2)-(y_1-y_2))<0.00001){
        ROS_INFO("The object is too near to ego!!May the position is wrong!!");
        return 0.1;
    }
    return std::abs((x_1*y_2-x_2*y_1)/(k*(x_1-x_2)-(y_1-y_2)))*std::sqrt(1+k*k);
} 

bool index_in_range(int index, double index_1, double index_2, int id){
    int expension_min = 1;
    int expension_max = 1;
    switch (id)
        {
        case VehicleId: // Vehicle
        {   expension_min = VehicleAngleMin;
            expension_max = VehicleAngleAdd;
            break;}
        case HumanId: 
        {   expension_min = HumanAngleMin;
            expension_max = HumanAngleAdd;
            break;}
        case TruckId: 
        {   expension_min = TruckAngleMin;
            expension_max = TruckAngleAdd;
            break;}
        case CyclistId:
        {
            expension_min = CyclistAngleMin;
            expension_max = CyclistAngleAdd;
            break;
        }
        default:{
            expension_min = OtherAngleMin;
            expension_max = OtherAngleAdd;
            break;}
        }

    int min_in = MIN(index_1,index_2)-expension_min;
    int max_in = MAX(index_1,index_2)+expension_max;
    return (min_in-index)*(index-max_in)>0;
}

GridMapConstructor::GridMapConstructor():poles(NUM_OF_GRID),obs_in_poles(NUM_OF_GRID){
    for(int i =0; i<NUM_OF_GRID;i++){
        distances[i] = MAX_DISTANCE;
        distances_obs[i] = MAX_DISTANCE;
    }
}

GridMapConstructor::~GridMapConstructor(){};

//this function receives obs info and transform them in polar coordinate. with some expension related to its class.
void GridMapConstructor::Receive_Obs_Info(std::vector<rsgrid_map::ObsGrid>::iterator be, 
                                          std::vector<rsgrid_map::ObsGrid>::iterator en){
    //receive obs
    for (int i = 0; i < NUM_OF_GRID; i++){
        obs_in_poles[i].clear();
    }
    double l_c1,l_c2,l_s1,l_s2;
    double temp_angle_1,temp_angle_2,temp_d;
    //double d_1,d_2,d_3,d_4;
    double theta_1, theta_2, theta_3, theta_4, index_1, index_2 , index_3, index_4;
    double x_1,x_2,x_3,x_4,y_1,y_2,y_3,y_4;
    double min_d, max_d;
    int min_index, max_index, temp_index;
    double expension_min, expension_max; 
    bool changed = false;
    for (std::vector<rsgrid_map::ObsGrid>::iterator it = be; it != en; it++){
        temp_angle_1 = it->alpha + it->angle;
        temp_angle_2 = it->alpha - it->angle;
        l_c1 = it->length * std::cos(temp_angle_1);
        l_c2 = it->length * std::cos(temp_angle_2);
        l_s1 = it->length * std::sin(temp_angle_1);
        l_s2 = it->length * std::sin(temp_angle_2);
        x_1 = it->x_car + l_c1;
        x_2 = it->x_car + l_c2;
        x_3 = it->x_car - l_c1;
        x_4 = it->x_car - l_c2;
        y_1 = it->y_car + l_s1;
        y_2 = it->y_car + l_s2;
        y_3 = it->y_car - l_s1;
        y_4 = it->y_car - l_s2;
        //d_1 = eu_distance(x_1, y_1);
        theta_1 = std::atan2(y_1,x_1);
        //d_2 = eu_distance(x_2,y_2);
        theta_2 = std::atan2(y_2,x_2);
        //d_3 = eu_distance(x_3,y_3);
        theta_3 = std::atan2(y_3,x_3);
        //d_4 = eu_distance(x_4,y_4);
        theta_4 = std::atan2(y_4,x_4);
        index_1 = RAD2DEG(theta_1)/(ANGLE_RESOLUTION);
        index_2 = RAD2DEG(theta_2)/(ANGLE_RESOLUTION);
        index_3 = RAD2DEG(theta_3)/(ANGLE_RESOLUTION);
        index_4 = RAD2DEG(theta_4)/(ANGLE_RESOLUTION);
        //ROS_INFO("INDEIES raw ARE %f, %f, %f, %f",index_1,index_2,index_3,index_4);
        min_index = int(MIN(MIN(index_1,index_2),MIN(index_3,index_4))-0.5);
        max_index = int(MAX(MAX(index_1,index_2),MAX(index_3,index_4))+1.5);
        switch (it->class_id)
            {
            case VehicleId: // Vehicle
            {   expension_min = VehicleExpensionMin;
                expension_max = VehicleExpensionMax;
                break;}
            case HumanId: 
            {   expension_min = HumanExpensionMin;
                expension_max = HumanExpensionMax;
                break;}
            case TruckId: 
            {   expension_min = TruckExpensionMin;
                expension_max = TruckExpensionMax;
                break;}
            case CyclistId:
            {
                expension_min = CyclistExpensionMin;
                expension_max = CyclistExpensionMax;
                break;            }
            default:{
                expension_min = OtherExpensionMin;
                expension_max = OtherExpensionMax;
                break;}
            }
        // this part considers 6 conditions of the relationship of a rectangle and polar system.
        //may  some simpler method exists
        for (int j = min_index ; j < max_index; j++){
            changed = false;
            min_d = MAX_DISTANCE;
            max_d = 0;

            if (index_in_range(j, index_1, index_2, it->class_id)){
                temp_d = point_to_line(x_1,y_1,x_2,y_2,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                //ROS_INFO("j, d, %d,%f, 1,2", j, temp_d);
                temp_d = point_to_line(x_1,y_1,x_2,y_2,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                //ROS_INFO("j, d, %d,%f, 1,2", j, temp_d);
            }
            if (index_in_range(j, index_1, index_3, it->class_id)){
                temp_d = point_to_line(x_1,y_1,x_3,y_3,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                temp_d = point_to_line(x_1,y_1,x_3,y_3,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
            }
            if (index_in_range(j, index_1, index_4, it->class_id)){
                temp_d = point_to_line(x_1,y_1,x_4,y_4,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                temp_d = point_to_line(x_1,y_1,x_4,y_4,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
            }
            if (index_in_range(j, index_3, index_2, it->class_id)){
                temp_d = point_to_line(x_3,y_3,x_2,y_2,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                temp_d = point_to_line(x_3,y_3,x_2,y_2,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
            }
            if (index_in_range(j, index_4, index_2, it->class_id)){
                temp_d = point_to_line(x_4,y_4,x_2,y_2,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                temp_d = point_to_line(x_4,y_4,x_2,y_2,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
            }
            if (index_in_range(j, index_3, index_4, it->class_id)){
                temp_d = point_to_line(x_3,y_3,x_4,y_4,std::tan(DEG2RAD((j-0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
                temp_d = point_to_line(x_4,y_4,x_3,y_3,std::tan(DEG2RAD((j+0.5)*ANGLE_RESOLUTION)));
                if (temp_d < min_d){min_d = temp_d;changed =true;};
                if (temp_d > max_d){max_d = temp_d;};
            }
            if (!changed){continue;}
            min_d = min_d - expension_min;
            max_d = max_d + expension_max;
            temp_index = (j<0? j+NUM_OF_GRID:j) ;
            rsgrid_map::ObsPole temp = {min_d, max_d};
            obs_in_poles[temp_index].push_back(temp);
        }
    }
}

void GridMapConstructor::Construct_GridMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& rawPoints){
    
    //this function construct grid map
    for(int i =0; i<NUM_OF_GRID;i++){
        distances[i] = MAX_DISTANCE;
        distances_obs[i] = MAX_DISTANCE;
    }
    convert_pc2_to_grid(rawPoints);
    distance_check();
}

void GridMapConstructor::getGridMapMessage(zzz_perception_msgs::GridMapMsgStampedPtr& output,
                                            zzz_perception_msgs::GridMapMsgStampedPtr& output_obs ){
    output->g_map.NUMOFGRID = NUM_OF_GRID;
    output->g_map.ANGLERESOLUTION = float(ANGLE_RESOLUTION);
    output->g_map.MAXDISTANCE = float(MAX_DISTANCE);
    for (int i =0; i<NUM_OF_GRID;i++){
        output->g_map.distancelist.push_back(distances[i]);
    }
    output_obs->g_map.NUMOFGRID = NUM_OF_GRID;
    output_obs->g_map.ANGLERESOLUTION = float(ANGLE_RESOLUTION);
    output_obs->g_map.MAXDISTANCE = float(MAX_DISTANCE);
    for (int i =0; i<NUM_OF_GRID;i++){
        output_obs->g_map.distancelist.push_back(distances_obs[i]);
    }
    //?
   
}

void GridMapConstructor::convert_pc2_to_grid(pcl::PointCloud<pcl::PointXYZI>::Ptr& rawPoints){
    //this function check each point to poles
    for (int i = 0; i<NUM_OF_GRID; i++){
        poles[i].clear();
    }
    //there each point is separately processed. I dont know if there is some package like numpy in c++
    for(unsigned int i = 0; i < rawPoints->points.size(); i++){
        double distance = eu_distance(rawPoints->points[i].y,rawPoints->points[i].x);
        if (distance>=MAX_DISTANCE){
            continue;
        }
        double angle = std::atan2(rawPoints->points[i].y,rawPoints->points[i].x)*180/PI;
        int temp = int(angle/(ANGLE_RESOLUTION) + 0.5);
        int index = temp<0? temp+NUM_OF_GRID : temp;
        double z = rawPoints->points[i].z;
        rsgrid_map::PointPole point_pole ;
        point_pole.distance = distance;
        point_pole.z = z;
        poles[index].push_back(point_pole);
    }
    for(int i =0; i<NUM_OF_GRID;i++){
        sort(poles[i].begin(),poles[i].end(),compare_d);
    }
}

void GridMapConstructor::distance_check(){
    //this function check distance of each pole
    for(int i = 0; i<NUM_OF_GRID;i++){
        distances_obs[i] = pole_check(i);
    }
    int left_i;
    int right_i;
    for (int i = 0; i<NUM_OF_GRID; i++){
        left_i = i<1? NUM_OF_GRID-1:i-1;
        right_i = i>NUM_OF_GRID-2?0:i+1;
        if (std::sqrt(distances[left_i]*distances[left_i]+distances[right_i]*distances[right_i]-2*COS2RES*distances[right_i]*distances[left_i])<DISTANCEONEOBJTHRE){
            distances[i]=(distances[left_i]+distances[right_i])/2;
        }
        if (std::sqrt(distances_obs[left_i]*distances_obs[left_i]+distances_obs[right_i]*distances_obs[right_i]-2*COS2RES*distances_obs[right_i]*distances_obs[left_i])<DISTANCEONEOBJTHRE){
            distances_obs[i]=(distances_obs[left_i]+distances_obs[right_i])/2;
        }
    }
}

double GridMapConstructor::pole_check(int index){
    //this function check distance of a pole

    if (poles[index].empty()){
        return MAX_DISTANCE;
    }

    std::vector<rsgrid_map::PointPole>::iterator head = poles[index].begin();
    std::vector<rsgrid_map::PointPole>::iterator now = poles[index].begin();
    std::vector<rsgrid_map::PointPole>::iterator tail = poles[index].begin()+1;
    std::deque<rsgrid_map::PointPole> heights;
    rsgrid_map::PointPole temp = {now->distance,now->z};
    heights.push_back(temp);
    double min_distance;
    double ground_z = GROUND_HEIGHT_THRE;
    bool is_ground = false;
    double last_ground_distance = 0;
    double deltad = 0.0;
    double mind = 0.0;
    //for visualize gridmap without obs
    bool no_obs_checked = false;


    while (now != poles[index].end()){
        min_distance = now->distance;
        while (tail!=poles[index].end()&&tail->distance <min_distance + DISTANCEONEOBJTHRE ){
            rsgrid_map::PointPole temp = {tail->distance,tail->z};
            heights.push_back(temp);
            tail++;
        }
        while (head->distance < min_distance - DISTANCEONEOBJTHRE){
            heights.pop_front();
            head++;
        }
        deltad = min_distance - last_ground_distance;
        mind = min_distance;
        if (grid_check(heights.begin(),heights.end(),&ground_z,&is_ground,deltad, &mind)){
            if (!no_obs_checked){
                distances[index] = mind;
                no_obs_checked = true;
            }
            if (distance_obs_check(index,mind)){
                return mind;
            }
        }
        if (is_ground ){
            last_ground_distance = min_distance;
            is_ground = false;
        }        
        //avoid check one point many times
        while (now != poles[index].end() && now->distance - min_distance < DISTANCEONEOBJTHRE ){
            now++;
        }
        
    }
    return MAX_DISTANCE;
}

bool GridMapConstructor::grid_check(std::deque<rsgrid_map::PointPole>::iterator be, std::deque<rsgrid_map::PointPole>::iterator en, double* gz, bool* ig,double deltad, double* mind){
    //check if a grid is occupied
    // if is occupied return True
    std::vector<rsgrid_map::PointPole> zs_grid;
    for (std::deque<rsgrid_map::PointPole>::iterator it = be;it<en;++it){
        rsgrid_map::PointPole temp = {it->distance,it->z};
        zs_grid.push_back(temp);
    }
    sort(zs_grid.begin(),zs_grid.end(),compare_z);
    if (zs_grid.empty()||zs_grid.size()==1){
        return false;
    }
    //if obs return True
    double min_z;
    double max_z;
    int num = 0;
    double thisthre = MIN(*gz + deltad * MAX_SLOPE,MAX_GROUND_THRE);
    double min_d = MAX_DISTANCE;
    std::vector<rsgrid_map::PlaneGrid> planes;
    for (std::vector<rsgrid_map::PointPole>::iterator it = zs_grid.begin();it<zs_grid.end();++it){
        min_d = MAX_DISTANCE;
        num = 0;
        min_z = it->z;
        if (min_d > it->distance){
            min_d = it->distance;
        }
        num++;
        while (it+1 != zs_grid.end() && (it+1)->z < it->z + PLANE_SEGMENT_THRE){
            it++;
            num++;
            if (min_d > (it)->distance){
                min_d = (it)->distance;
        }
        }
        max_z = it->z;
        if (num < PLANE_NUM_THRE){
            continue;
        }
        rsgrid_map::PlaneGrid temp_plane = {min_z,max_z,min_d}; 
        planes.push_back(temp_plane);
    }
    
    if (planes.empty()){
        return false;  //empty or all noise
    }
    std::vector<rsgrid_map::PlaneGrid>::iterator it = planes.begin();

    if (planes.size()==1){
        if (it->max_z < thisthre){
            *ig = true;
            *gz = it->max_z;//(it->max_z+it->min_z)/2;
            return false;//ground
        }
        if (it->max_z - it->min_z < 0.3 && it->max_z < thisthre ){
            *ig = true;
            *gz = it->max_z;//(it->max_z+it->min_z)/2;
            return false;//ground
        }
        //ROS_INFO("HERE!!!!001 %f", it->min_d);
        *mind = it->min_d;
        return true;//high ground or obj
    }
    else{
        while(it+1 != planes.end() && (it+1)->max_z < thisthre){
            it++ ;
        }
        if (it+1 == planes.end()){
            *ig = true;
            *gz = it->max_z;//(it->max_z+it->min_z)/2;
            return false;
        }
        if ((it+1)->max_z - (it+1)->min_z < 0.3 && (it+1)->max_z < thisthre && (it+2)==planes.end()){
            *ig = true;
            *gz = (it+1)->max_z;//((it+1)->max_z+(it+1)->min_z)/2;
            return false;
        }
        
        if ((it+1)->min_z - it->max_z > LOWEST_HANGOVER_DRIVABLE){
            *ig = true;
            *gz = it->max_z;//(it->max_z+it->min_z)/2;
            return false;//
        }
        *mind = (it+1)->min_d;
        return true;
    }   
}

bool GridMapConstructor::distance_obs_check(int index, double distance){
    //if not obs return True
    for (std::vector<rsgrid_map::ObsPole>::iterator it =  obs_in_poles[index].begin();
        it != obs_in_poles[index].end();it++){
        
        if ((it->min_d - distance)*(it->max_d - distance) < 0){
            return false;
        }
    }
    return true;
}
}