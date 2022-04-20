/*   

Copyright (C) 2019 Robotemi Ltd Tal Grossman
Modify (M) 2020 Robotemi Ltd George Konno 
 
*/
#include "ros/ros.h"    
#include <geometry_msgs/Point.h>


struct LaserScanType
{
  double scan_distance;
  int intensity;
  double angle_from_robot;
  geometry_msgs::Point scan_point;
};

class LaserType
{
public:
    LaserType();
    ~LaserType();
    void initParams();
    void setMainInputs(std::vector<double> ranges_vector); //consider remove
    void updateOriginalLaserScanVector(const std::vector <float> ranges_vector, const std::vector <float> intesities_vector);
    void UpdateObjectMembers(double start_angle, double end_angle);
    std::vector <double> FilterNanValuesFromVec(std::vector <double> original_vector);
    geometry_msgs::Point getLaserPointFromAngleAndRange(double angle_from_robot, double range_from_robot);
    double getLaserRangeFromRobot(double angle_from_robot); //consider remove
    int ConvertAngleFromRobotToLaserIndex(double angle_from_robot);
    std::vector <double> ConvertRawLaserRangesToLaserTypeVec(const std::vector <float> original_vector);
    double ConvertLaserIndexToAngleFromRobot(int laser_index);
    void PrintLaserFilteredPoints();
    std::vector <LaserScanType> original_laser_scan_vec;
    std::map <double, LaserScanType> raw_laser_scans_by_desired_angles_map;
    std::map <double, LaserScanType> filtered_laser_scans_by_desired_angles_map;
    std::vector <geometry_msgs::Point> erased_points_after_filter; // for debugging
    ros::Time laser_points_time_stamp;
private:
    void calculateRawLaserMapByAnglesFromRobot(double start_angle, double end_angle);
    void calculateFilteredLaserMapByAnglesFromRobot();
    void filterPointsByBasicScanProperties();
    void filterPointsByInconsistentNeighbors();
    void filterPointsByDistanceToNeighbors();
};
