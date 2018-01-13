
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


#include <sensor_msgs/Range.h>


// a single sensor with current obstacles
class RangeSensor
{
    tf2::Vector3 left_vec;
    tf2::Vector3 right_vec;

public:
    int id;
    std::string frame_id;
    tf2::Vector3 origin;
    // cone vertices from last Range message
    tf2::Vector3 left_vertex;
    tf2::Vector3 right_vertex;
    ros::Time stamp;

    RangeSensor();
    RangeSensor(int id, std::string frame_id,
                const tf2::Vector3& origin,
                const tf2::Vector3& left_vec,
                const tf2::Vector3& right_vec);

    void update(float range, ros::Time stamp);
};


// Handle Range messages and computes distance to obstacles
class ObstacleDetector
{
   std::map<std::string, RangeSensor> sensors;
   std::vector<ros::Subscriber> subscribers;
   ros::Publisher line_pub;
   tf2_ros::Buffer *tf_buffer;
   int sensor_id;

   void draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                  float r, float g, float b, int id);

public:
   ObstacleDetector(ros::NodeHandle& nh, tf2_ros::Buffer *tf_buffer);
   void callback(const sensor_msgs::Range::ConstPtr &msg);
   float obstacle_dist(float robot_width);
};

