
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>


#include <sensor_msgs/Range.h>


// a single sonar sensor
class SonarSensor
{
    tf2::Vector3 left_vec;
    tf2::Vector3 right_vec;

public:
    std::string frame_id;
    tf2::Vector3 origin;
    // cone vertices from last Range message
    tf2::Vector3 left_vertex;
    tf2::Vector3 right_vertex;
    ros::Time stamp;

    SonarSensor();
    SonarSensor(std::string frame_id,
                const tf2::Vector3& origin,
                const tf2::Vector3& left_vec,
                const tf2::Vector3& right_vec);

    void update(float range, ros::Time stamp);
};


// Handle Range messages from sonar
class SonarRanger
{
   std::map<std::string, SonarSensor> sensors;
   std::vector<ros::Subscriber> subscribers;
   ros::Publisher line_pub;
   tf2_ros::TransformListener listener;
   tf2_ros::Buffer tf_buffer;
   std::map<std::string, int>ids;

   void draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                  float r, float g, float b, int id);

public:
   SonarRanger();
   void initialize();
   void callback(const sensor_msgs::Range::ConstPtr &msg);
   float obstacle_dist(float robot_width);
};

