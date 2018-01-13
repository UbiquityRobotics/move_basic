
// Handle sonar range messages and determine distance to obstacle

// Jim Vaughan <jimv@mrjim.com> January 2018

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/Marker.h>
#include "move_basic/obstacle_detector.h"


ObstacleDetector::ObstacleDetector(ros::NodeHandle& nh, 
                                   tf2_ros::Buffer *tf_buffer)
{
    this->tf_buffer = tf_buffer;
    sensor_id = 0;

    line_pub = ros::Publisher(nh.advertise<visualization_msgs::Marker>("/sonar", 1));

    // TODO: make params
    std::string topic_prefix = "sonar_";

    for (int i=0; i<16; i++) {
        std::string topic = str(boost::format{"%1%%2%"} % topic_prefix % i);
        ROS_INFO("Subscribing to %s", topic.c_str());
        subscribers.push_back(nh.subscribe("/bus_server/sensor/" + topic, 1, &ObstacleDetector::callback, this));
    }
}

void ObstacleDetector::callback(const sensor_msgs::Range::ConstPtr &msg)
{
    std::string frame = msg->header.frame_id;
    ROS_INFO("Callback %s %f", frame.c_str(), msg->range);

    // ignore min values
    // XXX do we want to do this?
    if (msg->range <= msg->min_range || msg->range >= msg->max_range) {
       return;
    }

    // create sensor object if this is a new sensor
    std::map<std::string,RangeSensor>::iterator it = sensors.find(frame);
    if (it == sensors.end()) {
        try {
            geometry_msgs::TransformStamped tfs =
                tf_buffer->lookupTransform("base_link", frame, ros::Time(0));
            
            tf2::Transform tf;
            tf2::Vector3 S, A, B, C;
        
            // sensor origin
            geometry_msgs::PointStamped origin;
            origin.header.frame_id = frame;
            origin.point.x = 0;
            origin.point.y = 0;
            origin.point.z = 0;
            geometry_msgs::PointStamped base_origin;
            tf2::doTransform(origin, base_origin, tfs);
            fromMsg(base_origin.point, S);
            ROS_INFO("origin %f %f %f", S.x(), S.y(), S.z());

            // vector normal to sensor - represents the center of a 1m cone
            geometry_msgs::Vector3Stamped normal;
            normal.vector.x = 1.0;
            normal.vector.y = 0.0;
            normal.vector.z = 0.0;
            normal.header.frame_id = frame;
            geometry_msgs::Vector3Stamped base_normal;
            tf2::doTransform(normal, base_normal, tfs);
            fromMsg(base_normal.vector, A);
            ROS_INFO("normal %f %f %f", A.x(), A.y(), A.z());
            //draw_line(S, S+A, 1, 0, 0, sensor_id + 100);

            // vectors at the edges of cone
            double theta = msg->field_of_view / 2.0;
            float x = cos(theta);
            float y = sin(theta);

            geometry_msgs::Vector3Stamped left;
            left.vector.x = x;
            left.vector.y = -y;
            left.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_left;
            tf2::doTransform(left, base_left, tfs);
            fromMsg(base_left.vector, B);
            ROS_INFO("left %f %f %f", B.x(), B.y(), B.z());
            //draw_line(S, S+B, 0, 1, 0);

            geometry_msgs::Vector3Stamped right;
            right.vector.x = x;
            right.vector.y = y;
            right.vector.z = 0.0;
            geometry_msgs::Vector3Stamped base_right;
            tf2::doTransform(right, base_right, tfs);
            fromMsg(base_right.vector, C);
            ROS_INFO("right %f %f %f", C.x(), C.y(), C.z());
            //draw_line(S, S+C, 0, 0, 1);

            RangeSensor sensor(sensor_id++, frame, S, B, C);
            sensors[frame] = sensor;
            sensor.update(msg->range, msg->header.stamp);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
        }
    }
    else {
        RangeSensor& sensor = sensors[frame];
        sensor.update(msg->range, msg->header.stamp);
        draw_line(sensor.origin, sensor.left_vertex, 0, 0, 1, sensor.id + 200);
        draw_line(sensor.origin, sensor.right_vertex, 0, 1, 0, sensor.id + 300);
        draw_line(sensor.left_vertex, sensor.right_vertex, 0.5, 0.5, 0, sensor.id + 400);
    }
}

void ObstacleDetector::draw_line(const tf2::Vector3 &p1, const tf2::Vector3 &p2,
                            float r, float g, float b, int id)
{
    visualization_msgs::Marker line;
    line.type = visualization_msgs::Marker::LINE_LIST;
    line.action = visualization_msgs::Marker::MODIFY;
    line.header.frame_id = "/base_link";
    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0f;
    line.id = id;
    line.scale.x = line.scale.y = line.scale.z = 0.01;
    line.pose.position.x = 0;
    line.pose.position.y = 0;
    line.pose.orientation.w = 1;
    geometry_msgs::Point gp1, gp2;
    gp1.x = p1.x();
    gp1.y = p1.y();
    gp1.z = p1.z();
    gp2.x = p2.x();
    gp2.y = p2.y();
    gp2.z = p2.z();
    line.points.push_back(gp1);
    line.points.push_back(gp2);

    line_pub.publish(line);
}


// check for obstacles - only checks forward direction
float ObstacleDetector::obstacle_dist(float width)
{
    float min_dist = 10.0f;
    float width2 = width / 2.0f;
    //XXX param
    width2 = 0.12;
    ros::Time now = ros::Time::now();

    std::map<std::string,RangeSensor>::iterator it;
    for (it = sensors.begin(); it != sensors.end(); it++) {
        RangeSensor& sensor = it->second;

        ROS_INFO("checking %s", sensor.frame_id.c_str());
        float age = (now - sensor.stamp).toSec();
        if (age < 1.0) {
           float x0 = sensor.left_vertex.x();
           float y0 = sensor.left_vertex.y();
           float x1 = sensor.right_vertex.x();
           float y1 = sensor.right_vertex.y();
           ROS_INFO("age %f dists %f %f", age, x0, x1);
           if (x0 > 0 && -width2 < y0 && y0 < width2) {
               ROS_INFO("left %f %f", x0, y0);
               if (x0 < min_dist) {
                   ROS_INFO("** %s left", sensor.frame_id.c_str());
                   min_dist = x0;
               }
           }
           if (x1 > 0 && -width2 < y1 && y1 < width2) {
               ROS_INFO("right %f %f", x1, y1);
               if (x1 < min_dist) {
                   ROS_INFO("** %s right", sensor.frame_id.c_str());
                   min_dist = x1;
               }
           }
        }
    }
    ROS_INFO("min_dist %f", min_dist);
    draw_line(tf2::Vector3(min_dist, -width2, 0),
              tf2::Vector3(min_dist, width2, 0), 1, 0, 0, 1000);
    return min_dist;
}


RangeSensor::RangeSensor()
{
}

RangeSensor::RangeSensor(int id, std::string frame_id,
                         const tf2::Vector3& origin,
                         const tf2::Vector3& left_vec,
                         const tf2::Vector3& right_vec)
{
    this->id = id;
    this->frame_id = frame_id;
    this->origin = origin;
    this->left_vec = left_vec;
    this->right_vec = right_vec;
    ROS_INFO("Adding sensor %s", frame_id.c_str());
}

void RangeSensor::update(float range, ros::Time stamp)
{
    this->stamp = stamp;
    left_vertex = origin + left_vec * range;
    right_vertex = origin + right_vec * range;
}
