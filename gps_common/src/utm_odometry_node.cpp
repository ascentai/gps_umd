/*
 * Translates novatel_gps_msgs/Inspva into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <novatel_gps_msgs/Inspva.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <Eigen/Dense>

#define DEG2RAD M_PI / 180.0

using namespace gps_common;
using Rigid3d = std::pair<Eigen::Vector3d, Eigen::Quaterniond>;

class UTMOdometryNode {
  private:
    ros::NodeHandle node;
    ros::NodeHandle priv_node;

    std::string frame_id, child_frame_id, world_frame_id;
    bool publish_odom_tf, ignore_z;
    Rigid3d world_to_local_frame;

    ros::Publisher odom_pub, world_odom_pub, origin_pub;
    ros::Subscriber inspva_sub;
    tf::TransformBroadcaster odom_tf_broadcaster;
    geometry_msgs::TransformStamped odom_tf;
    nav_msgs::Odometry odom;
    nav_msgs::Odometry odom_world;

    void InitOrigin() {
      geometry_msgs::PoseStamped origin;
      bool use_fixed_origin;
      priv_node.param<bool>("use_fixed_origin", use_fixed_origin, false);

      // if use_fixed_origin is false, wait for a /inspva message
      // and use it as the origin
      if (!use_fixed_origin) {
        auto ins = ros::topic::waitForMessage<novatel_gps_msgs::Inspva>("/inspva", node);
        double northing, easting;
        std::string zone;
        LLtoUTM(ins->latitude, ins->longitude, northing, easting, zone);
        origin.pose.position.x = easting;
        origin.pose.position.y = northing;
        origin.pose.position.z = ins->height;
        origin.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
          ins->roll * DEG2RAD,
          ins->pitch * DEG2RAD,
          (90.0 - ins->azimuth) * DEG2RAD);
      } else {
        // TODO more than parameters, a service call would be better
        // so that we could change the origin at runtime
        priv_node.param<double>("origin_x", origin.pose.position.x, 0);
        priv_node.param<double>("origin_y", origin.pose.position.y, 0);
        priv_node.param<double>("origin_z", origin.pose.position.z, 0);
        // TODO using euler angles might be easier
        priv_node.param<double>("origin_ox", origin.pose.orientation.x, 0);
        priv_node.param<double>("origin_oy", origin.pose.orientation.y, 0);
        priv_node.param<double>("origin_oz", origin.pose.orientation.z, 0);
        priv_node.param<double>("origin_ow", origin.pose.orientation.w, 1);
      }

      if (ignore_z)
        origin.pose.position.z = 0;

      origin.header.frame_id = world_frame_id;
      origin.header.stamp = ros::Time::now();

      PublishOrigin(origin);
    }

    void PublishOrigin(const geometry_msgs::PoseStamped& origin) {
      origin_pub = node.advertise<geometry_msgs::PoseStamped>("/origin", 1, true);
      origin_pub.publish(origin);

      const Eigen::Vector3d translation(origin.pose.position.x,
        origin.pose.position.y,
        origin.pose.position.z);
      const Eigen::Quaterniond rotation(origin.pose.orientation.w,
        origin.pose.orientation.x,
        origin.pose.orientation.y,
        origin.pose.orientation.z);
        world_to_local_frame = std::make_pair(rotation * -translation, rotation);
    }

    void PublishOdomTF(const nav_msgs::Odometry& odom) {
      odom_tf.header.stamp = odom.header.stamp;
      odom_tf.transform.translation.x = odom.pose.pose.position.x;
      odom_tf.transform.translation.y = odom.pose.pose.position.y;
      odom_tf.transform.translation.z = odom.pose.pose.position.z;

      odom_tf.transform.rotation.x = odom.pose.pose.orientation.x;
      odom_tf.transform.rotation.y = odom.pose.pose.orientation.y;
      odom_tf.transform.rotation.z = odom.pose.pose.orientation.z;
      odom_tf.transform.rotation.w = odom.pose.pose.orientation.w;

      odom_tf_broadcaster.sendTransform(odom_tf);
    }

    void INSPVACallBack(const novatel_gps_msgs::InspvaConstPtr& ins) {

      double northing, easting, z;
      std::string zone;
      LLtoUTM(ins->latitude, ins->longitude, northing, easting, zone);
      if (ignore_z)
        z = 0.0;
      else
        z = ins->height;

      Eigen::Vector3d position(easting, northing, z);
      Eigen::Matrix3d rot_mat = world_to_local_frame.second.toRotationMatrix();
      Eigen::Vector3d local_position = rot_mat * position + world_to_local_frame.first;
      if (ignore_z) 
        local_position = Eigen::Vector3d(local_position(0), local_position(1), 0.0);

      Eigen::Quaterniond orientation =
        Eigen::AngleAxisd((90.0 - ins->azimuth) * DEG2RAD, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(ins->pitch * DEG2RAD, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(ins->roll * DEG2RAD, Eigen::Vector3d::UnitX());
      Eigen::Quaterniond local_orientation = world_to_local_frame.second * orientation;

      odom.header.stamp = ins->header.stamp;
      odom.pose.pose.position.x = local_position(0);
      odom.pose.pose.position.y = local_position(1);
      odom.pose.pose.position.z = local_position(2);
      odom.pose.pose.orientation.w = local_orientation.w();
      odom.pose.pose.orientation.x = local_orientation.x();
      odom.pose.pose.orientation.y = local_orientation.y();
      odom.pose.pose.orientation.z = local_orientation.z();

      double velocity = sqrt(pow(ins->east_velocity,2) + pow(ins->north_velocity,2) + pow(ins->up_velocity,2));
      odom.twist.twist.linear.x = velocity;
      odom.twist.twist.linear.y = 0; //TODO:
      odom.twist.twist.linear.z = 0; //TODO:

      // TODO use covariance from inscov
      odom.pose.covariance = {1,0,0,0,0,0,
                0,1,0,0,0,0,
                0,0,1,0,0,0,
                0,0,0,1,0,0,
                0,0,0,0,1,0,
                0,0,0,0,0,1};
      odom_pub.publish(odom);

      if (publish_odom_tf)
        PublishOdomTF(odom);

      // keep twist and covariance matrix
      odom_world.pose = odom.pose;
      odom_world.header.stamp = odom.header.stamp;
      // replace timestamp, position and orientation
      odom_world.pose.pose.position.x = position(0);
      odom_world.pose.pose.position.y = position(1);
      odom_world.pose.pose.position.z = position(2);
      odom_world.pose.pose.orientation.w = orientation.w();
      odom_world.pose.pose.orientation.x = orientation.x();
      odom_world.pose.pose.orientation.y = orientation.y();
      odom_world.pose.pose.orientation.z = orientation.z();
      world_odom_pub.publish(odom_world);
    }

  public:
    UTMOdometryNode() : priv_node("~") {}

    void Run() {
      priv_node.param<std::string>("frame_id", frame_id, std::string("odom"));
      priv_node.param<std::string>("child_frame_id", child_frame_id, std::string("base_link"));
      priv_node.param<std::string>("world_frame_id", world_frame_id, std::string("world"));
      priv_node.param<bool>("publish_odom_tf", publish_odom_tf, false);
      priv_node.param<bool>("ignore_z", ignore_z, false);

      odom.header.frame_id = frame_id;
      odom.child_frame_id = child_frame_id;
      odom_tf.header = odom.header;
      odom_tf.child_frame_id = odom.child_frame_id;
      odom_world.header.frame_id = world_frame_id;
      odom_world.child_frame_id = odom.child_frame_id;

      InitOrigin();

      int pub_queue_size;
      priv_node.param("pub_queue_size", pub_queue_size, 10);
      if(pub_queue_size < 0)
          pub_queue_size = 10;
      odom_pub = node.advertise<nav_msgs::Odometry>("/odom", pub_queue_size);
      world_odom_pub = node.advertise<nav_msgs::Odometry>("/odom_world", pub_queue_size);

      int sub_queue_size;
      priv_node.param("sub_queue_size", sub_queue_size, 10);
      if(sub_queue_size < 0)
        sub_queue_size = 10;
      inspva_sub = node.subscribe("/inspva", sub_queue_size, &UTMOdometryNode::INSPVACallBack, this);
      ros::spin();
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "utm_odometry_node");
  UTMOdometryNode node;
  node.Run();
  return 0;
}