#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

class ComputeVelocity
{
public:
    //Parent Constructor used for the initializations
    ComputeVelocity(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubVel()
        , m_listenPose()
    {
        ros::NodeHandle nh;
        m_listenPose.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0));
        m_pubVel = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
        //m_subscribePose = nh.subscribe("pose", 1, &ComputeVelocity::getPoseCallback, this);
    }
    
    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &ComputeVelocity::iteration, this);
        ros::spin();
    }

private:
    /*void getPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_getPose = *msg;
    }*/

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        tf::StampedTransform transform;
        m_listenPose.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

        /*ROS_INFO("x = %f", transform.getOrigin().x());
        ROS_INFO("y = %f", transform.getOrigin().y());
        ROS_INFO("z = %f", transform.getOrigin().z());*/

        //ROS_INFO("dt = %f", dt);

        //Calculate velocity here
        for (int i = 1; i <= 10; i++)
        {
            tempX += transform.getOrigin().x();
            tempY += transform.getOrigin().y();
            tempZ += transform.getOrigin().z();
        }
        currX = tempX/10;
        tempX = 0.0;
        dVx = (currX - prevX)/dt;
        prevX = currX;

        currY = tempY/10;
        tempY = 0.0;
        dVy = (currY - prevY)/dt;
        prevY = currY;

        currZ = tempZ/10;
        tempZ = 0.0;
        dVz = (currZ - prevZ)/dt;
        prevZ = currZ;

        //ROS_INFO ("dVx = %f, dVy = %f, dVz = %f", dVx, dVy, dVz);

        geometry_msgs::TwistStamped computedTwist;
        computedTwist.header.stamp = transform.stamp_;
        computedTwist.header.frame_id = m_worldFrame;
        computedTwist.twist.linear.x = dVx;
        computedTwist.twist.linear.y = dVy;
        computedTwist.twist.linear.z = dVz;

        computedTwist.twist.angular.x = 0.0;
        computedTwist.twist.angular.y = 0.0;  
        computedTwist.twist.angular.z = 0.0;  

        //ROS_INFO("vel_x = %f", computedTwist.twist.linear.x);
        m_pubVel.publish(computedTwist);
    }

private:
    std::string m_worldFrame;
    std::string m_frame;
    tf::TransformListener m_listenPose;
    ros::Publisher m_pubVel;

    float tempX = 0.0, currX = 0.0, dVx = 0.0, prevX = 0.0;
    float tempY = 0.0, currY = 0.0, dVy = 0.0, prevY = 0.0;
    float tempZ = 0.0, currZ = 0.0, dVz = 0.0, prevZ = 0.0;
    
    //geometry_msgs::PoseStamped m_getPose;
    //ros::Subscriber m_subscribePose;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "computed_vel");
    ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);

    ComputeVelocity computeVelocity(worldFrame, frame, n);
    computeVelocity.run(frequency);

    return 0;
}