#ifndef POTENTIAL_GAP_TRJECTORY_TYPE_H
#define POTENTIAL_GAP_TRJECTORY_TYPE_H

#include <tf/tf.h>
#include <string>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_generator_ros_interface.h>

#include <pips_trajectory_msgs/trajectory_point.h>
#include <pips_trajectory_msgs/trajectory_points.h>

namespace potential_gap
{
    typedef pips_trajectory_msgs::trajectory_points trajectory_msg_t;
    
    struct PotentialGapTrajectory : public trajectory_generator::general_trajectory
    {
        std::string frame_id_;
        ros::Time stamp_;
        geometry_msgs::PoseArray trajectory_;
        
        PotentialGapTrajectory(){}
        
        bool hasDuration()
        {
            return false;
        }
        
        ros::Duration getDuration()
        {
            ROS_WARN("No duration defined.");
            return ros::Duration(0);
        }
        
        bool validTrajLength()
        {
            if(trajectory_.poses.size() >= 2)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        
        trajectory_msg_t toMsg() //-> decltype(typename state_type::trajectory_msg_t)
        {
            trajectory_msg_t trajectory_msg;
            //trajectory_msg.points = points;
            trajectory_msg.header.frame_id = frame_id_;
            trajectory_msg.header.stamp = stamp_;
            
            auto& points = trajectory_msg.points;
            
            //using state_msg_t = decltype(state_type::toMsg);
            
            //ni_trajectory::printTrajectory();
            //std::vector<auto> points;
            for(unsigned int i = 0; i < trajectory_.poses.size(); ++i)
            {
                pips_trajectory_msgs::trajectory_point msg;
                msg.time = ros::Duration(0);
                msg.x = trajectory_.poses[i].position.x;
                msg.y = trajectory_.poses[i].position.y;
                
                tf::Quaternion q(
                    trajectory_.poses[i].orientation.x,
                    trajectory_.poses[i].orientation.y,
                    trajectory_.poses[i].orientation.z,
                    trajectory_.poses[i].orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                
                msg.theta = yaw;
                msg.v = 0;
                msg.w = 0;
                
                points.push_back(msg);
            }
            
            return trajectory_msg;
        }
        
        nav_msgs::PathPtr toPathMsg()
        {
            nav_msgs::PathPtr path_msg(new nav_msgs::Path);
            path_msg->header.frame_id = frame_id_;
            path_msg->header.stamp = stamp_;
            
            for(size_t i=0; i < trajectory_.poses.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame_id_;
                pose.header.stamp = stamp_;
                pose.pose = trajectory_.poses[i];
                
                path_msg->poses.push_back(pose);
            }
            
            return path_msg;
        }
        
        geometry_msgs::PoseStamped getPoseStamped(int i)
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame_id_;
            pose.header.stamp = stamp_;
            pose.pose = trajectory_.poses[i];
            
            return pose;
        }
        
        geometry_msgs::Point getPoint(int i)
        {
            geometry_msgs::Point point;
            point.x = trajectory_.poses[i].position.x;
            point.y = trajectory_.poses[i].position.y;
            
            return point;
        }
        
        int num_states()
        {
            return trajectory_.poses.size();
        }
        
        void setFrame(std::string frame_id)
        {
            frame_id_ = frame_id;
        }
        
        void setTime(ros::Time curr_time)
        {
            stamp_ = curr_time;
        }
        
        void setTrajectory(geometry_msgs::PoseArray traj)
        {
            trajectory_ = traj;
        }
        
        std::string getFrame() { return frame_id_; }
        
        geometry_msgs::PoseArray getTrajectory() { return trajectory_; }
    };
    
    typedef std::shared_ptr<PotentialGapTrajectory> pg_trajectory_ptr;
}

#endif
