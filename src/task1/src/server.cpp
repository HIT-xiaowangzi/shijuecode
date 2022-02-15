#include "ros/ros.h"
#include <task1/GetPoint.h>
// service回调函数,输入参数req,输出参数res
const float baseHeight=0.88;//机器人坐标原点距离地面的距离(gazebo中测定，回实验室需重新验证)
const float tableHeight=0.80;//桌面高度（比赛时需重新测定）
const float blockHeight[7]={0.06,0.03,0.06,0.03,0.06,0.015,0.03};//不同类别对应的木块高度
const float Rotation_Matrix[3][3]={{0.546527,0.015354,0.383803},{-0.057502,0.663221,0.055349},{-0.379781,-0.078321,0.543934}};//旋转矩阵 由标定得到
const float Translation_Matrix[3]={0.456101,0.073031,0.124617};//平移矩阵 由标定得到
const float fx=603.149536;//相机内参
const float fy=602.093201;//相机内参
const float x_0=325.593140;//相机内参
const float y_0=237.004852;//相机内参
geometry_msgs::Pose Callback(uint16_t x,uint16_t y,float height)
{
    float x_div_z=(x-x_0)/fx;
    float y_div_z=(y-y_0)/fy;
    float world_z=baseHeight-tableHeight-height;
    float camera_z=(world_z-Translation_Matrix[2])/
    (Rotation_Matrix[2][0]*x_div_z+Rotation_Matrix[2][1]*y_div_z+Rotation_Matrix[2][2]);
    ROS_INFO("camera_z:[%f]",camera_z);
    float world_x=(Rotation_Matrix[0][0]*x_div_z+Rotation_Matrix[0][1]*y_div_z+Rotation_Matrix[0][2]) 
    *camera_z+Translation_Matrix[0];
    float world_y=(Rotation_Matrix[1][0]*x_div_z+Rotation_Matrix[1][1]*y_div_z+Rotation_Matrix[1][2])
    *camera_z+Translation_Matrix[1];
    geometry_msgs::Pose ans;
    ans.position.x=world_x;
    ans.position.y=world_y;
    ans.position.z=world_z;
    ans.orientation.w=0;//方位信息由末端补偿来确定
    ans.orientation.x=0;
    ans.orientation.y=0;
    ans.orientation.z=0;
    return ans;
}
bool PointTransform(task1::GetPoint::Request &req,task1::GetPoint::Response &res)
{
    int size=req.result.MyLists.size();
    geometry_msgs::Pose tempPose[3];
    if(size>3)
    {
        ROS_INFO("The service input is too many!");
    }
    else if(size==3)
    {
        tempPose[0]=Callback(req.result.MyLists[0].x,req.result.MyLists[0].y,
        blockHeight[req.result.MyLists[0].class_-1]/2+blockHeight[req.result.MyLists[1].class_-1]);
        tempPose[1]=Callback(req.result.MyLists[1].x,req.result.MyLists[1].y,
        blockHeight[req.result.MyLists[1].class_-1]/2);
        tempPose[2]=Callback(req.result.MyLists[2].x,req.result.MyLists[2].y,
        blockHeight[req.result.MyLists[2].class_-1]/2);
        res.Poses.poses.push_back(tempPose[0]);
        res.Poses.poses.push_back(tempPose[1]);
        res.Poses.poses.push_back(tempPose[2]);
    }
    else
    {
        for(int i=0;i<size;i++)
        {
            tempPose[i]=Callback(req.result.MyLists[i].x,req.result.MyLists[i].y,
            blockHeight[req.result.MyLists[i].class_-1]/2);
            res.Poses.poses.push_back(tempPose[i]);
        }
    }
    
    return true;
}
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "task1_server");
    // 创建节点句柄
    ros::NodeHandle n;
    // 创建一个名为add_two_ints的server,注册回调函数add()
    ros::ServiceServer service = n.advertiseService("task1_service", PointTransform);
    // 循环等待回调函数
    ROS_INFO("task1_Server is ready!");
    ros::spin();
    return 0;
}
