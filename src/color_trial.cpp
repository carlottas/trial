//
// Created by carlotta on 21/03/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include "pitt_msgs/ColorSrvMsg.h"
#include <ros/console.h>
using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace std;
typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtrRGB;          // for point cloud smart pointer
typedef pcl::PointCloud< pcl::PointXYZRGB> PCLCloudRGB;                  // for point cloud
const string SRV_NAME_COLOR = "color_srv";


PointCloud2 cloudToRosMsg( PCLCloudPtrRGB input){
    PointCloud2Ptr cl( new PointCloud2);
    toROSMsg( *input, *cl);
    return( *cl);
}
//to convert ros msg in pcl point cloud with RGB information

PCLCloudPtrRGB cloudForRosMsgRGB( PointCloud2Ptr input){
    PCLCloudPtrRGB cl( new PCLCloudRGB);
    fromROSMsg ( *input, *cl);
    return( cl);
}

PCLCloudPtrRGB cloudForRosMsgRGB ( PointCloud2 input){
    PCLCloudPtrRGB cl( new PCLCloudRGB);
    fromROSMsg ( input, *cl);
    return( cl);
}

bool callColorSrv(PCLCloudPtrRGB cloud, string color){
    ROS_INFO("acquired cloud");
    NodeHandle n;
    ServiceClient client = n.serviceClient<ColorSrvMsg>(SRV_NAME_COLOR);
    ColorSrvMsg srv;
    srv.request.cloud=cloudToRosMsg( cloud);
    if(client.call(srv))
    {
        color=srv.response.color.data;
        ROS_INFO("%s color acquired", srv.response.color.data.c_str());

        return (true);
    }
    else
    {    ROS_INFO("not able to call the service");
        return(false);
    }
}
void callback( const PointCloud2Ptr& input){

    string color;
    PCLCloudPtrRGB Cloud = cloudForRosMsgRGB(input);
    callColorSrv(Cloud,color);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_a");
    ros::NodeHandle node;
    // set subscriber to get kinect depth points
    ros:: Subscriber subDepth = node.subscribe ("/cameraB/depth_registered/points", 1,callback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {


        //ROS_INFO("%s", msg.data.c_str());

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}