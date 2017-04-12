//
// Created by carlotta on 21/03/17.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include "pitt_msgs/ColorSrvMsg.h"
#include "pitt_msgs/ClustersOutput.h"
#include <ros/console.h>
#include "pcl/point_types_conversion.h"
#include "pitt_msgs/InliersCluster.h" // for cluster input (sub msg)
//#include "../point_cloud_library/pc_manager.h"				 // my static library


using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace std;
typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtrRGB;          // for point cloud smart pointer
typedef pcl::PointCloud< pcl::PointXYZRGB> PCLCloudRGB;                  // for point cloud
const string SRV_NAME_COLOR = "color_srv";
const string NAME_COLOR_RED = "red";
const string NAME_COLOR_BLUE = "blue";
const string NAME_COLOR_GREEN = "green";
const string NAME_COLOR_NONE = "none";
using namespace pitt_msgs;

typedef vector< InliersCluster> InliersClusters;
typedef boost::shared_ptr< InliersClusters> InliersClustersPtr;



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




//functions to check the color of the point cloud starting form its RGB data

//red color
bool color_red(float  hAverage)
{
    if(hAverage > 140){
        return true;
    }
    else {

        return false;
    }
}
//Green color

bool color_green(float  hAverage)
{
    if(hAverage < 150 && hAverage > 100){
        return true;
    }
    else {

        return false;
    }
}

//Blue color
bool color_blue(float  hAverage)
{
    if(hAverage > 200){
        return true;
    }
    else {

        return false;
    }
}


/*
bool callColorSrv(PCLCloudPtrRGB cloud, string color){
    ROS_INFO("acquired cloud");
    NodeHandle n;
    float h ;
    ServiceClient client = n.serviceClient<ColorSrvMsg>(SRV_NAME_COLOR);
    ColorSrvMsg srv;
    srv.request.cloud=cloudToRosMsg( cloud);
    if(client.call(srv))
    {
        color=srv.response.color.data;
        h=srv.response.hue.data;
        ROS_INFO("%s ", srv.response.color.data.c_str());

        return (true);
    }
    else
    {    ROS_INFO("not able to call the service");
        return(false);
    }
}
 */
void callback( const ClustersOutputConstPtr& clusterObj){
    InliersClusters clusters = clusterObj->cluster_objs;
    string color;
    for( int j = 0; j < clusters.size(); j++) { // scan all the clusters
        PCLCloudPtrRGB cloud = cloudForRosMsgRGB(clusters[j].cloud);
        //callColorSrv(Cloud,color);
        string color_name;

        float hAverage = 0;
        //computation of the average RGB value
        //average_color(cloud,&hAverage);
        //ROS_INFO("average color");
        //ROS_INFO("%d",hAverage);
        pcl::PointXYZHSV hsv;
        int cloudSize = cloud->points.size();
        for (int i = 0; i < cloudSize; i++) {
            pcl::PointXYZRGBtoXYZHSV(cloud->points[i], hsv);
            //  ROS_INFO("%d",cloud->points[i].g);
            hAverage = hAverage + hsv.h;

        }

        hAverage = hAverage / cloudSize;
        //hAverage=average;
        ROS_INFO("%f", hAverage);
        // check which color is the point cloud
        if (color_red(hAverage)) {
            color_name = NAME_COLOR_RED;
            ROS_INFO_STREAM("RED" << endl);
        } else if (color_green(hAverage)) {
            color_name = NAME_COLOR_GREEN;
            ROS_INFO_STREAM("GREEN" << endl);
        } else if (color_blue(hAverage)) {
            color_name = NAME_COLOR_BLUE;
            ROS_INFO_STREAM("BLUE" << endl);
        } else {
            color_name = NAME_COLOR_NONE;
            ROS_INFO_STREAM("NONE" << endl);
        }
    }
    //filling the response
    //res.color.data=color_name;
    //res.hue.data=hAverage;



    }
int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_a");
    ros::NodeHandle node;
    // set subscriber to get kinect depth points
    ros:: Subscriber subDepth = node.subscribe ("geometric_tracker/trackedCluster", 1,callback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {


        //ROS_INFO("%s", msg.data.c_str());

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}




