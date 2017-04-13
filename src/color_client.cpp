//
// Created by carlotta on 13/04/17.
//

//
// Created by carlotta on 21/03/17.
//
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <pcl_ros/point_cloud.h>
#include "pitt_msgs/ColorSrvMsg.h"
#include "pitt_msgs/ClustersOutput.h"
#include <ros/console.h>
#include "pcl/point_types_conversion.h"
#include "pitt_msgs/InliersCluster.h" // for cluster input (sub msg)
using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace std;
typedef pcl::PointCloud< pcl::PointXYZRGB>::Ptr PCLCloudPtrRGB;          // for point cloud smart pointer
typedef pcl::PointCloud< pcl::PointXYZRGB> PCLCloudRGB;                  // for point cloud
typedef vector< InliersCluster> InliersClusters;
typedef boost::shared_ptr< InliersClusters> InliersClustersPtr;
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
    float h;
    srv.request.cloud=cloudToRosMsg( cloud);
    if(client.call(srv))
    {
        color=srv.response.Color.data;
        ROS_INFO("%s color acquired", srv.response.Color.data.c_str());
        //h = srv.response.Hue.data;
        //ROS_INFO("%f Hue computed",h);
        return (true);
    }
    else
    {    ROS_INFO("not able to call the service");
        return(false);
    }
}
void callback(  const ClustersOutputConstPtr& clusterObj){
    InliersClusters clusters = clusterObj->cluster_objs;
    string color;
    for( int j = 0; j < clusters.size(); j++) { // scan all the clusters
        PCLCloudPtrRGB Cloud = cloudForRosMsgRGB(clusters[j].cloud);
        string color;

        callColorSrv(Cloud, color);
    }

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "example1_b");
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
