#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <object_msgs/DetectedObjects.h>

//onlib/server/simple_action_server.h>

#include <tf/transform_listener.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <cmath>
#include <algorithm>

using namespace std;

class ObjectDetectServer{

private:
	ros::NodeHandle nh_;
	tf::TransformListener tf_listener_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	// Parameters from goal
	std::string arm_link_;

	ros::Publisher block_pub_;
	ros::Publisher c_obj_pub_;

	//ROS_INFO("0.59");
	// Parameters from node
	std::vector<double> table_pose_;
	std::vector<geometry_msgs::Pose> blocks_poses;
	std::vector<object_msgs::DetectedObjects> colored_blocks_poses;

	//initialize variables
	//std::BlockDetectionResult result_;
	//std::BlokDetectionFeedback feedback_;

  //void addBlock(float x, float y, float z, float angle, std_msgs::ColorRGBA rgba )
  //{
    //object_msgs::DetectedObjects colored_block_pose;
    //geometry_msgs::Pose block_pose;

    //block_pose.position.x = x;
    //block_pose.position.y = y;
    //block_pose.position.z = z;

    //Eigen::Quaternionf quat(Eigen::AngleAxis<float>(angle, Eigen::Vector3f(0,0,1)));

    //block_pose.orientation.x = quat.x();
    //block_pose.orientation.y = quat.y();
    //block_pose.orientation.z = quat.z();
    //block_pose.orientation.w = quat.w();

    //blocks_poses.push_back(block_pose);

    //colored_block_pose.position = block_pose.position;
    //colored_block_pose.orientation = block_pose.orientation;
    //colored_block_pose.color = rgba;

    //colored_blocks_poses.push_back(colored_block_pose);
  //}


public:
ObjectDetectServer(const std::string name) :
    nh_("~")
  {
    ROS_INFO("0.6");
    // Subscribe to point cloud
    sub_ = nh_.subscribe("/camera_sr300/depth_registered/points", 1, &ObjectDetectServer::cloudCb, this);

    // Publish the filtered point cloud for debug purposes
    //pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZRGB> >("block_output", 1);

    // Publish detected blocks poses
    //block_pub_ = nh_.advertise<geometry_msgs::Pose>("/turtlebot_blocks", 1, true);
}

void cloudCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  { 
     ROS_INFO("1");
    //declare the size of object and the z coordinate of table
    double block_size_ = 0.02;
    double table_height_ = 0.0;
    //colored_blocks_poses[1].header.stamp  = msg->header.stamp;
    nh_.param<std::string>("/block_manipulation_demo/arm_link", arm_link_);
    
    // convert to PCL
    pcl::PointCloud < pcl::PointXYZRGB > cloud;
    pcl::fromROSMsg(*msg, cloud);

    // transform to whatever frame we're working in, probably the arm frame.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    ROS_INFO("1.6");

    tf_listener_.waitForTransform(std::string(arm_link_), cloud.header.frame_id, ros::Time::now(), ros::Duration(1.0));
    ROS_INFO("1.7");

    if (!pcl_ros::transformPointCloud(std::string(arm_link_), cloud, *cloud_transformed, tf_listener_))
    {
      ROS_ERROR("Error converting to desired frame");
      return;
    }
    ROS_INFO("2");

    // Create the segmentation object for the planar model and set all the parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(0.005);
    ROS_INFO("3");
    // Limit to things we think are roughly at the table height.
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("z");

    pass.setFilterLimits(table_height_ + table_pose_[2] - 0.05, table_height_ + table_pose_[2] + block_size_ + 0.05);

    pass.filter(*cloud_filtered);
    if (cloud_filtered->points.size() == 0)
    {
      ROS_ERROR("0 points left");
      return;
    }
    else
      ROS_INFO("Filtered, %d points left", (int ) cloud_filtered->points.size());

    int nr_points = cloud_filtered->points.size ();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud(cloud_filtered);
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0)
      {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        return;
      }

      std::cout << "Inliers: " << (inliers->indices.size()) << std::endl;
      ROS_INFO("4");
      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(inliers);
      extract.setNegative(false);

      // Write the planar inliers to disk
      extract.filter(*cloud_plane);
      std::cout << "PointCloud representing the planar component: "
                << cloud_plane->points.size() << " data points." << std::endl;

      // Remove the planar inliers, extract the rest
      extract.setNegative(true);
      extract.filter(*cloud_filtered);
    }
    ROS_INFO("5");
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.005);
    ec.setMinClusterSize(200); //TODO: this might make a nice parameter: ec.setMinClusterSize(125);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    pub_.publish(cloud_filtered);
    ROS_INFO("6");
    // for each cluster, see if it is a block
    for (size_t c = 0; c < cluster_indices.size(); ++c)
    {
      // find the outer dimensions of the cluster
      float xmin = 0; float xmax = 0;
      float ymin = 0; float ymax = 0;
      float zmin = 0; float zmax = 0;

      unsigned long redSum=0, greenSum=0, blueSum=0, redCount = 0, greenCount = 0, blueCount = 0;

      for (size_t i = 0; i < cluster_indices[c].indices.size(); i++)
      {
          int j = cluster_indices[c].indices[i];
          float x = cloud_filtered->points[j].x;
          float y = cloud_filtered->points[j].y;
          float z = cloud_filtered->points[j].z;
          unsigned long rgb = cloud_filtered->points[j].rgba;

          //Calculate average color of cluster
          //TODO: This may be BGR not RGB
          if ( rgb & 0xff > 128 )
          {
            redSum += rgb & 0xff;
            ++redCount;
          }
          if ( (rgb >> 8) & 0xff > 128)
          {
            greenSum += (rgb >> 8) & 0xff;
            ++greenCount;
          }
          if ( (rgb >> 16) & 0xff > 128 )
          {
            blueSum += (rgb >> 16) & 0xff;
            ++blueCount;
          }

          //Determine the min and max x,y,z for cluster
          if (i == 0)
          {
            xmin = xmax = x;
            ymin = ymax = y;
            zmin = zmax = z;
          }
          else
          {
            xmin = std::min(xmin, x);
            xmax = std::max(xmax, x);
            ymin = std::min(ymin, y);
            ymax = std::max(ymax, y);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
          }
      }
      ROS_INFO("7");
      // Check if these dimensions make sense for the block size specified
      float xside = xmax-xmin;
      float yside = ymax-ymin;
      float zside = zmax-zmin;

      const float tol = 0.01; // 1 cm error tolerance

      // In order to be part of the block, xside and yside must be between
      // blocksize and blocksize*sqrt(2)
      // z must be equal to or smaller than blocksize
      if (xside > block_size_ - tol && xside < block_size_*sqrt(2) + tol &&
          yside > block_size_ - tol && yside < block_size_*sqrt(2) + tol &&
          zside > tol && zside < block_size_ + tol)
      {
        // If so, then figure out the position and the orientation of the block
        float angle = atan(block_size_/((xside + yside)/2));

        if (yside < block_size_)
          angle = 0.0;

        // Then add it to our set
        ROS_INFO("Found new block! x=%.3f y=%.3f z=%.3f", (float) xmin + xside/2.0, (float) ymin + (float) yside/2.0, zmax - block_size_/2.0);
        ROS_INFO_STREAM("Block length x side: " << xside << "m y side: " << yside << "m z side " << zside << "m angle: " << angle);

        xmin += xside/2.0;
        ymin += yside/2.0;
        zmax -= block_size_/2.0;

        //TODO: Don't exclude blocks by hardcoded bounds
        if ( fabs(ymin) > 0.142 )
        {
          ROS_ERROR( "Block Y was outisde bounds" );
          continue;
        }
        ROS_INFO("7");
        ROS_INFO("Adding a new block! x=%.3f y=%.3f z=%.3f", (float) xmin , (float) ymin , (float) zmax );

        std_msgs::ColorRGBA rgba;
        rgba.r = redSum/redCount;
        rgba.g = greenSum/greenCount;
        rgba.b = blueSum/blueCount;

        ROS_WARN_STREAM("New block color RGB: " << rgba.r << ", " << rgba.g << ", " << rgba.b << "; cluster size: " << cluster_indices[c].indices.size() );

        //addBlock(xmin , ymin , zmax , angle, rgba);
      }
      else
      {
        ROS_WARN_STREAM("Block detection failed on cluster " << c << " with size xyz: " << xside << ", " << yside << ", " << zside << "; XYZ eval: " << (xside > block_size_ - tol && xside < block_size_*sqrt(2) + tol) << ", " << (yside > block_size_ - tol && yside < block_size_*sqrt(2) + tol) << ", " << (zside > tol && zside < block_size_ + tol) );
      }

    }

    if (colored_blocks_poses.size() > 0)
    {
      //as_.setSucceeded(result_);
      //block_pub_.publish(result_.blocks);
      ROS_INFO("[block detection] Succeeded!");
    }
    else
    {
      ROS_INFO_STREAM("[block detection] Couldn't find any blocks this iteration! Checked " << cluster_indices.size() << " possible clusters.");
      ros::Duration(2.0).sleep();
    }
  }

};

int main(int argc,char **argv){
	ros::init(argc,argv,"block_detection_server");
        ROS_INFO("0.5");
	ObjectDetectServer server("object_detection");
	ros::spin();
	return 0;
}

