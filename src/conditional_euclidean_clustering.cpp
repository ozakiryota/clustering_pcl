#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

class ConditionalEuclideanClustering{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*pcl objects*/
		pcl::visualization::PCLVisualizer viewer {"Conditional Euclidian Clustering"};
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud {new pcl::PointCloud<pcl::PointNormal>};
		std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> clusters;
		/*parameters*/
		double cluster_tolerance;
		int min_cluster_size;
	public:
		ConditionalEuclideanClustering();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Clustering(void);
		static bool CustomCondition(const pcl::PointNormal& seedPoint, const pcl::PointNormal& candidatePoint, float squaredDistance);
		void Visualization(void);
};

ConditionalEuclideanClustering::ConditionalEuclideanClustering()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/normals", 1, &ConditionalEuclideanClustering::CallbackPC, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(0.8, "axis");
	viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

	nhPrivate.param("cluster_tolerance", cluster_tolerance, 0.1);
	nhPrivate.param("min_cluster_size", min_cluster_size, 100);
	std::cout << "cluster_tolerance = " << cluster_tolerance << std::endl;
	std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void ConditionalEuclideanClustering::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "==========" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	clusters.clear();
	Clustering();
	Visualization();
}

void ConditionalEuclideanClustering::Clustering(void)
{
	double time_start = ros::Time::now().toSec();

	/*clustering*/
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::ConditionalEuclideanClustering<pcl::PointNormal> cec(true);
	cec.setInputCloud(cloud);
	cec.setConditionFunction(&CustomCondition);
	cec.setClusterTolerance(cluster_tolerance);
	cec.setMinClusterSize(min_cluster_size);
	cec.setMaxClusterSize(cloud->points.size());
	cec.segment(cluster_indices);
	// cec.getRemovedClusters (small_clusters, large_clusters);

	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;

	/*dividing*/
	pcl::ExtractIndices<pcl::PointNormal> ei;
	ei.setInputCloud(cloud);
	ei.setNegative(false);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointNormal>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		/*input*/
		clusters.push_back(tmp_clustered_points);
	}

	std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

bool ConditionalEuclideanClustering::CustomCondition(const pcl::PointNormal& seedPoint, const pcl::PointNormal& candidatePoint, float squaredDistance)
{
	Eigen::Vector3d N1(
		seedPoint.normal_x,
		seedPoint.normal_y,
		seedPoint.normal_z
	);
	Eigen::Vector3d N2(
		candidatePoint.normal_x,
		candidatePoint.normal_y,
		candidatePoint.normal_z
	);
	double angle = acos(N1.dot(N2)/N1.norm()/N2.norm());

	const double threshold_angle = 2.0;	//[deg]
	if(angle/M_PI*180.0 < threshold_angle)	return true;
	else	return false;
}

void ConditionalEuclideanClustering::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*normals*/
	viewer.addPointCloudNormals<pcl::PointNormal>(cloud, 1, 0.5, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, "cloud");
	/*clusters*/
	double rgb[3] = {};
	const int channel = 3;
	const double step = ceil(pow(clusters.size()+2, 1.0/(double)channel));	//exept (000),(111)
	const double max = 1.0;
	for(size_t i=0;i<clusters.size();i++){
		std::string name = "cluster_" + std::to_string(i);
		rgb[0] += 1/step;
		for(int j=0;j<channel-1;j++){
			if(rgb[j]>max){
				rgb[j] -= max + 1/step;
				rgb[j+1] += 1/step;
			}
		}
		/* std::cout << "step = " << step << std::endl; */
		/* std::cout << name << ": (r,g,b) = " << rgb[0] << ", " << rgb[1] << ", " << rgb[2] << std::endl; */
		/*input*/
		viewer.addPointCloudNormals<pcl::PointNormal>(clusters[i], 1, 0.5, name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, name);
	}

	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering");
	
	ConditionalEuclideanClustering euclidean_clustering;

	ros::spin();
}
