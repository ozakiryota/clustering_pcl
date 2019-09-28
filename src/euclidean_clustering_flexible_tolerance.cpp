#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
/* #include <pcl/segmentation/extract_clusters.h> */
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

class EuclideanClusteringFlexibleTolerance{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		ros::Subscriber sub_pc;
		/*pcl objects*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZ>};
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
		pcl::visualization::PCLVisualizer viewer {"Euclidian Clustering"};
		/*parameters*/
		double ratio_depth_tolerance;
		double min_tolerance;
		double max_tolerance;
		int min_cluster_size;
		int max_cluster_size;
	public:
		EuclideanClusteringFlexibleTolerance();
		void CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg);
		void Clustering(void);
		double ComputeTolerance(const pcl::PointXYZ& point);
		bool CustomCondition(const pcl::PointXYZ& seed_point, const pcl::PointXYZ& candidate_point, float squared_distance);
		void Visualization(void);
};

EuclideanClusteringFlexibleTolerance::EuclideanClusteringFlexibleTolerance()
	:nhPrivate("~")
{
	sub_pc = nh.subscribe("/velodyne_points", 1, &EuclideanClusteringFlexibleTolerance::CallbackPC, this);
	viewer.setBackgroundColor(1, 1, 1);
	viewer.addCoordinateSystem(1.0, "axis");
	viewer.setCameraPosition(0.0, 0.0, 35.0, 0.0, 0.0, 0.0);

	nhPrivate.param("ratio_depth_tolerance", ratio_depth_tolerance, 0.05);
	std::cout << "ratio_depth_tolerance = " << ratio_depth_tolerance << std::endl;
	nhPrivate.param("min_tolerance", min_tolerance, 0.1);
	std::cout << "min_tolerance = " << min_tolerance << std::endl;
	nhPrivate.param("max_tolerance", max_tolerance, 0.5);
	std::cout << "max_tolerance = " << max_tolerance << std::endl;
	nhPrivate.param("min_cluster_size", min_cluster_size, 100);
	std::cout << "min_cluster_size = " << min_cluster_size << std::endl;
}

void EuclideanClusteringFlexibleTolerance::CallbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	/* std::cout << "CALLBACK PC" << std::endl; */

	pcl::fromROSMsg(*msg, *cloud);
	std::cout << "==========" << std::endl;
	std::cout << "cloud->points.size() = " << cloud->points.size() << std::endl;

	max_cluster_size = cloud->points.size();
	clusters.clear();

	Clustering();
	Visualization();
}

void EuclideanClusteringFlexibleTolerance::Clustering(void)
{
	double time_start = ros::Time::now().toSec();

	/*search config*/
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud);
	max_cluster_size = cloud->points.size();
	/*objects*/
	std::vector<pcl::PointIndices> cluster_indices;
	std::vector<bool> processed(cloud->points.size(), false);
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	/*clustering*/
	for(size_t i=0;i<cloud->points.size();++i){
		if(processed[i])	continue;
		/*set seed*/
		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back(i);
		processed[i] = true;
		/*clustering*/
		while(sq_idx < seed_queue.size()){
			/*search*/
			double tolerance = ComputeTolerance(cloud->points[seed_queue[sq_idx]]);
			int ret = kdtree.radiusSearch(cloud->points[seed_queue[sq_idx]], tolerance, nn_indices, nn_distances);
			if(ret == -1){
				PCL_ERROR("[pcl::extractEuclideanClusters] Received error code -1 from radiusSearch\n");
				exit(0);
			}
			/*check*/
			for(size_t j=0;j<nn_indices.size();++j){
				if(nn_indices[j]==-1 || processed[nn_indices[j]])	continue;
				if(CustomCondition(cloud->points[seed_queue[sq_idx]], cloud->points[nn_indices[j]], nn_distances[j])){
					seed_queue.push_back(nn_indices[j]);
					processed[nn_indices[j]] = true;
				}
			}
			sq_idx++;
		}
		/*judge*/
		if(seed_queue.size()>=min_cluster_size && seed_queue.size()<=max_cluster_size){
			pcl::PointIndices tmp_indices;
			tmp_indices.indices = seed_queue;
			cluster_indices.push_back(tmp_indices);
		}
	}
	std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;
	/*extraction*/
	pcl::ExtractIndices<pcl::PointXYZ> ei;
	ei.setInputCloud(cloud);
	ei.setNegative(false);
	for(size_t i=0;i<cluster_indices.size();i++){
		/*extract*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_clustered_points (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
		*tmp_clustered_indices = cluster_indices[i];
		ei.setIndices(tmp_clustered_indices);
		ei.filter(*tmp_clustered_points);
		/*input*/
		clusters.push_back(tmp_clustered_points);
	}

	std::cout << "clustering time [s] = " << ros::Time::now().toSec() - time_start << std::endl;
}

double EuclideanClusteringFlexibleTolerance::ComputeTolerance(const pcl::PointXYZ& point)
{
	double depth = sqrt(
		point.x * point.x
		+ point.y * point.y
		+ point.z * point.z
	);

	double tolerance = ratio_depth_tolerance*depth;
	if(tolerance < min_tolerance)	tolerance = min_tolerance;
	if(tolerance > max_tolerance)	tolerance = max_tolerance;

	return tolerance;
}

bool EuclideanClusteringFlexibleTolerance::CustomCondition(const pcl::PointXYZ& seed_point, const pcl::PointXYZ& candidate_point, float squared_distance){
	return true;
}

void EuclideanClusteringFlexibleTolerance::Visualization(void)
{
	viewer.removeAllPointClouds();

	/*cloud*/
	viewer.addPointCloud(cloud, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
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
		viewer.addPointCloud(clusters[i], name);
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb[0], rgb[1], rgb[2], name);
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
	}
	
	viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "euclidean_clustering_flexible_tolerance");
	
	EuclideanClusteringFlexibleTolerance euclidean_clustering_flexible_tolerance;

	ros::spin();
}
