#include "ColorDistanceMap.h"
#include "GroupClustering.h"

#include "pcl_people/ground_based_people_detection_app.h"

#include <iostream>
#include <fstream>
#include <vector>

namespace Helpers
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		viewer->setBackgroundColor(0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "sample cloud");
		viewer->addCoordinateSystem(1.0);
		viewer->initCameraParameters();
		return (viewer);
	}

	void readInPointCloud(const std::string& filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		auto pointCloudData = ifstream{ filename };

		float x{};
		float y{};
		float z{};
		int red{};
		int blue{};
		int green{};

		while (pointCloudData >> x >> y >> z >> red >> blue >> green)
		{
			constexpr float feetToMeters = 0.3048f;

			pcl::PointXYZRGB p;
			p.x = feetToMeters * y;
			p.y = feetToMeters * x;
			p.z = feetToMeters * z;
			p.r = static_cast<uint8_t>(red);
			p.g = static_cast<uint8_t>(green);
			p.b = static_cast<uint8_t>(blue);
			cloud->push_back(std::move(p));
		}
	}

	std::vector<Classifier::ColorDistanceMap::Person> getPeopleClustersUsingPclPeopleLibrary(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
		float min_height, float max_height)
	{
		// Algorithm parameters:
		constexpr float voxel_size = 0.06;

		// People detection app initialization:
		pcl::people::GroundBasedPeopleDetectionApp<pcl::PointXYZRGB> people_detector;    // people detection object
		people_detector.setVoxelSize(voxel_size);                        // set the voxel size
		people_detector.setPersonClusterLimits(min_height, max_height, 0.1, 8.0);  // set person classifier
		people_detector.setSensorPortraitOrientation(true);             // set sensor orientation to vertical

		// Perform people detection on the new cloud:
		Eigen::VectorXf ground_coeffs{ 4 };
		ground_coeffs << 1, 0, 0, 0;
		std::vector<pcl::people::PersonCluster<pcl::PointXYZRGB>> clusters;   // vector containing persons clusters
		people_detector.setInputCloud(cloud);
		people_detector.setGround(ground_coeffs);                    // set floor coefficients
		std::vector<pcl::PointIndices> cluster_indices;
		people_detector.compute(clusters, cluster_indices);

		std::vector<Classifier::ColorDistanceMap::Person> newClusters;
		for (const auto& cluster : clusters)
			newClusters.push_back(std::make_shared<pcl::people::PersonCluster<pcl::PointXYZRGB>>(cluster));
		return newClusters;
	}
}

int main(int argc, char** argv)
{
	using namespace Classifier;
	using namespace Helpers;

	auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	readInPointCloud(argv[1], cloud);

	constexpr float min_height = 1.5;
	constexpr float max_height = 3.0; // Account for cases when a player jumps.
	auto clusters = getPeopleClustersUsingPclPeopleLibrary(cloud, min_height, max_height);
	auto clusterTooShort = [min_height](Classifier::ColorDistanceMap::Person cluster) {
		return cluster->getTop()(0) - cluster->getBottom()(0) < min_height;
	};
	clusters.erase(std::remove_if(clusters.begin(), clusters.end(), clusterTooShort),clusters.end());

	ColorDistanceMap colorMap{ cloud, clusters };
	GroupClustering colorClustering{ colorMap };
	auto division = colorClustering.tryBuildMinimumValidDivision();

	/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = rgbVis(cloud);
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	*/

	return 0;
}
