#pragma once

#include "pcl_people/ground_based_people_detection_app.h"

#include <vector>
#include <unordered_map>

namespace SecondSpectrum
{
	struct ColorDistanceMap
	{
		using Person = std::shared_ptr<pcl::people::PersonCluster<pcl::PointXYZRGB>>;

		// Working with many std::shared_ptr can become performance bottleneck. But you can't say upfront without profiler.
		ColorDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<Person> people);

		const std::unordered_map<Person, std::unordered_map< Person, float>> d_colorDistance;
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr d_cloud;
		const std::vector<Person> d_people;
	};
}