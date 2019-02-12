#include "ColorDistanceMap.h"

namespace
{
	using namespace Classifier;
	using Person = ColorDistanceMap::Person;
	using PeopleToColorDistance = std::unordered_map<Person, std::unordered_map<Person, float>>;

	struct UniformAverageColor
	{
		pcl::RGB shirt;
		pcl::RGB shorts;
	};

	struct UniformColorPoints
	{
		std::vector<pcl::RGB> shirt;
		std::vector<pcl::RGB> shorts;
	};

	// Can't make cluster const reference because getTop() is not const. 
	UniformColorPoints uniformColorPoints(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, pcl::people::PersonCluster<pcl::PointXYZRGB>& cluster)
	{
		// One could make these constexpr tunable. I made them constexpr because it's a fixed scope assignment. 
		constexpr float headPercent = 0.8f;
		constexpr float waistPercent = 0.5f;
		constexpr float kneesPercent = 0.25f;
		constexpr float tolerancePercent = 0.05f;

		const float top = cluster.getTop()(0);
		const float bottom = cluster.getBottom()(0);
		const float difference = top - bottom;
		const float shirtTop = bottom + difference * (headPercent - tolerancePercent);
		const float shirtBottom = bottom + difference * (waistPercent + tolerancePercent);
		const float shortsTop = bottom + difference * (waistPercent - tolerancePercent);
		const float shortsBottom = bottom + difference * (kneesPercent + tolerancePercent);

		UniformColorPoints toReturn;
		for (int index : cluster.getIndices().indices)
		{
			const auto& point = cloud[index];
			if (shirtBottom < point.x && point.x < shirtTop)
				toReturn.shirt.emplace_back(point.r, point.g, point.b);
			else if (shortsBottom < point.x && point.x < shortsTop)
				toReturn.shorts.emplace_back(point.r, point.g, point.b);
		}
		return toReturn;
	}

	pcl::RGB average(const std::vector<pcl::RGB>& rgbs)
	{
		float red = 0;
		float green = 0;
		float blue = 0;
		for (const auto& rgb : rgbs)
		{
			red += rgb.r;
			green += rgb.g;
			blue += rgb.b;
		}
		red /= rgbs.size();
		green /= rgbs.size();
		blue /= rgbs.size();
		return { static_cast<uint8_t>(round(red)),static_cast<uint8_t>(round(green)),static_cast<uint8_t>(round(blue)) };
	}

	float euclideanDistance(const pcl::RGB& first, const pcl::RGB& second)
	{
		int r = first.r - second.r;
		int g = first.g - second.g;
		int b = first.b - second.b;
		return sqrt(r*r + g * g + b * b);
	}

	// Can't make cluster const reference because in uniformColorPoints() it's not const. 
	PeopleToColorDistance colorDistance(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, const std::vector<Person>& people)
	{
		assert(cloud);
		std::unordered_map<Person, UniformAverageColor> averageColors;
		for (const auto& person : people)
		{
			assert(person);
			const auto points = uniformColorPoints(*cloud, *person);
			averageColors[person] = { average(points.shirt), average(points.shorts) };
		}

		PeopleToColorDistance colorDistance;
		for (const auto& personFirst : people)
		{
			for (const auto& personSecond : people)
			{
				const auto shirtColorDistance = euclideanDistance(averageColors[personFirst].shirt, averageColors[personSecond].shirt);
				const auto shortsColorDistance = euclideanDistance(averageColors[personFirst].shorts, averageColors[personSecond].shorts);
				colorDistance[personFirst][personSecond] = std::max(shirtColorDistance, shortsColorDistance);
			}
		}
		return colorDistance;
	}
}

namespace Classifier
{
	ColorDistanceMap::ColorDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<Person> people) :
		d_colorDistance(colorDistance(cloud, people)),
		d_cloud(std::move(cloud)),
		d_people(std::move(people))
	{
	}
}