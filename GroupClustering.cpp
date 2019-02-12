#include "GroupClustering.h"

namespace
{
	using namespace Classifier;
	using Group = NbaDivision::Group;
	using Person = ColorDistanceMap::Person;
	using Pair = std::pair<Person, Person>;

	NbaDivision buildDivisionFromGroups(const std::list<Group>& groups)
	{
		NbaDivision division;
		std::list<Group> sortedGroups = groups;
		auto bySize = [](const Group& left, const Group& right) { return left.size() < right.size(); };
		sortedGroups.sort(bySize);
		division.d_players[0] = { sortedGroups.back().cbegin(),sortedGroups.back().cend() };
		sortedGroups.pop_back();
		if (!sortedGroups.empty())
		{
			division.d_players[1] = { sortedGroups.back().cbegin(),sortedGroups.back().cend() };
			sortedGroups.pop_back();
		}
		if (!sortedGroups.empty())
		{
			division.d_referees = { sortedGroups.back().cbegin(),sortedGroups.back().cend() };
			sortedGroups.pop_back();
		}
		if (!sortedGroups.empty())
		{
			division.d_referees = { sortedGroups.back().cbegin(),sortedGroups.back().cend() };
			sortedGroups.pop_back();
		}
		return division;
	}

	void mergeGroups(std::list<Group>& groups, std::list<Group>::iterator firstGroup, std::list<Group>::iterator secondGroup)
	{
		if (firstGroup == secondGroup)
			return;
		static_assert(NbaDivision::MinimumNumberOfPlayersInTeam > NbaDivision::MinimumNumberOfReferees, "Referees more than players");
		if (firstGroup->size() >= NbaDivision::MinimumNumberOfPlayersInTeam || secondGroup->size() >= NbaDivision::MinimumNumberOfPlayersInTeam)
			return;
		if (count_if(groups.cbegin(), groups.cend(), [](const Group& group) {
			return group.size() >= NbaDivision::MinimumNumberOfPlayersInTeam;
		}) >= NbaDivision::NumberOfTeams)
		{
			if (firstGroup->size() >= NbaDivision::MinimumNumberOfReferees || secondGroup->size() >= NbaDivision::MinimumNumberOfReferees)
				return;
		}
		firstGroup->insert(secondGroup->cbegin(), secondGroup->cend());
		groups.erase(secondGroup);
	}

	std::multimap<float, Pair> getSortedDistancesFromMap(const ColorDistanceMap& colorMap)
	{
		std::multimap<float, Pair> sortedDistances;
		for (const auto& personFirst : colorMap.d_colorDistance)
		{
			for (const auto& personSecond : personFirst.second)
			{
				if (personFirst.first < personSecond.first)
				{
					float distance = personSecond.second;
					sortedDistances.emplace(distance, Pair{ personFirst.first, personSecond.first });
				}
			}
		}
		return sortedDistances;
	}
}

namespace Classifier
{
	GroupClustering::GroupClustering(ColorDistanceMap colorMap) :
		d_colorMap(std::move(colorMap))
	{
		d_currentDivision.d_unknown = { d_colorMap.d_people.begin(), d_colorMap.d_people.end() };
	}

	const ColorDistanceMap& GroupClustering::getColorDistanceMap() const
	{
		return d_colorMap;
	}

	const NbaDivision& GroupClustering::getCurrentDivision() const
	{
		return d_currentDivision;
	}

	const NbaDivision& GroupClustering::tryBuildMinimumValidDivision()
	{
		assert(d_colorMap.d_people.size() >= NbaDivision::TotalMinimumNumberOfPeople);
		auto currentSortedDistances = getSortedDistancesFromMap(d_colorMap);
		d_currentDivision = {};
		if (d_colorMap.d_people.empty())
			return d_currentDivision;
		std::list<Group> groups;
		for (const auto& person : d_colorMap.d_people)
			groups.push_back({ person });
		while (!currentSortedDistances.empty())
		{
			const Person& first = currentSortedDistances.begin()->second.first;
			const Person& second = currentSortedDistances.begin()->second.second;
			std::list<Group>::iterator firstGroup = groups.end();
			std::list<Group>::iterator secondGroup = groups.end();
			for (auto it = groups.begin(); it != groups.end(); ++it)
			{
				if (it->count(first))
					firstGroup = it;
				if (it->count(second))
					secondGroup = it;
			}
			assert(firstGroup != groups.end());
			assert(secondGroup != groups.end());
			// It's not the most efficient algorithm. But we don't care since there are not many clusters.
			mergeGroups(groups, firstGroup, secondGroup);
			currentSortedDistances.erase(currentSortedDistances.begin());
		}

		d_currentDivision = buildDivisionFromGroups(groups);
		return d_currentDivision;
	}
}