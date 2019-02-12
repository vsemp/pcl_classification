#pragma once

#include "ColorDistanceMap.h"

#include <unordered_set>

namespace Classifier
{
	struct NbaDivision {
		using Group = std::unordered_set<ColorDistanceMap::Person>;

		static constexpr int NumberOfTeams = 2;
		static constexpr int MinimumNumberOfReferees = 3;
		static constexpr int MinimumNumberOfPlayersInTeam = 5;
		static constexpr int TotalMinimumNumberOfPeople = 13;

		Group d_unknown;
		Group d_referees;
		std::array<Group, NumberOfTeams> d_players;
	};

	class GroupClustering
	{
	public:
		explicit GroupClustering(ColorDistanceMap colorMap);
		const ColorDistanceMap& getColorDistanceMap() const;
		const NbaDivision& getCurrentDivision() const;
		const NbaDivision& tryBuildMinimumValidDivision();

	private:
		NbaDivision d_currentDivision;
		const ColorDistanceMap d_colorMap;
	};
}
