#pragma once
#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
namespace behavior_planner { class BehaviorScorer { public: BehaviorScore score(const UrbanFeature&,const HardConstraint&)const; }; }
