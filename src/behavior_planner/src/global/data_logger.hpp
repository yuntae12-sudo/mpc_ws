#pragma once
#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
namespace behavior_planner { class DataLogger { public:void write(const UrbanFeature&,const BehaviorContextData&); }; }
