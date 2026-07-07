#pragma once
#include "behavior_planner/feature/urban_feature.hpp"
#include "behavior_planner/map/route_manager.hpp"
namespace behavior_planner { class ObjectProjector { public: ObjectFeature project(const TrackedObject&,const EgoFeature&,const RouteManager&)const; }; }
