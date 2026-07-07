#pragma once
#include "behavior_planner/BehaviorContext.h"
#include "behavior_planner/UrbanFeatureDebug.h"
#include "behavior_planner/context/behavior_context.hpp"
#include "behavior_planner/feature/urban_feature.hpp"
namespace behavior_planner { class MsgConverter { public: static behavior_planner::BehaviorContext toMsg(const BehaviorContextData&); static UrbanFeatureDebug toDebugMsg(const UrbanFeature&); }; }
