#pragma once
#include "behavior_planner/PlanFeedback.h"
#include "behavior_planner/decision/behavior_scorer.hpp"
#include "behavior_planner/rule/hard_rule_filter.hpp"
namespace behavior_planner { class BehaviorDecision { public: BehaviorContextData decide(const UrbanFeature&,const PlanFeedback&)const; private:HardRuleFilter rules_;BehaviorScorer scorer_; }; }
