#pragma once
#include <string>
#include "behavior_planner/context/behavior_types.hpp"
namespace behavior_planner { class TrafficLightProvider { public: virtual ~TrafficLightProvider()=default; virtual TrafficLightState getState(const std::string&)const=0; }; }
