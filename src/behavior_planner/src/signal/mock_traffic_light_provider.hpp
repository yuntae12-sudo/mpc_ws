#pragma once
#include <unordered_map>
#include "behavior_planner/signal/traffic_light_provider.hpp"
namespace behavior_planner { class MockTrafficLightProvider:public TrafficLightProvider { public: bool load(const std::string&); TrafficLightState getState(const std::string&)const override; private: TrafficLightState fallback_=TrafficLightState::GREEN;std::unordered_map<std::string,TrafficLightState> states_; }; }
