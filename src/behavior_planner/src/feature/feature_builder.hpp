#pragma once
#include "behavior_planner/feature/object_projector.hpp"
#include "behavior_planner/signal/traffic_light_provider.hpp"
namespace behavior_planner { class FeatureBuilder { public: UrbanFeature build(const EgoState&,const PerceptionObjects&,const RouteManager&,const TrafficLightProvider&); private:int frame_=0;int last_lead_vehicle_id_=-1;ObjectProjector projector_; }; }
