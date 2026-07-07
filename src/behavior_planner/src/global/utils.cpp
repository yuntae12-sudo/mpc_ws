#include "behavior_planner/global/utils.hpp"
#include <yaml-cpp/yaml.h>
namespace behavior_planner { std::vector<std::string> loadRouteLinks(const std::string&p){std::vector<std::string>v;try{auto y=YAML::LoadFile(p);if(y["route"]["links"])for(auto n:y["route"]["links"])v.push_back(n.as<std::string>());}catch(...){}return v;} }
