#include "parameter_loader.hpp"
#include "global.hpp"
#include "utils.hpp"

#include <cstdio>

#include <yaml-cpp/yaml.h>

namespace {

// node["a"]["b"] 형태의 dotted key 조회, 없으면 default 유지
template <typename T>
void loadInto(const YAML::Node& root, const std::string& dotted_key, T& out) {
    YAML::Node node = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        size_t dot = dotted_key.find('/', start);
        std::string key = dotted_key.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
        if (!node[key]) return;
        node = node[key];
        if (dot == std::string::npos) break;
        start = dot + 1;
    }
    out = node.as<T>();
}

}  // namespace

// ========================================
// yaml 파일 → mpc_params (mpc_params.yaml 구조/키 이름과 일치)
// ========================================
void loadMPCParameters(const std::string& yaml_path)
{
    auto& p = mpc_params;

    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::printf("[MPC] Failed to load yaml '%s': %s (기본값으로 진행)\n", yaml_path.c_str(), e.what());
        return;
    }

    // 예측
    loadInto(root, "mpc/horizon",            p.horizon);
    loadInto(root, "mpc/dt",                 p.dt);
    loadInto(root, "mpc/max_iterations",     p.max_iterations);
    loadInto(root, "mpc/wheelbase",          p.wheelbase);
    loadInto(root, "control_frequency",      p.control_frequency);

    // 제약
    loadInto(root, "constraints/steering/max_angle", p.steering_max);
    loadInto(root, "constraints/steering/max_rate",  p.steering_rate_max);
    loadInto(root, "constraints/acceleration/max_forward", p.accel_max);
    loadInto(root, "constraints/acceleration/max_reverse", p.accel_min);
    loadInto(root, "constraints/velocity/min",       p.vel_min);
    loadInto(root, "constraints/velocity/max",       p.vel_max);

    // 비용 가중치
    loadInto(root, "cost/path_error",     p.weight_path_error);
    loadInto(root, "cost/heading_error",  p.weight_heading_error);
    loadInto(root, "cost/speed_error",    p.weight_speed_error);
    loadInto(root, "cost/control",        p.weight_control);
    loadInto(root, "cost/control_rate",       p.weight_control_rate);
    loadInto(root, "cost/control_rate_accel", p.weight_control_rate_accel);
    loadInto(root, "cost/terminal",       p.weight_terminal);

    // 솔버
    loadInto(root, "solver/lr_init",          p.lr_init);
    loadInto(root, "solver/lr_min",           p.lr_min);
    loadInto(root, "solver/convergence_eps",  p.convergence_eps);
    loadInto(root, "solver/line_search_steps",p.line_search_steps);

    std::printf("[MPC] Params: horizon=%d dt=%.2f max_iter=%d wheelbase=%.2f freq=%.1fHz\n",
                p.horizon, p.dt, p.max_iterations, p.wheelbase, p.control_frequency);
    std::printf("[MPC] Weights: path=%.2f head=%.2f speed=%.2f term=%.2f\n",
                p.weight_path_error, p.weight_heading_error, p.weight_speed_error,
                p.weight_terminal);
}
