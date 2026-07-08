#include "frenet/collision_checker.hpp"
#include "math/frenet_converter.hpp"

OrientedBox MakeEgoBox(const CartesianState& cs, const VehicleShape& shape, double margin) {
    OrientedBox box;
    box.cx = cs.x;
    box.cy = cs.y;
    box.heading = cs.yaw;
    box.half_length = shape.length / 2.0 + margin;
    box.half_width  = shape.width  / 2.0 + margin;
    return box;
}

OrientedBox MakeObstacleBox(const ObjectInfo& obj, double t, double margin) {
    OrientedBox box;
    // 등속 직선 예측 (heading 방향으로 speed만큼 이동, 가속도 정보 없음)
    box.cx = obj.x + obj.speed * std::cos(obj.heading) * t;
    box.cy = obj.y + obj.speed * std::sin(obj.heading) * t;
    box.heading = obj.heading;
    box.half_length = obj.length / 2.0 + margin;
    box.half_width  = obj.width  / 2.0 + margin;
    return box;
}

// =========================================================
// SAT(분리축 정리): 두 OBB의 4개 축(각 박스의 길이/폭 방향) 중 하나라도
// 두 박스를 분리시키면 겹치지 않음. 전부 분리 못 시키면 겹침.
// 축 L 위로의 박스 반경 = half_length*|axis_length·L| + half_width*|axis_width·L|
// =========================================================

namespace {

double ProjectedRadius(const OrientedBox& box, double lx, double ly) {
    const double al_x = std::cos(box.heading), al_y = std::sin(box.heading);   // 길이축
    const double aw_x = -std::sin(box.heading), aw_y = std::cos(box.heading);  // 폭축
    return box.half_length * std::abs(al_x * lx + al_y * ly)
         + box.half_width  * std::abs(aw_x * lx + aw_y * ly);
}

bool SeparatedAlongAxis(const OrientedBox& a, const OrientedBox& b, double lx, double ly) {
    const double center_dist = std::abs((b.cx - a.cx) * lx + (b.cy - a.cy) * ly);
    return center_dist > ProjectedRadius(a, lx, ly) + ProjectedRadius(b, lx, ly);
}

}  // namespace

bool CheckOBBOverlap(const OrientedBox& a, const OrientedBox& b) {
    // a의 길이축/폭축, b의 길이축/폭축 — 총 4개 후보 분리축
    const double axes[4][2] = {
        {std::cos(a.heading), std::sin(a.heading)},
        {-std::sin(a.heading), std::cos(a.heading)},
        {std::cos(b.heading), std::sin(b.heading)},
        {-std::sin(b.heading), std::cos(b.heading)},
    };

    for (const auto& axis : axes) {
        if (SeparatedAlongAxis(a, b, axis[0], axis[1])) {
            return false;  // 분리축 하나라도 찾으면 겹치지 않음
        }
    }
    return true;  // 4개 축 전부 분리 실패 -> 겹침
}

// =========================================================
// FilterByCollision
// =========================================================

void FilterByCollision(std::vector<FrenetPath>& combined,
                        const RefLine& ref,
                        const std::vector<ObjectInfo>& obstacles,
                        const VehicleShape& ego_shape,
                        const CollisionCheckConfig& cfg) {
    // 이 값은 나눗셈이 아니라 "정지 상태면 coast 연장 추정을 할 필요가 없다"는
    // 최적화 판단에만 쓰인다 (아래 reactive-lookahead 섹션). 곡률/충돌 본 검사
    // 자체는 더 이상 이 임계값에 의존하지 않는다 (ComputeGeometricPath 참고).
    constexpr double kMinSpeedForCheck = 0.1;  // [m/s]

    for (auto& path : combined) {
        if (!path.valid) continue;

        bool collided = false;

        // FilterByCurvature와 동일한 이유로, 저속(s_dot≈0)에서 발산하는
        // TimeDerivToArcDeriv 기반 계산 대신 위치(x,y) 기반 기하학적 yaw를
        // 쓴다 (ego box 방향에는 yaw만 필요, kappa/v/a는 안 씀).
        GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);

        for (size_t i = 0; i < path.t.size() && !collided; i++) {
            CartesianState cs{};
            cs.x = geo.x[i];
            cs.y = geo.y[i];
            cs.yaw = geo.yaw[i];

            // Sec.VI: 마진은 "자차 크기"에만 더해짐 (장애물 쪽은 원래 크기 그대로).
            // 시간이 지날수록(t가 클수록) 마진이 커져 장애물이 "뒤로 물러나는" 효과.
            const double margin = cfg.safety_margin + cfg.margin_growth_rate * path.t[i];
            OrientedBox ego_box = MakeEgoBox(cs, ego_shape, margin);

            for (const auto& obj : obstacles) {
                OrientedBox obs_box = MakeObstacleBox(obj, path.t[i], 0.0);
                if (CheckOBBOverlap(ego_box, obs_box)) {
                    collided = true;
                    break;
                }
            }
        }

        // [Sec.VI/Fig.6] reactive layer 최소 lookahead 보강: 후보 자신의 T가
        // cfg.reactive_lookahead보다 짧으면, 그 이후는 후보의 마지막 상태
        // (s_dot, d)를 그대로 유지("coast")한다고 가정해 lookahead까지 계속
        // 검사한다. path.valid가 이미 위 루프에서 false가 됐으면(저속 등)
        // 여기서 더 볼 필요 없음.
        //
        // 마진은 후보 자신의 마지막 시점(last_t) 값으로 고정한다 — margin_growth_rate는
        // 원래 후보 자신의 실제 구간(최대 5초)을 염두에 둔 값인데, 이걸 연장구간까지
        // 그대로 키우면 "이미 안전하게 다 피한 후보"까지 연장 시간이 길어질수록 마진이
        // 비현실적으로 커져서 가짜로 충돌 판정되는 버그가 생긴다 (오프라인 시뮬레이션으로
        // 확인: growth_rate>0일 때 reactive_lookahead를 키울수록 오히려 회피 후보까지
        // 전부 무효화됨). "이 이후로는 뭘 할지 모른다"는 가정에서 마진을 계속 키우는 건
        // 근거가 없으므로, 후보가 실제로 관측된 마지막 마진 값을 그대로 유지한다.
        if (path.valid && !collided && !path.t.empty()) {
            const double dt = (path.t.size() >= 2) ? (path.t[1] - path.t[0]) : 0.1;
            const double last_t     = path.t.back();
            const double last_s     = path.s.back();
            const double last_s_dot = path.s_d.back();
            const double last_d     = path.d.back();
            const double coast_margin = cfg.safety_margin + cfg.margin_growth_rate * last_t;

            if (std::abs(last_s_dot) >= kMinSpeedForCheck) {
                for (double t = last_t + dt; t <= cfg.reactive_lookahead + 1e-9 && !collided; t += dt) {
                    const double s_coast = last_s + last_s_dot * (t - last_t);

                    RefPoint rp = Interpolate(ref, s_coast);
                    // 등속 직진 가정이므로 s_ddot=0, d_dot=0, d_ddot=0 -> d_prime=d_pprime=0
                    CartesianState cs = FrenetToCartesian(rp, s_coast, last_s_dot, 0.0,
                                                           last_d, 0.0, 0.0);

                    OrientedBox ego_box = MakeEgoBox(cs, ego_shape, coast_margin);

                    for (const auto& obj : obstacles) {
                        OrientedBox obs_box = MakeObstacleBox(obj, t, 0.0);
                        if (CheckOBBOverlap(ego_box, obs_box)) {
                            collided = true;
                            break;
                        }
                    }
                }
            }
        }

        if (collided) {
            path.valid = false;
        }
    }
}
