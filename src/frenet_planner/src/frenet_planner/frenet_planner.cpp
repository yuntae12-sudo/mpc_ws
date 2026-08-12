#include "frenet_planner/frenet_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <limits>

#include "global/utils.hpp"

namespace {

const HighwayMergeZone* HighwayZoneAt(const HighwayMergeConfig& cfg, int index) {
    if (index < 0 || static_cast<size_t>(index) >= cfg.zones.size()) return nullptr;
    return &cfg.zones[static_cast<size_t>(index)];
}

const HighwayMergeCheckpoint* HighwayCheckpointAt(
        const HighwayMergeConfig& cfg, int zone_index, int checkpoint_index) {
    const HighwayMergeZone* zone = HighwayZoneAt(cfg, zone_index);
    if (!zone || checkpoint_index < 0 ||
        static_cast<size_t>(checkpoint_index) >= zone->checkpoints.size()) return nullptr;
    return &zone->checkpoints[static_cast<size_t>(checkpoint_index)];
}

int FindHighwayMergeZone(const HighwayMergeConfig& cfg, double s) {
    if (!cfg.enabled) return -1;
    for (size_t i = 0; i < cfg.zones.size(); ++i) {
        const auto& zone = cfg.zones[i];
        if (s >= zone.start_s && s <= zone.completion_s)
            return static_cast<int>(i);
    }
    return -1;
}

// 횡가속도 한계에서 곡률별 연속 목표속도를 계산한다.
double VelocityFromCurvature(double k, const CurveSpeedConfig& cfg, double max_lateral_accel) {
    constexpr double kSafetyMargin = 0.8;         // 한계의 80%만 사용 (여유폭 확보)
    constexpr double kMinKappaForCalc = 1e-4;      // 0 나누기 방지 (거의 직선)
    const double v_curve = std::sqrt(max_lateral_accel * kSafetyMargin / std::max(k, kMinKappaForCalc));
    return clip(v_curve, 0.0, cfg.target_vel);
}

// 현재 속도의 제동거리와 기본 lookahead 중 큰 범위의 최대 곡률을 사용한다.
double LookaheadTargetSpeed(const RefLine& ref, double s_start, double ego_speed,
                             const CurveSpeedConfig& cfg, double max_longitudinal_accel,
                             double max_lateral_accel, double max_curvature) {
    constexpr double kComfortableDecelFactor = 0.5;  // 실제 평균 감속도는 max의 절반 정도로 가정
    constexpr double kReactionMargin = 5.0;          // [m] 추가 여유거리
    const double usable_decel = std::max(0.5, max_longitudinal_accel * kComfortableDecelFactor);
    const double v_floor = VelocityFromCurvature(max_curvature, cfg, max_lateral_accel);
    const double speed_drop = std::max(0.0, ego_speed * ego_speed - v_floor * v_floor);
    const double required_dist = speed_drop / (2.0 * usable_decel);
    const double lookahead_m = std::max(cfg.curve_lookahead_m, required_dist + kReactionMargin);

    // 폐루프 wrap-around: s_start+lookahead_m이 s_max를 넘으면(피니시라인
    // 근처) 원래 코드는 뒤가 안 나와 break만 되고 이음새 너머(실제로는 바로
    // 이어지는 트랙 시작 구간)의 곡률을 놓쳤다 - 그 구간에 급커브가 있으면
    // 감속 없이 그대로 진입하게 된다. s_max로 감아서 [s_start, s_max]와
    // [0, 넘친 만큼] 두 구간을 모두 본다.
    const double s_max = ref.points.back().s;
    double s_start_w = std::fmod(s_start, s_max);
    if (s_start_w < 0.0) s_start_w += s_max;
    const double s_end_w = s_start_w + lookahead_m;
    const double wrapped_end = s_end_w - s_max;  // s_end_w > s_max일 때만 유효

    double max_k = 0.0;
    for (const auto& p : ref.points) {
        const bool in_range = (s_end_w <= s_max)
            ? (p.s >= s_start_w && p.s <= s_end_w)
            : (p.s >= s_start_w || p.s <= wrapped_end);
        if (in_range) max_k = std::max(max_k, std::fabs(p.kappa));
    }
    return VelocityFromCurvature(max_k, cfg, max_lateral_accel);
}

const char* ModeName(BehaviorState mode) {
    switch (mode) {
        case LANE_KEEPING: return "LANE_KEEPING";
        case FOLLOWING: return "FOLLOWING";
        case AVOID: return "AVOID";
        case MERGE: return "MERGE";
        default: return "OTHER";
    }
}

const char* AvoidPhaseName(int phase) {
    switch (phase) {
        case 0: return "TRACKING";
        case 1: return "SHIFT";
        case 2: return "PASS";
        default: return "NONE";
    }
}

const char* MergePhaseName(int phase) {
    switch (phase) {
        case 0: return "APPROACH";
        case 1: return "WAIT";
        case 2: return "COMMIT";
        case 3: return "CROSS";
        case 4: return "CLEAR";
        default: return "NONE";
    }
}

void ExtendMergePathThroughConflict(CartesianPath& path, const RefLine& ref,
                                    double conflict_s, double completion_distance,
                                    double path_end_s, double target_speed, double dt) {
    if (path.x.empty()) return;
    const double speed = std::max(target_speed, 0.5);
    for (double s = std::max(conflict_s, path_end_s) + speed * dt;
         s <= conflict_s + completion_distance + 1e-6; s += speed * dt) {
        const RefPoint rp = Interpolate(ref, s);
        path.x.push_back(rp.x);
        path.y.push_back(rp.y);
        path.yaw.push_back(rp.theta);
        path.kappa.push_back(rp.kappa);
        path.v.push_back(speed);
        path.a.push_back(0.0);
    }
}

CartesianPath RemainingPath(const CartesianPath& path, double ego_x, double ego_y,
                            size_t& progress_index) {
    CartesianPath result;
    if (path.x.empty()) return result;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size(),
                                   path.kappa.size(), path.v.size(), path.a.size()});
    const size_t search_begin = std::min(progress_index, count - 1);
    size_t first = search_begin;
    double best_dist2 = std::numeric_limits<double>::infinity();
    for (size_t i = search_begin; i < count; ++i) {
        const double dx = path.x[i] - ego_x;
        const double dy = path.y[i] - ego_y;
        const double dist2 = dx * dx + dy * dy;
        if (dist2 < best_dist2) {
            best_dist2 = dist2;
            first = i;
        }
    }
    progress_index = first;
    result.x.assign(path.x.begin() + first, path.x.begin() + count);
    result.y.assign(path.y.begin() + first, path.y.begin() + count);
    result.yaw.assign(path.yaw.begin() + first, path.yaw.begin() + count);
    result.kappa.assign(path.kappa.begin() + first, path.kappa.begin() + count);
    result.v.assign(path.v.begin() + first, path.v.begin() + count);
    result.a.assign(path.a.begin() + first, path.a.begin() + count);
    return result;
}

bool CartesianTrajectoryClear(const CartesianPath& path,
                              const std::vector<ObjectInfo>& obstacles,
                              const VehicleShape& ego_shape,
                              const CollisionCheckConfig& collision_cfg,
                              double dt, int* blocking_id = nullptr) {
    if (blocking_id) *blocking_id = -1;
    const size_t count = std::min({path.x.size(), path.y.size(), path.yaw.size()});
    for (size_t i = 0; i < count; ++i) {
        CartesianState ego{};
        ego.x = path.x[i];
        ego.y = path.y[i];
        ego.yaw = path.yaw[i];
        const double t = static_cast<double>(i) * dt;
        const double margin = collision_cfg.safety_margin +
                              collision_cfg.margin_growth_rate * t;
        const OrientedBox ego_box = MakeEgoBox(ego, ego_shape, margin);
        for (const auto& obj : obstacles) {
            if (CheckOBBOverlap(ego_box, MakeObstacleBox(obj, t, margin))) {
                if (blocking_id) *blocking_id = obj.id;
                return false;
            }
        }
    }
    return true;
}

// 후보 경로의 최대 기하 곡률에서 a_lat=v^2*|kappa| 한계를
// 만족하는 속도를 계산한다. 제어/모델 오차 여유로 한계의 80%를 쓴다.
double CandidateCurvatureSpeedLimit(const FrenetPath& path, const RefLine& ref,
                                    double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    constexpr double kMinCurvature = 1e-4;
    const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
    double max_kappa = 0.0;
    for (double kappa : geo.kappa) max_kappa = std::max(max_kappa, std::fabs(kappa));
    if (max_kappa < kMinCurvature) return std::numeric_limits<double>::infinity();
    return std::sqrt(max_lateral_accel * kLateralAccelMargin / max_kappa);
}

// 재생성된 후보의 실제 샘플별 속도²×곡률을 검증한다. 단순히
// 기하 곡률만 작은 경로가 아니라, 해당 속도에서 실제로 횡가속도
// 한계를 만족하는 경로만 남긴다.
void FilterByCartesianLateralAcceleration(std::vector<FrenetPath>& candidates,
                                          const RefLine& ref,
                                          double max_lateral_accel) {
    constexpr double kLateralAccelMargin = 0.8;
    const double limit = max_lateral_accel * kLateralAccelMargin;
    for (auto& path : candidates) {
        if (!path.valid) continue;
        const GeometricPath geo = ComputeGeometricPath(path.s, path.d, ref);
        const size_t count = std::min(geo.kappa.size(), path.s_d.size());
        for (size_t i = 0; i < count; ++i) {
            const double lateral_accel = path.s_d[i] * path.s_d[i] * std::fabs(geo.kappa[i]);
            if (lateral_accel > limit) {
                path.valid = false;
                path.rejection_reason = RejectionReason::CURVATURE;
                break;
            }
        }
    }
}

}  // namespace

bool FrenetPlanner::Init(const std::string& yaml_path) {
    std::string waypoint_file;
    LoadParams(yaml_path, path_cfg_, limits_, cost_weights_, vehicle_shape_,
               collision_cfg_, curve_speed_cfg_, following_cfg_, avoid_cfg_,
               merge_cfg_, highway_merge_cfg_, visualization_cfg_,
               wheelbase_, lane_width_, waypoint_file);

    debug_writer_ = std::make_unique<PlannerDebugWriter>(visualization_cfg_);

    if (waypoint_file.empty() || !LoadReferenceLine(waypoint_file, ref_, limits_.max_curvature)) {
        std::printf("[FrenetPlanner] Reference line load failed (waypoint_file='%s')\n",
                    waypoint_file.c_str());
        return false;
    }
    if (merge_cfg_.conflict_s < 0.0) {
        std::printf("[FrenetPlanner] Roundabout MERGE disabled: set "
                    "planner.merge.conflict_s in params.yaml\n");
    } else {
        const RefPoint conflict = Interpolate(ref_, merge_cfg_.conflict_s);
        std::printf("[FrenetPlanner] Roundabout MERGE conflict: s=%.2f pos=(%.2f, %.2f)\n",
                    merge_cfg_.conflict_s, conflict.x, conflict.y);
    }
    if (!highway_merge_cfg_.enabled || highway_merge_cfg_.zones.empty()) {
        std::printf("[FrenetPlanner] Highway MERGE disabled: measure start/conflict/"
                    "completion_s, then set planner.highway_merge.enabled=true\n");
    } else {
        for (const auto& zone : highway_merge_cfg_.zones) {
            bool valid_zone = zone.start_s >= 0.0 && !zone.checkpoints.empty();
            double previous_clear_s = zone.start_s;
            for (const auto& checkpoint : zone.checkpoints) {
                valid_zone = valid_zone && checkpoint.conflict_s > previous_clear_s &&
                    checkpoint.clear_s > checkpoint.conflict_s;
                previous_clear_s = checkpoint.clear_s;
            }
            valid_zone = valid_zone && zone.completion_s >= previous_clear_s;
            if (!valid_zone) {
                std::printf("[FrenetPlanner] Highway MERGE zone '%s' invalid s ordering; "
                            "policy disabled\n", zone.name.c_str());
                highway_merge_cfg_.enabled = false;
                break;
            }
            std::printf("[FrenetPlanner] Highway MERGE '%s': start=%.2f complete=%.2f "
                        "checkpoints=%zu\n", zone.name.c_str(), zone.start_s,
                        zone.completion_s, zone.checkpoints.size());
            for (size_t i = 0; i < zone.checkpoints.size(); ++i) {
                const auto& checkpoint = zone.checkpoints[i];
                const RefPoint conflict = Interpolate(ref_, checkpoint.conflict_s);
                std::printf("  checkpoint[%zu]: conflict=%.2f clear=%.2f pos=(%.2f, %.2f)\n",
                            i, checkpoint.conflict_s, checkpoint.clear_s,
                            conflict.x, conflict.y);
            }
        }
    }
    ref_loaded_ = true;
    return true;
}

// 저속 fallback은 시간 대신 이동거리 기준 cosine smoothstep으로 d를 복귀시킨다.
// 양 끝의 d'가 0이므로 정지 근처에서도 급격한 횡이동과 곡률 spike가 생기지 않는다.
bool FrenetPlanner::PlanLowSpeedFallback(const CartesianState& ego, double s, double d,
                                          double target_d,
                                          CartesianPath& out_path, double duration_s) const {
    constexpr double kRecenterDistM = 15.0;   // [m] d를 0으로 되돌리는 데 쓰는 이동거리
    const int n = static_cast<int>(duration_s / path_cfg_.dt) + 1;
    out_path.x.resize(n); out_path.y.resize(n); out_path.yaw.resize(n);
    out_path.kappa.resize(n); out_path.v.resize(n); out_path.a.resize(n);

    double s_i = s;
    double v_prev = std::max(ego.v, kMinCreepSpeed);
    for (int i = 0; i < n; ++i) {
        const double t = i * path_cfg_.dt;
        const double v_i = clip(ego.v + kCrawlAccel * t, kMinCreepSpeed, kCrawlSpeedCap);
        if (i > 0) s_i += 0.5 * (v_i + v_prev) * path_cfg_.dt;
        v_prev = v_i;

        const double ds = s_i - s;
        const double frac = clip(ds / kRecenterDistM, 0.0, 1.0);
        // smoothstep(frac) = 0.5*(1+cos(pi*frac)): frac=0 -> d, frac=1 -> 0, 양끝 기울기 0.
        const double d_error = d - target_d;
        const double d_i      = target_d + d_error * 0.5 * (1.0 + std::cos(M_PI * frac));
        const double d_prime  = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI / kRecenterDistM) * std::sin(M_PI * frac);
        const double d_pprime = (ds >= kRecenterDistM) ? 0.0
                               : d_error * (-0.5 * M_PI * M_PI / (kRecenterDistM * kRecenterDistM))
                                     * std::cos(M_PI * frac);

        RefPoint rp = Interpolate(ref_, s_i);
        const double a_i = (v_i < kCrawlSpeedCap) ? kCrawlAccel : 0.0;
        CartesianState cs = FrenetToCartesian(rp, s_i, v_i, a_i, d_i, d_prime, d_pprime);

        out_path.x[i] = cs.x;
        out_path.y[i] = cs.y;
        out_path.yaw[i] = cs.yaw;
        out_path.kappa[i] = cs.kappa;
        out_path.v[i] = v_i;
        out_path.a[i] = a_i;
    }
    return true;
}

bool FrenetPlanner::Plan(const CartesianState& ego, const std::vector<ObjectInfo>& obstacles,
                          CartesianPath& out_path) {
    last_failure_reason_ = PlanFailureReason::NONE;
    if (!ref_loaded_) {
        last_failure_reason_ = PlanFailureReason::NOT_INITIALIZED;
        return false;
    }

    double s, s_dot, s_ddot, d, d_prime, d_pprime;
    CartesianToFrenet(ref_, ego, s, s_dot, s_ddot, d, d_prime, d_pprime,
                       has_last_s_ ? &last_s_ : nullptr);
    last_s_ = s;
    has_last_s_ = true;

    // ObjectInfo의 연속 heading으로 yaw-rate를 추정한다. MORAI 객체의 heading은
    // 회전교차로에서 계속 변하므로 이를 보존해야 접선 직선 예측의 누락을 피할 수 있다.
    std::unordered_map<int, double> object_yaw_rates;
    for (auto& item : object_tracks_) ++item.second.missing_cycles;
    for (const auto& obj : obstacles) {
        auto it = object_tracks_.find(obj.id);
        if (it == object_tracks_.end()) {
            object_tracks_[obj.id] = ObjectTrack{obj.heading, 0.0, 0};
        } else {
            const double measured = std::remainder(obj.heading - it->second.heading,
                                                    2.0 * M_PI) / 0.05;
            // UDP heading 양자화/순간 점프를 억제하고 차량 수준의 yaw-rate로 제한한다.
            const double bounded = clip(measured, -1.0, 1.0);
            it->second.yaw_rate = 0.8 * it->second.yaw_rate + 0.2 * bounded;
            it->second.heading = obj.heading;
            it->second.missing_cycles = 0;
        }
        object_yaw_rates[obj.id] = object_tracks_[obj.id].yaw_rate;
    }
    for (auto it = object_tracks_.begin(); it != object_tracks_.end();) {
        if (it->second.missing_cycles > 20) it = object_tracks_.erase(it);
        else ++it;
    }
    std::vector<ObjectInfo> predicted_obstacles = obstacles;
    for (auto& obj : predicted_obstacles) {
        const auto it = object_yaw_rates.find(obj.id);
        obj.yaw_rate = it == object_yaw_rates.end() ? 0.0 : it->second;
    }

    // 저속(정지 근처)에서는 quintic 후보/곡률필터 파이프라인 전체를 건너뛰고
    // fallback으로 직접 경로를 만든다 (클래스 헤더 주석 참고 - 나누기 없는 안전한
    // 경로). AVOID/장애물 회피는 이 fallback에서는 다루지 않는다 - 정지 근처
    // 속도로는 위험한 회피 기동이 필요할 상황이 원래 적고, 속도가 오르면 바로
    // 아래의 정상 파이프라인으로 자동 복귀한다.
    last_d_ = d;

    constexpr double kLowSpeedFallbackV = 0.5;  // [m/s]
    const HighwayMergeZone* context_highway_zone = HighwayZoneAt(
        highway_merge_cfg_, merge_.highway_zone_index);
    const HighwayMergeCheckpoint* context_highway_checkpoint = HighwayCheckpointAt(
        highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
    const double context_conflict_s = merge_.type == MergeType::HIGHWAY
        && context_highway_checkpoint
            ? context_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
    const double context_stop_buffer = merge_.type == MergeType::HIGHWAY
        ? highway_merge_cfg_.stop_buffer : merge_cfg_.stop_buffer;
    const double low_speed_conflict_gap = merge_cfg_.conflict_s - s;
    const bool near_roundabout = merge_cfg_.conflict_s >= 0.0 &&
        low_speed_conflict_gap >= -merge_cfg_.completion_distance &&
        low_speed_conflict_gap <= merge_cfg_.approach_distance;
    const bool near_highway_merge = FindHighwayMergeZone(highway_merge_cfg_, s) >= 0;
    const bool near_any_merge = near_roundabout || near_highway_merge || merge_.active;

    // COMMIT 이후에는 noisy localization 상태로 exact-arrival 다항식을 매 cycle
    // 재생성하지 않는다. 최초 COMMIT 때 conflict zone 이탈까지 OBB 검증한 시간
    // 궤적을 시간에 맞춰 잘라 그대로 추종한다. 실제 동적 충돌이 새로 예측될 때만
    // 정지선 전에는 WAIT로 취소하고, 정지선 이후에는 비상정지를 요청한다.
    if (merge_.active && merge_.gap_locked && merge_.phase == MergePhase::COMMIT) {
        if (s >= context_conflict_s) {
            const HighwayMergeCheckpoint* next_checkpoint =
                merge_.type == MergeType::HIGHWAY
                    ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                                          merge_.highway_conflict_index + 1)
                    : nullptr;
            if (next_checkpoint) {
                ++merge_.highway_conflict_index;
                merge_.gap_locked = false;
                merge_.commit_pending = false;
                merge_.gap_safe = false;
                merge_.safe_cycles = 0;
                merge_.phase = MergePhase::APPROACH;
                merge_.locked_path = CartesianPath{};
                merge_.locked_elapsed = 0.0;
                merge_.locked_path_index = 0;
                std::printf("[FrenetPlanner-MERGE] highway checkpoint advanced: "
                            "zone=%s index=%d next_conflict=%.2f s=%.2f\n",
                            context_highway_zone ? context_highway_zone->name.c_str() : "-",
                            merge_.highway_conflict_index,
                            next_checkpoint->conflict_s, s);
            } else {
                merge_.phase = MergePhase::CROSS;
                merge_.locked_path = CartesianPath{};
            }
        } else if (!merge_.locked_path.x.empty()) {
            CartesianPath remaining = RemainingPath(
                merge_.locked_path, ego.x, ego.y, merge_.locked_path_index);
            int blocking_id = -1;
            const bool clear = CartesianTrajectoryClear(
                remaining, predicted_obstacles, vehicle_shape_, collision_cfg_,
                path_cfg_.dt, &blocking_id);
            if (!clear) {
                if (s < context_conflict_s - context_stop_buffer) {
                    std::printf("[FrenetPlanner-MERGE] locked trajectory emergency recheck "
                                "blocked before yield: object_id=%d; returning to WAIT\n",
                                blocking_id);
                    merge_.gap_locked = false;
                    merge_.gap_safe = false;
                    merge_.safe_cycles = 0;
                    merge_.phase = MergePhase::WAIT;
                    merge_.locked_path = CartesianPath{};
                    merge_.locked_elapsed = 0.0;
                    merge_.locked_path_index = 0;
                } else {
                    std::printf("[FrenetPlanner-MERGE] EMERGENCY: locked trajectory blocked "
                                "after yield by object_id=%d\n", blocking_id);
                    last_failure_reason_ = PlanFailureReason::COLLISION_BLOCKED;
                    return false;
                }
            } else if (remaining.x.size() >= 2) {
                out_path = std::move(remaining);
                static int locked_log_tick = 0;
                if (++locked_log_tick % 5 == 0) {
                    std::printf("[FrenetPlanner-STATE] mode=MERGE phase=COMMIT_LOCKED "
                                "s=%.2f d=%.3f speed=%.2f progress=%zu remaining=%zu\n",
                                s, d, ego.v, merge_.locked_path_index, out_path.x.size());
                }
                return true;
            } else {
                std::printf("[FrenetPlanner-MERGE] locked trajectory exhausted before conflict "
                            "(s=%.2f); requesting safe stop\n", s);
                last_failure_reason_ = PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
                return false;
            }
        }
    }
    // 회전교차로에서는 저속이어도 아래의 MERGE FSM을 반드시 거친다. 여기서
    // 조기 반환하면 WAIT/COMMIT 잠금보다 먼저 매 cycle 전체 도로가 빌 때까지
    // 검사하게 되어 conflict point 앞에서 두 번째 정지와 데드락이 발생한다.
    if (std::fabs(ego.v) < kLowSpeedFallbackV && !near_any_merge) {
        static int low_speed_log_tick = 0;
        if (++low_speed_log_tick % 5 == 0) {
            std::printf("[FrenetPlanner-STATE] mode=LOW_SPEED_FALLBACK "
                        "s=%.2f d=%.3f speed=%.2f candidates=N/A\n",
                        s, d, ego.v);
        }
        // TRACKING 근거리에서 한 점짜리 v=0 경로를 무기한 보내던 분기는
        // ego.v==0 -> 같은 조기 return이라는 자기잠금을 만들었다. 정적 장애물을
        // 등록할 때 검증한 1차 회피 방향을 저속 경로의 목표로 사용해 재출발한다.
        // 정상 속도로 회복되면 기존 좌/우 전체 후보 비교가 방향을 다시 확정한다.
        const bool tracking_near = avoidance_.active &&
            avoidance_.phase == AvoidPhase::TRACKING &&
            avoidance_.obstacle_s - s <= avoid_cfg_.shift_start_distance;
        const bool hold_avoid_target = avoidance_.active &&
            (tracking_near || avoidance_.phase == AvoidPhase::SHIFT ||
             avoidance_.phase == AvoidPhase::PASS);
        const double fallback_target_d = hold_avoid_target ? avoidance_.target_d : 0.0;
        const bool fallback_ok = PlanLowSpeedFallback(
            ego, s, d, fallback_target_d, out_path, 2.0);
        return fallback_ok;
    }

    double d_dot, d_ddot;
    ArcDerivToTimeDeriv(s_dot, s_ddot, d_prime, d_pprime, d_dot, d_ddot);
    last_d_dot_ = d_dot;  // 클램프 전 원본 값 - 진단 목적(아래 kMaxDDot 클램프 폭 자체를 보려고)

    // 투영 오차가 후보의 초기 가속도 경계조건을 비현실적으로 키우지 않도록
    // 다항식 중간구간의 overshoot 여유를 포함해 한계의 80%로 제한한다.
    constexpr double kBoundaryMargin = 0.8;
    s_ddot = clip(s_ddot, -limits_.max_longitudinal_accel * kBoundaryMargin,
                  limits_.max_longitudinal_accel * kBoundaryMargin);
    d_ddot = clip(d_ddot, -limits_.max_lateral_accel * kBoundaryMargin,
                  limits_.max_lateral_accel * kBoundaryMargin);

    // 큰 heading 오차로 과대 추정될 수 있는 초기 횡속도도 제한한다.
    constexpr double kMaxDDot = 0.5;  // [m/s]
    d_dot = clip(d_dot, -kMaxDDot, kMaxDDot);

    FrenetState start{s, s_dot, s_ddot, d, d_dot, d_ddot};

    // Behavior Planner 연동 전의 정적 장애물 AVOID context. 장애물 ID와
    // 회피 방향을 최초 1회만 고정한다. TRACKING 동안은 LANE_KEEPING을
    // 유지하고, 실제 기동 거리에서 SHIFT->PASS를 AVOID 안에서 수행한다.
    // 장애물 통과 후 중앙 복귀는 중복된 RETURN 단계 대신 LANE_KEEPING이 담당한다.
    PlannerCommand cmd{};
    double avoid_offset = 0.0;
    double leader_s = 0.0, leader_speed = 0.0, leader_accel = 0.0;
    bool begin_avoidance = false;

    // 활성 context의 장애물 위치를 새 UDP 프레임으로 갱신한다. 일시적
    // packet dropout이 있어도 저장된 위치/길이로 회피를 계속한다.
    if (avoidance_.active) {
        for (const auto& obj : obstacles) {
            if (obj.id != avoidance_.obstacle_id) continue;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            avoidance_.obstacle_s = obj_s;
            avoidance_.obstacle_d = obj_d;
            avoidance_.obstacle_width = obj.width;
            avoidance_.obstacle_length = obj.length;
            break;
        }

        const double gap = avoidance_.obstacle_s - s;
        const double clear_distance = 0.5 * (vehicle_shape_.length + avoidance_.obstacle_length)
                                    + avoid_cfg_.pass_clearance;
        if (avoidance_.phase == AvoidPhase::TRACKING && gap <= avoid_cfg_.shift_start_distance) {
            // 이 사이클에서 좌/우 전체 후보를 비교한 뒤, 유효 경로가
            // 있는 방향만 SHIFT로 잠그다.
            begin_avoidance = true;
        } else if (avoidance_.phase == AvoidPhase::SHIFT &&
                   std::fabs(d - avoidance_.target_d) <= avoid_cfg_.lateral_tolerance) {
            avoidance_.phase = AvoidPhase::PASS;
            std::printf("[FrenetPlanner] avoid phase: SHIFT -> PASS (id=%d gap=%.2f)\n",
                        avoidance_.obstacle_id, gap);
        } else if ((avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS) &&
                   gap < -clear_distance) {
            // 자차 뒤가 장애물을 완전히 통과했으므로 AVOID를 즉시 종료한다.
            // 이 아래의 !avoidance_.active 탐색이 같은 사이클에서 다음 정적
            // 장애물을 TRACKING으로 등록하고, 중앙 복귀는 LANE_KEEPING이 수행한다.
            std::printf("[FrenetPlanner] avoid complete after PASS: id=%d gap=%.2f\n",
                        avoidance_.obstacle_id, gap);
            avoidance_ = AvoidanceContext{};
        }
    }

    if (!avoidance_.active) {
        StaticObstacleTarget target;
        // LANE_KEEPING 충돌 필터는 reactive_lookahead 시간까지 미리 본다.
        // 정적 장애물 추적 거리가 그보다 짧으면, 장애물을 TRACKING으로
        // 등록하기 전에 충돌 필터가 중앙 후보를 전부 제거 -> stop path ->
        // 등록 후 다시 가속하는 속도 급락/반등이 생긴다. 설정값은 최소
        // 탐지거리로 쓰고, 실제 등록 거리는 "최고속도×충돌 lookahead +
        // 차량 길이/마진"보다 항상 길게 잡는다.
        AvoidConfig acquisition_cfg = avoid_cfg_;
        const double collision_horizon_distance =
            curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
            + vehicle_shape_.length + collision_cfg_.safety_margin;
        acquisition_cfg.detection_distance =
            std::max(avoid_cfg_.detection_distance, collision_horizon_distance);
        if (FindStaticAvoidanceTarget(ref_, start, obstacles, lane_width_, vehicle_shape_,
                                      acquisition_cfg, target)) {
            avoidance_.active = true;
            avoidance_.obstacle_id = target.id;
            avoidance_.phase = AvoidPhase::TRACKING;
            avoidance_.obstacle_s = target.s;
            avoidance_.obstacle_d = target.d;
            avoidance_.obstacle_width = target.width;
            avoidance_.obstacle_length = target.length;
            // 저속/정지 상태에서도 영구 정지하지 않도록 selector가 확인한
            // 1차 방향을 보관한다. 정상 속도에서는 회피 시작 직전 양쪽의
            // 전체 후보를 비교해 더 정확한 target_d로 덮어쓴다.
            avoidance_.target_d = target.avoidance_offset;
            std::printf("[FrenetPlanner] static obstacle tracked: id=%d gap=%.2f "
                        "effective_detection=%.2f\n",
                        target.id, target.s - s, acquisition_cfg.detection_distance);
        }
    }

    // FOLLOWING 탐색거리도 충돌 lookahead보다 항상 길게 잡아,
    // leader를 모드 전환 전에 충돌물로만 먼저 보고 정지하는 현상을 막는다.
    FollowingConfig effective_following_cfg = following_cfg_;
    const double following_collision_horizon =
        curve_speed_cfg_.target_vel * collision_cfg_.reactive_lookahead
        + vehicle_shape_.length + collision_cfg_.safety_margin;
    effective_following_cfg.max_leader_search_s =
        std::max(following_cfg_.max_leader_search_s, following_collision_horizon);

    // 직전 leader ID를 유지해 탐색거리 경계/패킷 누락에서 모드가
    // FOLLOWING<->LANE_KEEPING으로 번복되지 않게 한다.
    if (following_.active) {
        bool id_seen = false;
        bool leader_valid = false;
        const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
        for (const auto& obj : obstacles) {
            if (obj.id != following_.leader_id) continue;
            id_seen = true;
            double obj_s, obj_d, obj_s_dot;
            ProjectObjectToFrenet(ref_, obj, obj_s, obj_d, obj_s_dot);
            double gap = obj_s - s;
            if (gap <= 0.0 && track_length > 0.0) gap += track_length;
            const bool same_direction = obj_s_dot > following_cfg_.min_leader_speed;
            const bool same_lane = std::fabs(obj_d) <= lane_width_ * 0.5;
            const bool in_range = gap > 0.0 &&
                gap <= effective_following_cfg.max_leader_search_s
                     + following_cfg_.exit_search_margin;
            if (same_direction && same_lane && in_range) {
                following_.leader_s = s + gap;
                following_.leader_d = obj_d;
                following_.leader_speed = obj_s_dot;
                following_.leader_accel = 0.0;
                following_.missing_cycles = 0;
                leader_valid = true;
            }
            break;
        }

        if (!leader_valid) {
            if (id_seen) {
                // 관측됐지만 차선/방향/범위가 바뀐 것은 dropout이 아니므로 즉시 해제.
                following_ = FollowingContext{};
            } else if (++following_.missing_cycles <= following_cfg_.dropout_grace_cycles) {
                // ObjectInfo 일시 누락 동안은 직전 속도로 leader s를 예측.
                constexpr double kPlannerCycleDt = 0.05;  // main.cpp 20Hz
                following_.leader_s += following_.leader_speed * kPlannerCycleDt;
            } else {
                following_ = FollowingContext{};
            }
        }
    }

    // 유지 중인 ID만 보는 것으로 끝내면 합류 직후 실제로 더 가까워진 본선 차량을
    // 놓칠 수 있다. 매 cycle 동일차선 최인접 차량도 찾되, 작은 투영 흔들림으로
    // ID가 왕복하지 않도록 거리 hysteresis를 만족할 때만 인계한다.
    LeaderTarget nearest_leader;
    const bool nearest_found = FindLeader(
        ref_, start, obstacles, lane_width_, effective_following_cfg, nearest_leader);
    if (following_.active && nearest_found &&
        nearest_leader.id != following_.leader_id) {
        const double current_gap = following_.leader_s - s;
        const double nearest_gap = nearest_leader.s - s;
        if (nearest_gap + following_cfg_.leader_switch_margin < current_gap) {
            const int old_id = following_.leader_id;
            following_.leader_id = nearest_leader.id;
            following_.leader_s = nearest_leader.s;
            following_.leader_d = nearest_leader.d;
            following_.leader_speed = nearest_leader.speed;
            following_.leader_accel = nearest_leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader handoff: old=%d new=%d "
                        "old_gap=%.2f new_gap=%.2f\n",
                        old_id, nearest_leader.id, current_gap, nearest_gap);
        }
    }

    if (!following_.active && nearest_found) {
            const LeaderTarget& leader = nearest_leader;
            following_.active = true;
            following_.leader_id = leader.id;
            following_.leader_s = leader.s;
            following_.leader_d = leader.d;
            following_.leader_speed = leader.speed;
            following_.leader_accel = leader.accel;
            following_.missing_cycles = 0;
            std::printf("[FrenetPlanner] leader acquired: id=%d gap=%.2f speed=%.2f "
                        "effective_search=%.2f\n",
                        leader.id, leader.s - s, leader.speed,
                        effective_following_cfg.max_leader_search_s);
    }

    // 두 MERGE 모두 Global Path(d=0)를 그대로 따른다. 회전교차로는 conflict
    // point 도착시간, 고주로는 목표 본선의 완료시점 앞/뒤 gap+TTC를 판단한다.
    const double braking_distance = ego.v * ego.v /
        (2.0 * std::max(limits_.max_longitudinal_accel, 0.1));
    const double dynamic_roundabout_approach = std::max(
        merge_cfg_.approach_distance,
        ego.v * collision_cfg_.reactive_lookahead + braking_distance + 5.0);
    const double roundabout_conflict_gap = merge_cfg_.conflict_s - s;
    const bool roundabout_window = merge_cfg_.conflict_s >= 0.0 &&
        roundabout_conflict_gap >= -merge_cfg_.completion_distance &&
        roundabout_conflict_gap <= dynamic_roundabout_approach;
    const int detected_highway_zone = FindHighwayMergeZone(highway_merge_cfg_, s);
    const bool highway_window = detected_highway_zone >= 0;

    MergeType requested_merge_type = merge_.active ? merge_.type : MergeType::NONE;
    if (requested_merge_type == MergeType::NONE) {
        if (roundabout_window) requested_merge_type = MergeType::ROUNDABOUT;
        else if (highway_window) requested_merge_type = MergeType::HIGHWAY;
    }
    const bool in_merge_approach =
        (requested_merge_type == MergeType::ROUNDABOUT &&
         (roundabout_window || (merge_.active &&
          roundabout_conflict_gap >= -merge_cfg_.completion_distance))) ||
        (requested_merge_type == MergeType::HIGHWAY &&
         (merge_.active
              ? (context_highway_zone && s <= context_highway_zone->completion_s)
              : highway_window));
    if (in_merge_approach) {
        if (!merge_.active) {
            merge_ = MergeContext{};
            merge_.type = requested_merge_type;
            if (merge_.type == MergeType::HIGHWAY)
                merge_.highway_zone_index = detected_highway_zone;
            const HighwayMergeZone* activated_zone = HighwayZoneAt(
                highway_merge_cfg_, merge_.highway_zone_index);
            std::printf("[FrenetPlanner-MERGE] policy activated: %s zone=%s s=%.2f\n",
                        merge_.type == MergeType::HIGHWAY ? "HIGHWAY" : "ROUNDABOUT",
                        activated_zone ? activated_zone->name.c_str() : "-", s);
        }
        merge_.active = true;

        const bool highway = merge_.type == MergeType::HIGHWAY;
        const HighwayMergeZone* active_highway_zone = HighwayZoneAt(
            highway_merge_cfg_, merge_.highway_zone_index);
        const HighwayMergeCheckpoint* active_highway_checkpoint = HighwayCheckpointAt(
            highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
        if (highway && !active_highway_checkpoint) {
            std::printf("[FrenetPlanner-MERGE] active highway zone missing; reset\n");
            merge_ = MergeContext{};
            return false;
        }
        const double active_conflict_s = highway
            ? active_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
        const double conflict_gap = active_conflict_s - s;
        const double active_commit_distance = highway
            ? highway_merge_cfg_.commit_distance : merge_cfg_.commit_distance;
        const int active_confirm_cycles = highway
            ? highway_merge_cfg_.safe_confirm_cycles : merge_cfg_.safe_confirm_cycles;
        const double active_cross_speed = highway
            ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;

        if (merge_.gap_locked) {
            merge_.commit_pending = false;
            merge_.entry_time = std::max(0.0, merge_.entry_time - 0.05);
            if (merge_.preceding_time >= 0.0)
                merge_.preceding_time = std::max(0.0, merge_.preceding_time - 0.05);
            if (merge_.following_time >= 0.0)
                merge_.following_time = std::max(0.0, merge_.following_time - 0.05);
            merge_.gap_safe = true;
            // stop_s는 WAIT 정지 위치이지 CROSS 시작점이 아니다. 대표 충돌점에
            // 실제로 도달한 뒤부터 교차구역 통과 단계로 전환한다.
            if (s >= active_conflict_s) {
                const HighwayMergeCheckpoint* next_checkpoint = highway
                    ? HighwayCheckpointAt(highway_merge_cfg_, merge_.highway_zone_index,
                                          merge_.highway_conflict_index + 1)
                    : nullptr;
                if (next_checkpoint) {
                    ++merge_.highway_conflict_index;
                    merge_.gap_locked = false;
                    merge_.gap_safe = false;
                    merge_.safe_cycles = 0;
                    merge_.phase = MergePhase::APPROACH;
                    merge_.locked_path = CartesianPath{};
                    merge_.locked_elapsed = 0.0;
                    merge_.locked_path_index = 0;
                    merge_.preceding_id = -1;
                    merge_.following_id = -1;
                    merge_.preceding_time = -1.0;
                    merge_.following_time = -1.0;
                    std::printf("[FrenetPlanner-MERGE] highway checkpoint advanced: "
                                "zone=%s index=%d next_conflict=%.2f s=%.2f\n",
                                active_highway_zone ? active_highway_zone->name.c_str() : "-",
                                merge_.highway_conflict_index,
                                next_checkpoint->conflict_s, s);
                } else {
                    merge_.phase = MergePhase::CROSS;
                }
            }
        } else {
            const RoundaboutGap gap = highway
                ? FindHighwayMergeGap(
                    ref_, start, predicted_obstacles, highway_merge_cfg_,
                    *active_highway_checkpoint,
                    vehicle_shape_, collision_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel)
                : FindRoundaboutGap(
                    ref_, start, predicted_obstacles, merge_cfg_,
                    std::max(active_cross_speed, ego.v),
                    limits_.max_longitudinal_accel, &object_yaw_rates);
            const bool same_gap = merge_.preceding_id == gap.preceding_id &&
                                  merge_.following_id == gap.following_id;
            merge_.safe_cycles = gap.confirmed && gap.safe
                ? (same_gap ? merge_.safe_cycles + 1 : 1) : 0;
            merge_.gap_safe = gap.safe;
            merge_.entry_time = gap.entry_time;
            merge_.clear_time = gap.clear_time;
            merge_.preceding_id = gap.preceding_id;
            merge_.preceding_time = gap.preceding_time;
            merge_.following_id = gap.following_id;
            merge_.following_time = gap.following_time;
            merge_.crossing_vehicle_count = gap.crossing_vehicle_count;

            if (conflict_gap > active_commit_distance) {
                // APPROACH에서도 실제 gap 판정을 유지한다. unsafe이면 path_generator가
                // 제동거리 밖에서는 순항하고 제동구간부터 정지 경로로 전환한다.
                merge_.phase = MergePhase::APPROACH;
                // 예측창 밖(unconfirmed)은 진입 허가가 아니라 접근만 허용한다.
                merge_.gap_safe = !gap.confirmed || gap.safe;
                merge_.commit_pending = false;
            } else if (gap.confirmed && gap.safe &&
                       merge_.safe_cycles >= active_confirm_cycles) {
                // 아직 상태를 잠그지 않는다. 이 cycle의 실제 종/횡 결합 후보가
                // 곡률과 충돌검사를 통과한 뒤에만 아래에서 COMMIT으로 승격한다.
                merge_.phase = MergePhase::WAIT;
                merge_.commit_pending = true;
            } else {
                merge_.phase = MergePhase::WAIT;
                merge_.gap_safe = false;
                merge_.commit_pending = false;
            }
        }
    } else {
        merge_ = MergeContext{};
    }

    if (begin_avoidance || (avoidance_.active && avoidance_.phase != AvoidPhase::TRACKING)) {
        cmd.mode = AVOID;
        avoid_offset = (avoidance_.phase == AvoidPhase::SHIFT || avoidance_.phase == AvoidPhase::PASS)
                     ? avoidance_.target_d : 0.0;
        cmd.avoidance_d_offset = avoid_offset;
    } else if (merge_.active) {
        const bool highway = merge_.type == MergeType::HIGHWAY;
        const HighwayMergeZone* active_highway_zone = HighwayZoneAt(
            highway_merge_cfg_, merge_.highway_zone_index);
        const HighwayMergeCheckpoint* active_highway_checkpoint = HighwayCheckpointAt(
            highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
        const double active_conflict_s = highway
            && active_highway_checkpoint
                ? active_highway_checkpoint->conflict_s : merge_cfg_.conflict_s;
        const double active_stop_buffer = highway
            ? highway_merge_cfg_.stop_buffer : merge_cfg_.stop_buffer;
        cmd.mode = MERGE;
        cmd.target_lane = 0;
        cmd.merge_target_d = 0.0;
        cmd.merge_gap_safe = merge_.gap_safe;
        cmd.merge_conflict_s = active_conflict_s;
        cmd.merge_stop_s = active_conflict_s - active_stop_buffer;
        cmd.merge_entry_time = merge_.entry_time;
        cmd.merge_sa_id = merge_.preceding_id;
        cmd.merge_sb_id = merge_.following_id;
        cmd.merge_committed = merge_.gap_locked || merge_.commit_pending;
        cmd.merge_crossing = merge_.phase == MergePhase::CROSS;
        cmd.merge_type = merge_.type;
        // 합류 후 같은 corridor에 들어온 선행차는 일반 장애물이 아니라
        // FOLLOWING과 동일한 종방향 gap 제어 대상으로 전달한다.
        if (nearest_found) {
            cmd.leader_id = nearest_leader.id;
            cmd.leader_s = nearest_leader.s;
            cmd.leader_speed = nearest_leader.speed;
            cmd.leader_accel = nearest_leader.accel;
            cmd.time_gap = following_cfg_.time_gap;
            cmd.min_gap = following_cfg_.min_gap;
            cmd.gap_gain = following_cfg_.gap_gain;
        } else {
            cmd.leader_id = -1;
        }
    } else if (following_.active) {
        leader_s = following_.leader_s;
        leader_speed = following_.leader_speed;
        leader_accel = following_.leader_accel;
        cmd.mode = FOLLOWING;
        cmd.leader_id = following_.leader_id;
        cmd.leader_s = leader_s;
        cmd.leader_speed = leader_speed;
        cmd.leader_accel = leader_accel;
        cmd.time_gap = following_cfg_.time_gap;
        cmd.min_gap = following_cfg_.min_gap;
        cmd.gap_gain = following_cfg_.gap_gain;
    } else {
        cmd.mode = LANE_KEEPING;
    }

    cmd.target_speed = LookaheadTargetSpeed(ref_, s, ego.v, curve_speed_cfg_,
                                             limits_.max_longitudinal_accel,
                                             limits_.max_lateral_accel, limits_.max_curvature);
    if (cmd.mode == MERGE && merge_.phase == MergePhase::CROSS) {
        const double floor = merge_.type == MergeType::HIGHWAY
            ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;
        cmd.target_speed = std::max(cmd.target_speed, floor);
    }

    PlannerDebugStats stats;
    // TRACKING은 아직 회피를 시작하지 않는 구간이다. 8초 충돌 lookahead가
    // 먼 정적 장애물까지 미리 중앙 경로를 전부 제거하면 stop/go가 발생하므로,
    // 잠금된 해당 장애물만 이 구간의 충돌 필터에서 제외한다. 다른
    // 장애물은 계속 필터링되고, 25m 진입 시 잠금 장애물도 즉시 다시 포함된다.
    std::vector<ObjectInfo> planning_obstacles = predicted_obstacles;
    if (avoidance_.active && avoidance_.phase == AvoidPhase::TRACKING && !begin_avoidance) {
        planning_obstacles.erase(
            std::remove_if(planning_obstacles.begin(), planning_obstacles.end(),
                           [this](const ObjectInfo& obj) {
                               return obj.id == avoidance_.obstacle_id;
                           }),
            planning_obstacles.end());
    }

    std::vector<FrenetPath> candidates;
    // AVOID의 고정 속도 대신 후보 곡률로 속도를 자동 결정한다.
    // 1차 일반 목표속도 생성 -> 최적 후보 곡률의 허용속도 계산 ->
    // 필요할 때만 낮은 목표속도로 2차 재생성 순서다.
    auto generate_adaptive_avoid = [&](PlannerCommand& avoid_cmd, double target_d,
                                       PlannerDebugStats& avoid_stats) {
        auto generate_once = [&](PlannerDebugStats& generated_stats) {
            generated_stats = PlannerDebugStats{};
            auto generated = ResolveManeuver(start, avoid_cmd, ref_, path_cfg_, limits_,
                                             lane_width_, planning_obstacles, vehicle_shape_,
                                             collision_cfg_, &generated_stats);
            EvaluateCosts(generated, cost_weights_, target_d);
            return generated;
        };

        std::vector<FrenetPath> generated = generate_once(avoid_stats);
        const FrenetPath* initial_best = SelectBestPath(generated);
        if (initial_best) {
            const double speed_limit = CandidateCurvatureSpeedLimit(
                *initial_best, ref_, limits_.max_lateral_accel);
            if (std::isfinite(speed_limit) && speed_limit + 0.1 < avoid_cmd.target_speed) {
                avoid_cmd.target_speed = speed_limit;
                generated = generate_once(avoid_stats);
            }
        }

        FilterByCartesianLateralAcceleration(generated, ref_, limits_.max_lateral_accel);
        // 추가 횡가속도 필터를 반영해 시각화 통계를 재계산한다.
        avoid_stats.combined_valid_after_curvature = 0;
        avoid_stats.combined_valid_after_collision = 0;
        for (const auto& path : generated) {
            if (path.valid || path.rejection_reason == RejectionReason::COLLISION) {
                ++avoid_stats.combined_valid_after_curvature;
            }
            if (path.valid) ++avoid_stats.combined_valid_after_collision;
        }
        return generated;
    };

    if (begin_avoidance) {
        // 장애물 d에서 자차/장애물 반폭과 SAT 안전 마진을 더한 지점을
        // 양쪽 목표로 삼는다. 기존 ±3m 절대값보다 장애물의 실제
        // 횡위치를 반영하므로 불필요하게 멀리 이동하지 않는다.
        constexpr double kDirectionExtraMargin = 0.3;
        const double clearance = 0.5 * (vehicle_shape_.width + avoidance_.obstacle_width)
                               + collision_cfg_.safety_margin + kDirectionExtraMargin;
        const double left_target = avoidance_.obstacle_d + clearance;
        const double right_target = avoidance_.obstacle_d - clearance;

        auto generate_side = [&](double target_d, PlannerDebugStats& side_stats,
                                 double& used_target_speed) {
            PlannerCommand side_cmd = cmd;
            side_cmd.avoidance_d_offset = target_d;
            auto side = generate_adaptive_avoid(side_cmd, target_d, side_stats);
            used_target_speed = side_cmd.target_speed;
            return side;
        };

        PlannerDebugStats left_stats, right_stats;
        double left_target_speed = cmd.target_speed;
        double right_target_speed = cmd.target_speed;
        std::vector<FrenetPath> left_candidates =
            generate_side(left_target, left_stats, left_target_speed);
        std::vector<FrenetPath> right_candidates =
            generate_side(right_target, right_stats, right_target_speed);
        const FrenetPath* left_best = SelectBestPath(left_candidates);
        const FrenetPath* right_best = SelectBestPath(right_candidates);

        bool choose_right = false;
        bool direction_found = left_best || right_best;
        if (!left_best && right_best) {
            choose_right = true;
        } else if (left_best && right_best) {
            const double tie_margin = std::max(1.0, std::fabs(left_best->cost_total)) * 0.05;
            choose_right = right_best->cost_total < left_best->cost_total ||
                (avoid_cfg_.prefer_right_when_equal &&
                 right_best->cost_total <= left_best->cost_total + tie_margin);
        }

        if (direction_found) {
            avoidance_.target_d = choose_right ? right_target : left_target;
            avoidance_.phase = AvoidPhase::SHIFT;
            cmd.avoidance_d_offset = avoidance_.target_d;
            cmd.target_speed = choose_right ? right_target_speed : left_target_speed;
            avoid_offset = avoidance_.target_d;
            candidates = choose_right ? std::move(right_candidates) : std::move(left_candidates);
            stats = choose_right ? right_stats : left_stats;
            std::printf("[FrenetPlanner] avoid direction locked: %s id=%d gap=%.2f "
                        "target_d=%.2f target_speed=%.2f valid(left=%zu right=%zu)\n",
                        choose_right ? "RIGHT" : "LEFT", avoidance_.obstacle_id,
                        avoidance_.obstacle_s - s, avoidance_.target_d, cmd.target_speed,
                        left_stats.combined_valid_after_collision,
                        right_stats.combined_valid_after_collision);
        } else {
            // 양쪽 모두 실패한 경우 방향을 잠그지 않고 두 집합을 모두
            // 시각화한다. 상위의 no-valid 처리가 안전 정지를 보낸 뒤
            // 다음 사이클에서 좌/우를 다시 평가한다.
            candidates = std::move(left_candidates);
            candidates.insert(candidates.end(),
                              std::make_move_iterator(right_candidates.begin()),
                              std::make_move_iterator(right_candidates.end()));
            stats.lateral_total = left_stats.lateral_total + right_stats.lateral_total;
            stats.lateral_valid = left_stats.lateral_valid + right_stats.lateral_valid;
            stats.longitudinal_total = left_stats.longitudinal_total + right_stats.longitudinal_total;
            stats.longitudinal_valid = left_stats.longitudinal_valid + right_stats.longitudinal_valid;
            stats.combined_total = left_stats.combined_total + right_stats.combined_total;
            stats.combined_valid_after_curvature = left_stats.combined_valid_after_curvature
                                                 + right_stats.combined_valid_after_curvature;
            stats.combined_valid_after_collision = 0;
            std::printf("[FrenetPlanner] avoid direction unavailable: id=%d gap=%.2f "
                        "valid(left=0 right=0)\n",
                        avoidance_.obstacle_id, avoidance_.obstacle_s - s);
        }
    } else {
        if (cmd.mode == AVOID) {
            candidates = generate_adaptive_avoid(cmd, cmd.avoidance_d_offset, stats);
        } else {
            candidates = ResolveManeuver(start, cmd, ref_, path_cfg_, limits_, lane_width_,
                                         planning_obstacles, vehicle_shape_, collision_cfg_, &stats);
            // 회전교차로 MERGE도 횡방향으로는 global path(d=0)를 추종한다.
            EvaluateCosts(candidates, cost_weights_, 0.0);
        }
    }

    const double active_merge_conflict_gap = cmd.mode == MERGE
        ? cmd.merge_conflict_s - s : 0.0;
    const HighwayMergeCheckpoint* output_highway_checkpoint = HighwayCheckpointAt(
        highway_merge_cfg_, merge_.highway_zone_index, merge_.highway_conflict_index);
    const double active_merge_completion_distance =
        merge_.type == MergeType::HIGHWAY && output_highway_checkpoint
            ? output_highway_checkpoint->clear_s - output_highway_checkpoint->conflict_s
            : merge_cfg_.completion_distance;
    const double active_merge_cross_speed = merge_.type == MergeType::HIGHWAY
        ? highway_merge_cfg_.cross_speed_floor : merge_cfg_.cross_speed_floor;

    if (cmd.mode != prev_mode_) {
        std::printf("[FrenetPlanner] mode: %s -> %s (s=%.2f d=%.3f)\n",
                    ModeName(prev_mode_), ModeName(cmd.mode), s, d);
        if (cmd.mode == AVOID) {
            std::printf("[FrenetPlanner]   avoid_id=%d phase=%s target_d=%.3f\n",
                        avoidance_.obstacle_id,
                        AvoidPhaseName(static_cast<int>(avoidance_.phase)), avoid_offset);
        } else if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            std::printf("[FrenetPlanner]   leader_id=%d leader_s=%.2f leader_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f\n",
                        following_.leader_id, leader_s, leader_speed, gap, desired_gap);
        } else if (cmd.mode == MERGE) {
            std::printf("[FrenetPlanner]   %s conflict_gap=%.2f safe=%d "
                        "entry_time=%.2f preceding_id=%d following_id=%d crossing=%zu\n",
                        merge_.type == MergeType::HIGHWAY ? "highway" : "roundabout",
                        active_merge_conflict_gap, merge_.gap_safe ? 1 : 0,
                        merge_.entry_time, merge_.preceding_id, merge_.following_id,
                        merge_.crossing_vehicle_count);
        }
        prev_mode_ = cmd.mode;
    }
    if (cmd.mode == MERGE && merge_.phase != prev_merge_phase_) {
        std::printf("[FrenetPlanner-MERGE] phase: %s -> %s s=%.2f gap=%.2f "
                    "safe=%d locked=%d front=%d rear=%d entry=%.2f\n",
                    MergePhaseName(static_cast<int>(prev_merge_phase_)),
                    MergePhaseName(static_cast<int>(merge_.phase)),
                    s, active_merge_conflict_gap, merge_.gap_safe ? 1 : 0,
                    merge_.gap_locked ? 1 : 0, merge_.preceding_id,
                    merge_.following_id, merge_.entry_time);
        prev_merge_phase_ = merge_.phase;
    } else if (cmd.mode != MERGE) {
        prev_merge_phase_ = MergePhase::CLEAR;
    }

    // gap selector의 point-arrival 판정만으로 COMMIT하지 않는다. 실제 후보를
    // conflict zone 이탈점까지 연장한 뒤, 동일 CTRV+OBB 검사로 전체 시간 궤적이
    // 안전한 후보만 남긴다. 이 검사를 통과한 경로 자체가 아래에서 잠긴다.
    if (cmd.mode == MERGE && merge_.commit_pending) {
        for (auto& candidate : candidates) {
            if (!candidate.valid) continue;
            CartesianPath full = ConvertToCartesianPath(candidate, ref_);
            ExtendMergePathThroughConflict(
                full, ref_, cmd.merge_conflict_s, active_merge_completion_distance,
                candidate.s.empty() ? cmd.merge_conflict_s : candidate.s.back(),
                std::max(cmd.target_speed, active_merge_cross_speed), path_cfg_.dt);
            int blocking_id = -1;
            if (!CartesianTrajectoryClear(full, predicted_obstacles, vehicle_shape_,
                                          collision_cfg_, path_cfg_.dt, &blocking_id)) {
                candidate.valid = false;
                candidate.rejection_reason = RejectionReason::COLLISION;
                candidate.collision_object_id = blocking_id;
            }
        }
        stats.combined_valid_after_collision = 0;
        for (const auto& candidate : candidates)
            if (candidate.valid) ++stats.combined_valid_after_collision;
    }

    const FrenetPath* best = SelectBestPath(candidates);
    const int selected_index = best
        ? static_cast<int>(best - candidates.data())
        : -1;

    if (debug_writer_) {
        debug_writer_->Publish(ref_, ego, start, cmd.mode,
                               cmd.mode == AVOID
                                   ? AvoidPhaseName(static_cast<int>(avoidance_.phase))
                                   : (cmd.mode == MERGE
                                      ? MergePhaseName(static_cast<int>(merge_.phase)) : "NONE"),
                               cmd.target_speed,
                               obstacles, candidates,
                               selected_index, stats, cmd);
    }

    if (!best) {
        // Gap 계산만 통과하고 실제 ego 궤적이 충돌 검사를 실패했다면 COMMIT은
        // 성립하지 않는다. 정지선 통과 전에는 잠금을 즉시 풀고 WAIT로 되돌린다.
        if (cmd.mode == MERGE && merge_.commit_pending &&
            s < cmd.merge_stop_s) {
            std::printf("[FrenetPlanner-MERGE] commit trajectory rejected "
                        "(curvature=%zu collision=%zu); returning to WAIT (s=%.2f)\n",
                        stats.combined_valid_after_curvature,
                        stats.combined_valid_after_collision, s);
            merge_.gap_locked = false;
            merge_.commit_pending = false;
            merge_.gap_safe = false;
            merge_.safe_cycles = 0;
            merge_.phase = MergePhase::WAIT;
        }
        // 후보가 곡률 단계까지는 살아 있었는데 충돌 검사에서 전부 제거된 것은
        // 일시적인 생성 실패가 아니라 명시적인 안전 차단이다. main.cpp가 이
        // 값을 보고 과거 경로 재전송을 금지한다.
        last_failure_reason_ =
            stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0
                ? PlanFailureReason::COLLISION_BLOCKED
                : PlanFailureReason::TRANSIENT_GENERATION_FAILURE;
        std::printf("[FrenetPlanner] No valid candidate this cycle "
                    "(mode=%s, %zu generated)\n",
                    ModeName(cmd.mode), candidates.size());
        std::printf("[FrenetPlanner-DEBUG] ego: x=%.3f y=%.3f yaw=%.3f v=%.3f a=%.3f\n",
                     ego.x, ego.y, ego.yaw, ego.v, ego.a);
        std::printf("[FrenetPlanner-DEBUG] start: s=%.2f d=%.3f s_dot=%.2f s_ddot=%.2f d_dot=%.3f d_ddot=%.3f "
                     "target_speed=%.2f\n",
                     s, d, s_dot, s_ddot, d_dot, d_ddot, cmd.target_speed);
        std::printf("[FrenetPlanner-DEBUG] lateral %zu/%zu valid | longitudinal %zu/%zu valid | "
                     "combined %zu -> after curvature %zu -> after collision %zu valid\n",
                     stats.lateral_valid, stats.lateral_total,
                     stats.longitudinal_valid, stats.longitudinal_total,
                     stats.combined_total, stats.combined_valid_after_curvature,
                     stats.combined_valid_after_collision);
        if (stats.combined_valid_after_curvature > 0 &&
            stats.combined_valid_after_collision == 0) {
            std::unordered_map<int, int> blocker_counts;
            std::unordered_map<int, double> blocker_first_times;
            for (const auto& candidate : candidates) {
                if (candidate.rejection_reason != RejectionReason::COLLISION ||
                    candidate.collision_object_id < 0) continue;
                const int blocker_id = candidate.collision_object_id;
                ++blocker_counts[blocker_id];
                double collision_t = -1.0;
                if (candidate.collision_sample_index >= 0 &&
                    static_cast<size_t>(candidate.collision_sample_index) < candidate.t.size()) {
                    collision_t = candidate.t[static_cast<size_t>(candidate.collision_sample_index)];
                }
                auto time_it = blocker_first_times.find(blocker_id);
                if (collision_t >= 0.0 &&
                    (time_it == blocker_first_times.end() || collision_t < time_it->second))
                    blocker_first_times[blocker_id] = collision_t;
            }
            std::vector<std::pair<int, int>> ranked_blockers(
                blocker_counts.begin(), blocker_counts.end());
            std::sort(ranked_blockers.begin(), ranked_blockers.end(),
                      [](const auto& lhs, const auto& rhs) {
                          return lhs.second > rhs.second;
                      });
            const size_t report_count = std::min<size_t>(3, ranked_blockers.size());
            for (size_t rank = 0; rank < report_count; ++rank) {
                const int blocker_id = ranked_blockers[rank].first;
                const int blocked_paths = ranked_blockers[rank].second;
                const auto obj_it = std::find_if(
                    predicted_obstacles.begin(), predicted_obstacles.end(),
                    [blocker_id](const ObjectInfo& obj) { return obj.id == blocker_id; });
                if (obj_it == predicted_obstacles.end()) {
                    std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d "
                                "paths=%d object=missing leader_id=%d\n",
                                rank + 1, blocker_id, blocked_paths,
                                following_.active ? following_.leader_id : -1);
                    continue;
                }
                double obj_s, obj_d, obj_s_dot;
                ProjectObjectToFrenet(ref_, *obj_it, obj_s, obj_d, obj_s_dot);
                double obj_gap = obj_s - s;
                const double track_length = ref_.points.empty() ? 0.0 : ref_.points.back().s;
                if (obj_gap <= 0.0 && track_length > 0.0) obj_gap += track_length;
                const auto time_it = blocker_first_times.find(blocker_id);
                const double first_t = time_it == blocker_first_times.end()
                    ? -1.0 : time_it->second;
                std::printf("[FrenetPlanner-COLLISION] rank=%zu blocking_id=%d paths=%d "
                            "first_t=%.2f obj_s=%.2f obj_d=%.2f gap=%.2f speed_s=%.2f "
                            "same_lane=%d leader_id=%d leader_match=%d\n",
                            rank + 1, blocker_id, blocked_paths, first_t,
                            obj_s, obj_d, obj_gap, obj_s_dot,
                            std::fabs(obj_d) <= lane_width_ * 0.5 ? 1 : 0,
                            following_.active ? following_.leader_id : -1,
                            following_.active && following_.leader_id == blocker_id ? 1 : 0);
            }
        }
        return false;
    }

    CartesianPath selected_cartesian = ConvertToCartesianPath(*best, ref_);

    if (cmd.mode == MERGE && merge_.commit_pending) {
        ExtendMergePathThroughConflict(
            selected_cartesian, ref_, cmd.merge_conflict_s,
            active_merge_completion_distance,
            best->s.empty() ? cmd.merge_conflict_s : best->s.back(),
            std::max(cmd.target_speed, active_merge_cross_speed), path_cfg_.dt);
        merge_.commit_pending = false;
        merge_.gap_locked = true;
        merge_.phase = MergePhase::COMMIT;
        merge_.locked_path = selected_cartesian;
        merge_.locked_elapsed = 0.0;
        merge_.locked_path_index = 0;
        std::printf("[FrenetPlanner-MERGE] gap committed after trajectory validation: "
                    "front=%d(%.2fs) rear=%d(%.2fs) entry=%.2fs s=%.2f samples=%zu\n",
                    merge_.preceding_id, merge_.preceding_time,
                    merge_.following_id, merge_.following_time,
                    merge_.entry_time, s, merge_.locked_path.x.size());
    }

    // 모든 모드의 현재 상태를 20Hz 기준 약 4Hz로 출력한다. 모드 전환 로그만으로는
    // 후보가 사라지는 바로 그 순간이 LANE_KEEPING/AVOID 중 어느 상태였는지 알 수
    // 없으므로, s/d/속도와 필터 후 후보 수를 같은 줄에 남긴다.
    static int state_log_tick = 0;
    if (++state_log_tick % 5 == 0) {
        if (cmd.mode == FOLLOWING) {
            const double gap = leader_s - s;
            const double desired_gap = following_cfg_.min_gap
                                     + following_cfg_.time_gap * leader_speed;
            const double approach_speed = std::max(0.0, std::min(
                cmd.target_speed,
                leader_speed + following_cfg_.gap_gain * (gap - desired_gap)));
            std::printf("[FrenetPlanner-STATE] mode=FOLLOWING leader_id=%d s=%.2f d=%.3f "
                        "ego_speed=%.2f leader_speed=%.2f relative_speed=%.2f "
                        "gap=%.2f desired_gap=%.2f gap_error=%.2f approach_speed=%.2f chosen_d=%.3f "
                        "cost=%.2f candidates=%zu/%zu missing=%d\n",
                        following_.leader_id, s, d, ego.v, leader_speed,
                        ego.v - leader_speed, gap, desired_gap, gap - desired_gap,
                        approach_speed, best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total,
                        following_.missing_cycles);
        } else if (cmd.mode == MERGE) {
            std::printf("[FrenetPlanner-STATE] mode=MERGE phase=%s s=%.2f d=%.3f "
                        "speed=%.2f target_speed=%.2f conflict_gap=%.2f "
                        "entry=%.2f clear=%.2f front=%d(%.2f) rear=%d(%.2f) "
                        "locked=%d candidates=%zu/%zu\n",
                        MergePhaseName(static_cast<int>(merge_.phase)),
                        s, d, ego.v, cmd.target_speed, active_merge_conflict_gap,
                        merge_.entry_time, merge_.clear_time,
                        merge_.preceding_id, merge_.preceding_time,
                        merge_.following_id, merge_.following_time,
                        merge_.gap_locked ? 1 : 0,
                        stats.combined_valid_after_collision, stats.combined_total);
        } else {
            std::printf("[FrenetPlanner-STATE] mode=%s phase=%s s=%.2f d=%.3f speed=%.2f target_speed=%.2f "
                        "target_d=%.3f chosen_d=%.3f cost=%.2f candidates=%zu/%zu\n",
                        ModeName(cmd.mode), cmd.mode == AVOID
                            ? AvoidPhaseName(static_cast<int>(avoidance_.phase))
                            : (cmd.mode == MERGE
                               ? MergePhaseName(static_cast<int>(merge_.phase)) : "NONE"),
                        s, d, ego.v, cmd.target_speed,
                        cmd.mode == AVOID ? avoid_offset : 0.0,
                        best->d.empty() ? d : best->d.back(), best->cost_total,
                        stats.combined_valid_after_collision, stats.combined_total);
        }
    }

    out_path = std::move(selected_cartesian);
    return true;
}
