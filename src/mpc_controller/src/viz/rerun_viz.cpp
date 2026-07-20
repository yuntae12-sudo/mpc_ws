#include "viz/rerun_viz.hpp"

#ifdef ENABLE_RERUN_VIZ

#include <cmath>
#include <cstdio>

#include <rerun.hpp>

#include "frenet_planner/math/frenet_converter.hpp"

namespace rerun_viz {

namespace {

// 일부러 raw pointer로 두고 절대 delete하지 않는다: RecordingStream을 static
// duration으로 두면 프로그램 종료 시 정적 소멸자 순서 문제로 rerun 내부 Rust
// 백그라운드 스레드가 "use of std::thread::current() ... thread's local data
// has been destroyed" 패닉을 내며 core dump하는 걸 실측으로 확인했다(WSL2에서
// main()이 return으로 정상 종료하는 경로에서 재현). 어차피 프로세스가 끝나면
// OS가 전부 정리하므로, 디버그용 시각화 스트림은 굳이 정리하지 않고 그냥 둔다.
rerun::RecordingStream* g_rec = nullptr;

// world(x,y) -> ego 기준 로컬 좌표(ego가 원점, +x가 진행방향).
rerun::datatypes::Vec2D ToEgoLocal(double x, double y, const CartesianState& ego) {
    const double dx = x - ego.x;
    const double dy = y - ego.y;
    const double c = std::cos(ego.yaw);
    const double s = std::sin(ego.yaw);
    const float lx = static_cast<float>(dx * c + dy * s);
    const float ly = static_cast<float>(-dx * s + dy * c);
    return rerun::datatypes::Vec2D(lx, ly);
}

}  // namespace

void Init() {
    g_rec = new rerun::RecordingStream("mpc_controller");
    // WSL2(WSLg)에서는 spawn()이 띄우는 네이티브 뷰어가 GPU 텍스처 포맷 미지원으로
    // 크래시한다(wgpu: "does not support drawing to texture format R32Float").
    // 대신 미리 `rerun --serve-web`으로 띄워둔 서버(TCP 9876)에 연결만 하고,
    // 사람은 브라우저로 http://localhost:9090 에 접속해서 본다.
    auto err = g_rec->connect_tcp();
    if (err.is_err()) {
        std::fprintf(stderr,
                     "[rerun_viz] connect_tcp 실패: %s\n"
                     "  먼저 터미널에서 `rerun --serve-web` 을 실행해두세요.\n",
                     err.description.c_str());
    }
}

void LogCycle(const CartesianState& ego,
              const std::vector<FrenetPath>& candidates,
              const FrenetPath* best,
              const RefLine& ref) {
    if (!g_rec) return;

    // ego 마커: 원점 + 진행방향(+x) 짧은 화살표선.
    const std::vector<rerun::components::Position2D> ego_pt = {
        rerun::components::Position2D(0.0f, 0.0f)};
    g_rec->log("world/ego", rerun::Points2D(ego_pt)
                                 .with_colors(rerun::Color(255, 255, 255))
                                 .with_radii(0.3f));

    const std::vector<rerun::datatypes::Vec2D> heading_pts = {
        rerun::datatypes::Vec2D(0.0f, 0.0f), rerun::datatypes::Vec2D(2.0f, 0.0f)};
    const std::vector<rerun::components::LineStrip2D> heading_strip = {
        rerun::components::LineStrip2D(heading_pts)};
    g_rec->log("world/ego_heading",
               rerun::LineStrips2D(heading_strip).with_colors(rerun::Color(255, 255, 255)));

    // 후보 경로 전체 (valid=회색, invalid=흐린 빨강).
    std::vector<rerun::components::LineStrip2D> strips;
    std::vector<rerun::Color> colors;
    strips.reserve(candidates.size());
    colors.reserve(candidates.size());

    for (const auto& cand : candidates) {
        if (&cand == best) continue;  // best는 아래서 따로, 더 굵게 그림
        CartesianPath cp = ConvertToCartesianPath(cand, ref);
        std::vector<rerun::datatypes::Vec2D> pts;
        pts.reserve(cp.x.size());
        for (size_t i = 0; i < cp.x.size(); ++i) {
            pts.push_back(ToEgoLocal(cp.x[i], cp.y[i], ego));
        }
        strips.emplace_back(std::move(pts));
        colors.push_back(cand.valid ? rerun::Color(120, 120, 200, 120)
                                     : rerun::Color(200, 60, 60, 60));
    }
    g_rec->log("world/candidates",
               rerun::LineStrips2D(strips).with_colors(colors).with_radii(0.03f));

    // 선택된 최적 경로 (밝은 녹색, 굵게).
    if (best) {
        CartesianPath cp = ConvertToCartesianPath(*best, ref);
        std::vector<rerun::datatypes::Vec2D> pts;
        pts.reserve(cp.x.size());
        for (size_t i = 0; i < cp.x.size(); ++i) {
            pts.push_back(ToEgoLocal(cp.x[i], cp.y[i], ego));
        }
        const std::vector<rerun::components::LineStrip2D> best_strip = {
            rerun::components::LineStrip2D(std::move(pts))};
        g_rec->log("world/best_path", rerun::LineStrips2D(best_strip)
                                           .with_colors(rerun::Color(0, 255, 0))
                                           .with_radii(0.08f));
    } else {
        g_rec->log("world/best_path", rerun::Clear::RECURSIVE);
    }
}

}  // namespace rerun_viz

#endif  // ENABLE_RERUN_VIZ
