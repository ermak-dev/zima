// MRPT adapter: one JSON request line on stdin -> one JSON reply line on
// stdout. Two modes, selected by the first request:
//   {"op":"start_mapping"}                    — CMetricMapBuilderICP
//   {"op":"load_map","path":"map.simplemap"}  — Monte Carlo localization
// Scan angles are radians in base_link (0 = robot forward, CCW positive),
// ranges meters; odometry deltas are in the previous robot frame.
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/core/Clock.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <vector>

// ---- tiny JSON helpers: requests come from robot-server, only a few known
// fields are extracted, so a tolerant scanner is enough ----
static std::string json_escape(const std::string& s)
{
    std::string out;
    for (char c : s)
    {
        switch (c)
        {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            default: out += c;
        }
    }
    return out;
}
static std::string field_str(const std::string& line, const char* key)
{
    const std::string needle = "\"" + std::string(key) + "\"";
    auto p = line.find(needle);
    if (p == std::string::npos) return "";
    p = line.find(':', p + needle.size());
    if (p == std::string::npos) return "";
    p = line.find('"', p);
    if (p == std::string::npos) return "";
    auto e = line.find('"', p + 1);
    if (e == std::string::npos) return "";
    return line.substr(p + 1, e - p - 1);
}
static double field_num(const std::string& line, const char* key, double dflt)
{
    const std::string needle = "\"" + std::string(key) + "\"";
    auto p = line.find(needle);
    if (p == std::string::npos) return dflt;
    p = line.find(':', p + needle.size());
    if (p == std::string::npos) return dflt;
    try { return std::stod(line.substr(p + 1)); }
    catch (...) { return dflt; }
}
static std::vector<double> nested_nums(
    const std::string& line, const char* obj, const char* key)
{
    auto o = line.find(std::string("\"") + obj + "\"");
    if (o == std::string::npos) return {};
    auto p = line.find(std::string("\"") + key + "\"", o);
    if (p == std::string::npos) return {};
    p = line.find('[', p);
    auto e = line.find(']', p);
    if (e == std::string::npos) return {};
    std::vector<double> out;
    std::stringstream ss(line.substr(p + 1, e - p - 1));
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
        try { out.push_back(std::stod(tok)); }
        catch (...) { out.push_back(0.0); }
    }
    return out;
}

// Protocol replies go to the original stdout, saved at startup; fd 1 itself
// is redirected to stderr so no library (MRPT logs) can corrupt the channel.
static int proto_fd = 1;
static void send_line(const std::string& s)
{
    if (write(proto_fd, s.data(), s.size()) < 0) std::exit(0);
    if (write(proto_fd, "\n", 1) < 0) std::exit(0);
}

static void reply_ok(const std::string& extra = "")
{
    send_line("{\"ok\":true" + (extra.empty() ? "" : "," + extra) + "}");
}
static void reply_err(const std::string& msg)
{
    send_line("{\"ok\":false,\"error\":\"" + json_escape(msg) + "\"}");
}

// ---- state ----
static std::unique_ptr<mrpt::slam::CMetricMapBuilderICP> builder;  // mapping
static std::unique_ptr<mrpt::slam::CMonteCarloLocalization2D> mcl;
static mrpt::maps::COccupancyGridMap2D::Ptr grid;  // shared by MCL + map op
static bool mapping = false;
static mrpt::poses::CPose2D last_pose{0, 0, 0};
// Scan angles arrive already in base_link (robot-server applies the LDS
// mounting yaw); ranges are measured from the LDS at x = sensor_x.

// MRPT has no per-beam bearing: index i means -PI + i * 2PI/(SLOTS-1). Beams
// are binned into these fixed slots; empty slots stay invalid. 0.5° slots
// keep the quantization error under ~1.5 cm at 3 m.
static constexpr size_t SLOTS = 721;
// Grid insertion range: sparse far beams paint radial streaks, not walls.
static constexpr float MAX_INSERT_M = 5.0f;

static mrpt::obs::CObservation2DRangeScan::Ptr scan_from(const std::string& line)
{
    auto angles = nested_nums(line, "scan", "angles");
    auto ranges = nested_nums(line, "scan", "ranges");
    double max_range = field_num(line, "range_max", 8.0);
    auto obs = mrpt::obs::CObservation2DRangeScan::Create();
    obs->sensorLabel = "lds";
    obs->maxRange = max_range;
    // The ICP builder asserts on INVALID_TIMESTAMP; timestamps only need to
    // be increasing across scans.
    static mrpt::system::TTimeStamp t = mrpt::Clock::now();
    t = mrpt::system::timestampAdd(t, 0.05);
    obs->timestamp = t;
    obs->aperture = static_cast<float>(2.0 * M_PI);
    obs->rightToLeft = true;
    obs->sensorPose = mrpt::poses::CPose3D(field_num(line, "sensor_x", 0.0), 0, 0, 0, 0, 0);
    obs->resizeScan(SLOTS);
    for (size_t i = 0; i < SLOTS; i++)
    {
        obs->setScanRange(i, 0.f);
        obs->setScanRangeValidity(i, false);
    }
    const double step = 2.0 * M_PI / (SLOTS - 1);
    const size_t n = std::min(angles.size(), ranges.size());
    for (size_t k = 0; k < n; k++)
    {
        const double r = ranges[k], a = angles[k];
        if (!std::isfinite(r) || !std::isfinite(a) || r <= 0 || r > max_range) continue;
        const double wrapped = std::remainder(a, 2.0 * M_PI);  // [-PI, PI]
        const size_t i = std::min(SLOTS - 1, static_cast<size_t>(std::lround((wrapped + M_PI) / step)));
        // Two beams in one slot: keep the nearer obstacle.
        if (!obs->getScanRangeValidity(i) || r < obs->getScanRange(i))
        {
            obs->setScanRange(i, static_cast<float>(r));
            obs->setScanRangeValidity(i, true);
        }
    }
    return obs;
}

// Load a CSimpleMap into a fresh occupancy grid at 5 cm resolution.
static void grid_from_simplemap(
    const mrpt::maps::CSimpleMap& sm, mrpt::maps::COccupancyGridMap2D::Ptr& out)
{
    auto g = mrpt::maps::COccupancyGridMap2D::Create();
    // Initial window only; MRPT grows the grid as observations land outside.
    g->setSize(-10.f, 10.f, -10.f, 10.f, 0.05f);
    g->insertionOptions.maxDistanceInsertion = MAX_INSERT_M;
    // Empty angular slots are "no data", not "free up to the range limit".
    g->insertionOptions.considerInvalidRangesAsFreeSpace = false;
    for (unsigned i = 0; i < sm.size(); i++)
    {
        const auto& kf = sm.get(i);
        mrpt::poses::CPose3D p;
        kf.pose->getMean(p);
        for (const auto& o : *kf.sf)
            g->insertObservationPtr(o, std::optional<const mrpt::poses::CPose3D>(p));
    }
    out = g;
}

// MRPT cells hold P(free); never-observed cells stay exactly at 0.5.
static bool cell_unknown(float p_free) { return std::abs(p_free - 0.5f) < 1e-3f; }

// Protocol grid: occupancy percent (0 free .. 100 occupied, -1 unknown),
// rows in y-ascending order, cropped to the observed bounding box.
static std::string grid_json(const mrpt::maps::COccupancyGridMap2D& g)
{
    const unsigned W = g.getSizeX(), H = g.getSizeY();
    unsigned x0 = W, y0 = H, x1 = 0, y1 = 0;
    for (unsigned y = 0; y < H; y++)
        for (unsigned x = 0; x < W; x++)
            if (!cell_unknown(g.getCell(x, y)))
            {
                x0 = std::min(x0, x), x1 = std::max(x1, x);
                y0 = std::min(y0, y), y1 = std::max(y1, y);
            }
    const unsigned w = x0 <= x1 ? x1 - x0 + 1 : 0, h = y0 <= y1 ? y1 - y0 + 1 : 0;
    const float res = g.getResolution();
    // Assemble without snprintf for the cells: the payload can reach hundreds
    // of KB and a fixed buffer would silently truncate the JSON line.
    std::string out;
    out.reserve(static_cast<size_t>(w) * h * 3 + 128);
    char head[160];
    std::snprintf(
        head, sizeof head,
        "\"grid\":{\"x_min\":%.3f,\"y_min\":%.3f,\"res\":%.3f,\"w\":%u,\"h\":%u,\"cells\":\"",
        g.getXMin() + (w ? x0 : 0) * res, g.getYMin() + (h ? y0 : 0) * res, res, w, h);
    out += head;
    for (unsigned y = 0; y < h; y++)
        for (unsigned x = 0; x < w; x++)
        {
            const float p_free = g.getCell(x0 + x, y0 + y);
            out += cell_unknown(p_free) ? "-1"
                                        : std::to_string(static_cast<int>(std::lround((1.0f - p_free) * 100)));
            out += ',';
        }
    if (out.back() == ',') out.pop_back();
    out += "\"}";
    return out;
}

static std::string pose_json(const mrpt::poses::CPose2D& p, double q)
{
    char buf[128];
    std::snprintf(
        buf, sizeof buf,
        "\"pose\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.3f},\"quality\":%.2f",
        p.x(), p.y(), p.phi(), q);
    return buf;
}

// MCL spread (max deviation from the mean) as a coarse quality proxy.
static double mcl_spread(const mrpt::slam::CMonteCarloLocalization2D& pdf)
{
    mrpt::poses::CPose2D mean;
    pdf.getMean(mean);
    double worst = 0;
    auto [cov, _] = pdf.getCovarianceAndMean();
    double sd = std::sqrt(cov(0, 0) + cov(1, 1) + cov(2, 2));
    worst = sd;
    (void)mean;
    return worst;
}

int main()
{
    std::ios::sync_with_stdio(false);
    proto_fd = dup(1);
    if (proto_fd < 0) return 1;
    dup2(2, 1);  // every stdout write (incl. MRPT logs) lands on stderr
    std::string line;
    mrpt::bayes::CParticleFilter pf;
    pf.m_options.PF_algorithm = mrpt::bayes::CParticleFilter::pfStandardProposal;
    // BETA (ESS resampling threshold) keeps its default 0.5.

    while (std::getline(std::cin, line))
    {
        const std::string op = field_str(line, "op");
        try
        {
            if (op == "ping")
            {
                reply_ok(
                    "\"mode\":" +
                    std::string(mapping ? "\"mapping\"" : mcl ? "\"localize\"" : "\"idle\""));
            }
            else if (op == "start_mapping")
            {
                builder = std::make_unique<mrpt::slam::CMetricMapBuilderICP>();
                builder->ICP_options.matchAgainstTheGrid = false;
                builder->ICP_options.minICPgoodnessToAccept = 0.30;
                // Roborock pace: LDS scans every ~0.1-0.2 m at 0.1 m/s.
                builder->ICP_options.insertionLinDistance = 0.10;
                builder->ICP_options.insertionAngDistance = 10.0 * M_PI / 180.0;
                builder->ICP_options.localizationLinDistance = 0.05;
                builder->ICP_options.localizationAngDistance = 5.0 * M_PI / 180.0;
                if (getenv("MRPT_ADAPTER_DEBUG"))
                    builder->setVerbosityLevel(mrpt::system::LVL_DEBUG);
                builder->ICP_options.mapInitializers.clear();
                builder->ICP_options.mapInitializers.push_back(
                    mrpt::maps::CSimplePointsMap::MapDefinition());
                builder->initialize();
                mapping = true;
                last_pose = {0, 0, 0};
                reply_ok("\"mode\":\"mapping\"");
            }
            else if (op == "load_map")
            {
                const std::string path = field_str(line, "path");
                if (path.empty())
                {
                    reply_err("load_map needs a path");
                    continue;
                }
                mrpt::maps::CSimpleMap sm;
                if (!sm.loadFromFile(path))
                {
                    reply_err("cannot load simplemap: " + path);
                    continue;
                }
                grid_from_simplemap(sm, grid);
                mcl = std::make_unique<mrpt::slam::CMonteCarloLocalization2D>(300);
                mcl->options.metricMap =
                    std::static_pointer_cast<const mrpt::maps::CMetricMap>(grid);
                // Global relocalization over the whole map right away: the
                // robot may have been moved since the last session.
                mcl->resetUniformFreeSpace(grid.get(), 0.7f, -1);
                mapping = false;
                last_pose = {0, 0, 0};
                reply_ok("\"mode\":\"localize\"");
            }
            else if (op == "update")
            {
                auto obs = scan_from(line);
                auto dx = field_num(line, "dx", 0.0);
                auto dy = field_num(line, "dy", 0.0);
                auto dyaw = field_num(line, "dyaw", 0.0);
                mrpt::poses::CPose2D delta(dx, dy, dyaw);
                if (mapping && builder)
                {
                    // The ICP builder reads odometry as a CObservationOdometry
                    // carrying the ABSOLUTE accumulated pose; feed odometry
                    // first, then the scan, both via processObservation.
                    static mrpt::poses::CPose2D odom_abs(0, 0, 0);
                    odom_abs = odom_abs + delta;
                    auto odometry = mrpt::obs::CObservationOdometry::Create();
                    odometry->sensorLabel = "odometry";
                    odometry->timestamp = obs->timestamp;
                    odometry->odometry = odom_abs;
                    builder->processObservation(odometry);
                    builder->processObservation(obs);
                    auto pdf = builder->getCurrentPoseEstimation();
                    mrpt::poses::CPose3D p;
                    pdf->getMean(p);
                    last_pose = mrpt::poses::CPose2D(p.x(), p.y(), p.yaw());
                    reply_ok(pose_json(last_pose, 1.0));
                }
                else if (mcl)
                {
                    mrpt::obs::CActionCollection acts;
                    auto move = mrpt::obs::CActionRobotMovement2D::Create();
                    move->computeFromOdometry(
                        delta, mrpt::obs::CActionRobotMovement2D::TMotionModelOptions());
                    acts.insertPtr(move);
                    mrpt::obs::CSensoryFrame sf;
                    sf.insert(obs);
                    pf.executeOn(*mcl, &acts, &sf);
                    mrpt::poses::CPose2D mean;
                    mcl->getMean(mean);
                    last_pose = mean;
                    // 0.4 m of particle spread ≈ lost; 0.05 m ≈ locked in.
                    double spread = mcl_spread(*mcl);
                    double q = 1.0 - std::min(1.0, spread / 0.4);
                    reply_ok(pose_json(mean, q));
                }
                else
                {
                    reply_err("no mode selected: send start_mapping or load_map first");
                }
            }
            else if (op == "local_reset" || op == "global_reset")
            {
                if (!mcl)
                {
                    reply_err("not in localize mode");
                    continue;
                }
                if (op == "global_reset")
                {
                    mcl->resetUniformFreeSpace(grid.get(), 0.7f, -1);
                }
                else
                {
                    mcl->resetUniformFreeSpace(
                        grid.get(), 0.7f, -1,
                        last_pose.x() - 0.5f, last_pose.x() + 0.5f,
                        last_pose.y() - 0.5f, last_pose.y() + 0.5f, -M_PI, M_PI);
                }
                reply_ok();
            }
            else if (op == "map")
            {
                if (mapping && builder)
                {
                    // Rebuild the grid from the simplemap built so far; the
                    // grid object itself is created lazily here because
                    // mapping mode starts with none.
                    mrpt::maps::CSimpleMap sm;
                    builder->getCurrentlyBuiltMap(sm);
                    grid_from_simplemap(sm, grid);
                }
                if (!grid)
                {
                    reply_err("no map yet");
                    continue;
                }
                reply_ok(grid_json(*grid));
            }
            else if (op == "save_map")
            {
                const std::string path = field_str(line, "path");
                if (path.empty())
                {
                    reply_err("save_map needs a path");
                    continue;
                }
                if (mapping && builder)
                {
                    mrpt::maps::CSimpleMap sm;
                    builder->getCurrentlyBuiltMap(sm);
                    if (!sm.saveToFile(path))
                    {
                        reply_err("cannot write simplemap: " + path);
                        continue;
                    }
                }
                else
                {
                    // The map is immutable in localize mode; saving it again
                    // would only duplicate the source file's data.
                    reply_err("save_map is only available in mapping mode");
                    continue;
                }
                reply_ok();
            }
            else
            {
                reply_err("unknown op: " + op);
            }
        }
        catch (const std::exception& e)
        {
            reply_err(e.what());
        }
    }
    return 0;
}
