// MRPT adapter: one JSON request line on stdin -> one JSON reply line on
// stdout. Two modes, selected by the first request:
//   {"op":"start_mapping"}                    — CMetricMapBuilderICP
//   {"op":"load_map","path":"map.simplemap"}  — Monte Carlo localization
// Scan angles are radians in base_link (0 = robot forward, CCW positive),
// ranges meters; odometry deltas are in the previous robot frame.
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
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
    const std::string needle = "\"" + std::string(key) + "\":\"";
    auto p = line.find(needle);
    if (p == std::string::npos) return "";
    p += needle.size();
    auto e = line.find('"', p);
    if (e == std::string::npos) return "";
    return line.substr(p, e - p);
}
static double field_num(const std::string& line, const char* key, double dflt)
{
    const std::string needle = "\"" + std::string(key) + "\":";
    auto p = line.find(needle);
    if (p == std::string::npos) return dflt;
    p += needle.size();
    try { return std::stod(line.substr(p)); }
    catch (...) { return dflt; }
}
static std::vector<double> nested_nums(
    const std::string& line, const char* obj, const char* key)
{
    auto o = line.find(std::string("\"") + obj + "\":");
    if (o == std::string::npos) return {};
    auto p = line.find(std::string("\"") + key + "\":[", o);
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

static void reply_ok(const std::string& extra = "")
{
    std::cout << "{\"ok\":true" << (extra.empty() ? "" : "," + extra) << "}\n"
              << std::flush;
}
static void reply_err(const std::string& msg)
{
    std::cout << "{\"ok\":false,\"error\":\"" << json_escape(msg) << "\"}\n"
              << std::flush;
}

// ---- state ----
static std::unique_ptr<mrpt::slam::CMetricMapBuilderICP> builder;  // mapping
static std::unique_ptr<mrpt::slam::CMonteCarloLocalization2D> mcl;
static mrpt::maps::COccupancyGridMap2D::Ptr grid;  // shared by MCL + map op
static bool mapping = false;
static mrpt::poses::CPose2D last_pose{0, 0, 0};
static const double LIDAR_YAW_OFFSET = 97.6 * M_PI / 180.0;  // matches panel

static mrpt::obs::CObservation2DRangeScan::Ptr scan_from(const std::string& line)
{
    auto angles = nested_nums(line, "scan", "angles");
    auto ranges = nested_nums(line, "scan", "ranges");
    double max_range = field_num(line, "range_max", 8.0);
    auto obs = mrpt::obs::CObservation2DRangeScan::Create();
    obs->sensorLabel = "lds";
    obs->aperture = 2.0 * M_PI;
    obs->maxRange = max_range;
    // LDS zero bearing points backward relative to base_link forward.
    obs->sensorPose = mrpt::poses::CPose3D(0, 0, 0, LIDAR_YAW_OFFSET, 0, 0);
    const size_t n = std::min(angles.size(), ranges.size());
    obs->resizeScan(n);
    for (size_t i = 0; i < n; i++)
    {
        double r = ranges[i];
        bool valid = std::isfinite(r) && r > 0.02 && r <= max_range;
        obs->setScanRange(i, valid ? static_cast<float>(r) : 0.f);
        obs->setScanRangeValidity(i, valid);
        obs->setScanAngle(i, static_cast<float>(angles[i]));
    }
    return obs;
}

// Load a CSimpleMap into a fresh occupancy grid at 5 cm resolution.
static void grid_from_simplemap(
    const mrpt::maps::CSimpleMap& sm, mrpt::maps::COccupancyGridMap2D::Ptr& out)
{
    auto g = mrpt::maps::COccupancyGridMap2D::Create();
    // Roborock LDS reaches ~8 m; a ±10 m window covers an apartment room and
    // corridor around the start point until a real pose graph exists.
    g->setSize(-10.f, 10.f, -10.f, 10.f, 0.05f);
    for (unsigned i = 0; i < sm.size(); i++)
    {
        const auto [posepdf, sf] = sm.get(i);
        mrpt::poses::CPose3D p;
        posepdf->getMean(p);
        for (const auto& o : *sf)
            g->insertObservationPtr(o, std::optional<const mrpt::poses::CPose3D>(p));
    }
    out = g;
}

static std::string grid_json(const mrpt::maps::COccupancyGridMap2D& g)
{
    std::string cells;
    cells.reserve(static_cast<size_t>(g.getSizeX()) * g.getSizeY() * 2);
    for (unsigned y = 0; y < g.getSizeY(); y++)
        for (unsigned x = 0; x < g.getSizeX(); x++)
        {
            float p = g.getCell(x, y);
            cells += (p >= 0) ? std::to_string(static_cast<int>(p * 100)) : "-1";
            cells += ',';
        }
    if (!cells.empty()) cells.pop_back();
    char buf[192];
    std::snprintf(
        buf, sizeof buf,
        "\"grid\":{\"x_min\":%.2f,\"y_min\":%.2f,\"res\":%.3f,\"w\":%u,\"h\":%u,\"cells\":\"%s\"}",
        g.getXMin(), g.getYMin(), g.getResolution(), g.getSizeX(), g.getSizeY(),
        cells.c_str());
    return buf;
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
    std::string line;
    mrpt::bayes::CParticleFilter pf;
    pf.m_options.PF_algorithm = mrpt::bayes::CParticleFilter::pfStandardProposal;
    pf.m_options.resamplingThreshold = 0.5;

    while (std::getline(std::cin, line))
    {
        const std::string op = field_str(line, "op");
        try
        {
            if (op == "ping")
            {
                reply_ok(
                    ",\"mode\":" +
                    std::string(mapping ? "\"mapping\"" : mcl ? "\"localize\"" : "\"idle\""));
            }
            else if (op == "start_mapping")
            {
                builder = std::make_unique<mrpt::slam::CMetricMapBuilderICP>();
                builder->ICP_options.matchAgainstTheGrid = true;
                builder->ICP_options.minICPgoodnessToAccept = 0.30;
                builder->ICP_options.mapInitializers.clear();
                builder->ICP_options.mapInitializers.push_back(
                    mrpt::maps::COccupancyGridMap2D::MapDefinition());
                builder->initialize();
                mapping = true;
                last_pose = {0, 0, 0};
                reply_ok(",\"mode\":\"mapping\"");
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
                reply_ok(",\"mode\":\"localize\"");
            }
            else if (op == "update")
            {
                auto obs = scan_from(line);
                auto d = nested_nums(line, "odom", "d");
                mrpt::poses::CPose2D delta =
                    (d.size() >= 3) ? mrpt::poses::CPose2D(d[0], d[1], d[2])
                                    : mrpt::poses::CPose2D(0, 0, 0);
                if (mapping && builder)
                {
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
                    acts.insert(move);
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
                if (!grid)
                {
                    reply_err("no map yet");
                    continue;
                }
                if (mapping && builder)
                {
                    mrpt::maps::CSimpleMap sm;
                    builder->getCurrentlyBuiltMap(sm);
                    grid_from_simplemap(sm, grid);
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
                else if (grid)
                {
                    // Localization mode: persist the grid as the map snapshot.
                    if (!grid->saveToTextFile(path))
                    {
                        reply_err("cannot write grid: " + path);
                        continue;
                    }
                }
                else
                {
                    reply_err("nothing to save");
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
