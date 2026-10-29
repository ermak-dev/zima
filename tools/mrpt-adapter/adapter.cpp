// One JSON request per line; protocol output is isolated from MRPT logging.
// ICP consumes explicit base_link XY rays, occupancy consumes sparse angular
// observations. Never run CMetricMapBuilderICP's gap-dropping points filter.
#include "json.h"
#include <mrpt/bayes/CParticleFilter.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/core/Clock.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/slam/CMonteCarloLocalization2D.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <iomanip>
#include <limits>
#include <unistd.h>

static std::string json_escape(const std::string& s)
{
    std::string out;
    for (unsigned char c : s)
    {
        if (c == '"') out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c < 32) { char b[7]; std::snprintf(b,sizeof b,"\\u%04x",c); out += b; }
        else out += char(c);
    }
    return out;
}
static double number(const Json& j, const char* key, double fallback)
{
    auto it=j.object.find(key);
    return it==j.object.end() ? fallback : it->second.numeric();
}
static std::string text(const Json& j, const char* key)
{
    const auto& v=j.get(key);
    if (v.type != Json::String) throw std::runtime_error(std::string("expected string: ")+key);
    if (v.text.find('\0') != std::string::npos) throw std::runtime_error("NUL in string");
    return v.text;
}
static std::string num(double v)
{
    if (!std::isfinite(v)) return "null";
    std::ostringstream s; s << std::setprecision(12) << v; return s.str();
}
static int proto_fd=1;
static void send_line(const std::string& s)
{
    std::string packet=s+'\n';
    size_t sent=0;
    while (sent<packet.size())
    {
        const auto n=write(proto_fd,packet.data()+sent,packet.size()-sent);
        if (n<0 && errno==EINTR) continue;
        if (n<=0) std::exit(0);
        sent+=static_cast<size_t>(n);
    }
}
static void reply_ok(const std::string& extra="")
{ send_line("{\"ok\":true"+(extra.empty()?"":","+extra)+"}"); }
static void reply_err(const std::string& msg)
{ send_line("{\"ok\":false,\"error\":\""+json_escape(msg)+"\"}"); }

struct Config
{
    double min_goodness=.30, threshold=.75, insertion_linear=.10;
    double insertion_angular=10*M_PI/180, resolution=.05, max_range=5;
    double uncompensated_linear=.08, uncompensated_angular=8*M_PI/180;
    unsigned iterations=40, decimation=1;
    static double env(const char* name, double fallback, double lo, double hi)
    {
        const char* raw=std::getenv(name);
        if (!raw) return fallback;
        char* end=nullptr;
        double v=std::strtod(raw,&end);
        if (end==raw || *end || !std::isfinite(v) || v<lo || v>hi)
            throw std::runtime_error(std::string("invalid configuration: ")+name);
        return v;
    }
    static unsigned integer(const char* name, unsigned fallback, unsigned hi)
    {
        double v=env(name,fallback,1,hi);
        if (std::floor(v)!=v) throw std::runtime_error(std::string("expected integer: ")+name);
        return static_cast<unsigned>(v);
    }
    static Config load()
    {
        Config c;
        c.min_goodness=env("ROBOT_SLAM_ICP_MIN_GOODNESS",.30,0,1);
        c.threshold=env("ROBOT_SLAM_ICP_THRESHOLD_DIST",.75,.001,10);
        c.iterations=integer("ROBOT_SLAM_ICP_MAX_ITERATIONS",40,1000);
        c.decimation=integer("ROBOT_SLAM_ICP_DECIMATION",1,1000);
        c.insertion_linear=env("ROBOT_SLAM_INSERTION_LINEAR",.10,0,10);
        c.insertion_angular=env("ROBOT_SLAM_INSERTION_ANGULAR_DEG",10,0,180)*M_PI/180;
        c.resolution=env("ROBOT_SLAM_GRID_RESOLUTION",.05,.01,1);
        c.max_range=env("ROBOT_SLAM_MAX_INSERT_RANGE",5,.1,100);
        c.uncompensated_linear=env("ROBOT_SLAM_MAX_UNCOMPENSATED_TRANSLATION",.08,0,1);
        c.uncompensated_angular=env("ROBOT_SLAM_MAX_UNCOMPENSATED_ROTATION_DEG",8,0,180)*M_PI/180;
        return c;
    }
};
static Config config;
static std::unique_ptr<mrpt::slam::CMonteCarloLocalization2D> mcl;
static mrpt::maps::COccupancyGridMap2D::Ptr grid;
static mrpt::maps::CSimpleMap simplemap;
static mrpt::maps::CSimplePointsMap reference;
static bool mapping=false;
static mrpt::poses::CPose2D last_pose{0,0,0}, insertion_pose{0,0,0};
static constexpr size_t SLOTS=721;

struct Scan
{
    mrpt::obs::CObservation2DRangeScan::Ptr obs;
    mrpt::maps::CSimplePointsMap points;
    size_t input_points=0;
};
static Scan read_scan(const Json& req)
{
    const auto& s=req.get("scan");
    const auto& aa=s.get("angles"); const auto& rr=s.get("ranges");
    if (s.type!=Json::Object || aa.type!=Json::Array || rr.type!=Json::Array ||
        aa.array.empty() || aa.array.size()!=rr.array.size() || aa.array.size()>100000)
        throw std::runtime_error("scan needs equal nonempty angles/ranges arrays");
    const double max_range=number(req,"range_max",8), sensor_x=number(req,"sensor_x",0);
    if (max_range<=0 || max_range>1000 || std::abs(sensor_x)>10)
        throw std::runtime_error("invalid scan range or sensor offset");
    Scan out;
    out.input_points=aa.array.size();
    auto& obs=out.obs;
    obs=mrpt::obs::CObservation2DRangeScan::Create();
    obs->sensorLabel="lds";
    obs->maxRange=max_range;
    // A supplied stamp is source time, not UTC. Otherwise use observation
    // receipt time, without claiming a manufactured acquisition cadence.
    obs->timestamp=mrpt::Clock::now();
    if(req.object.count("stamp"))
    {
        // MRPT's fromDouble casts to uint64_t before adding its epoch offset;
        // an enormous finite value (or negative source time) must not overflow
        // that conversion. Construct the representable timestamp explicitly.
        const long double ticks=mrpt::Clock::fromDouble(0).time_since_epoch().count()+
            static_cast<long double>(req.get("stamp").numeric())*
            mrpt::Clock::period::den/mrpt::Clock::period::num;
        if(ticks<std::numeric_limits<mrpt::Clock::rep>::min() ||
           ticks>std::numeric_limits<mrpt::Clock::rep>::max() || ticks==0)
            throw std::runtime_error("stamp outside MRPT clock range");
        obs->timestamp=mrpt::Clock::time_point(mrpt::Clock::duration(static_cast<mrpt::Clock::rep>(ticks)));
    }
    obs->aperture=static_cast<float>(2*M_PI);
    obs->rightToLeft=true;
    obs->sensorPose=mrpt::poses::CPose3D(sensor_x,0,0,0,0,0);
    obs->resizeScan(SLOTS);
    for (size_t i=0;i<SLOTS;++i) { obs->setScanRange(i,0); obs->setScanRangeValidity(i,false); }
    const double step=2*M_PI/(SLOTS-1);
    for (size_t k=0;k<out.input_points;++k)
    {
        double a=aa.array[k].numeric(), r=rr.array[k].numeric();
        if (r<=0 || r>max_range) continue;
        a=std::remainder(a,2*M_PI);
        // Use the actual, unquantized ray for matching. The LDS lever arm
        // belongs in base_link before the ICP pose transform.
        out.points.insertPoint(sensor_x+r*std::cos(a),r*std::sin(a));
        const size_t i=std::min(SLOTS-1,static_cast<size_t>(std::lround((a+M_PI)/step)));
        if (!obs->getScanRangeValidity(i) || r<obs->getScanRange(i))
        { obs->setScanRange(i,r); obs->setScanRangeValidity(i,true); }
    }
    if (out.points.empty()) throw std::runtime_error("scan has no valid rays");
    return out;
}
// Kept as a narrow unit-test entry point for sparse occupancy observation timing.
[[maybe_unused]] static mrpt::obs::CObservation2DRangeScan::Ptr scan_from(const std::string& line)
{ return read_scan(JsonReader(line).parse()).obs; }

static mrpt::maps::COccupancyGridMap2D::Ptr new_grid()
{
    auto g=mrpt::maps::COccupancyGridMap2D::Create();
    g->setSize(-10,10,-10,10,config.resolution);
    g->insertionOptions.maxDistanceInsertion=config.max_range;
    g->insertionOptions.considerInvalidRangesAsFreeSpace=false;
    return g;
}
static void grid_from_simplemap(const mrpt::maps::CSimpleMap& sm,
                               mrpt::maps::COccupancyGridMap2D::Ptr& out)
{
    auto g=new_grid();
    for (const auto& kf:sm)
    {
        mrpt::poses::CPose3D p; kf.pose->getMean(p);
        for (const auto& obs:*kf.sf) g->insertObservationPtr(obs,std::optional<const mrpt::poses::CPose3D>(p));
    }
    out=g;
}
static bool cell_unknown(float p) { return std::abs(p-.5f)<1e-3f; }
static std::string grid_json(const mrpt::maps::COccupancyGridMap2D& g)
{
    const unsigned W=g.getSizeX(), H=g.getSizeY();
    unsigned x0=W,y0=H,x1=0,y1=0;
    for(unsigned y=0;y<H;++y) for(unsigned x=0;x<W;++x)
        if(!cell_unknown(g.getCell(x,y)))
        { x0=std::min(x0,x); y0=std::min(y0,y); x1=std::max(x1,x); y1=std::max(y1,y); }
    const unsigned w=x0<=x1?x1-x0+1:0,h=y0<=y1?y1-y0+1:0;
    const double res=g.getResolution();
    std::string out="\"grid\":{\"x_min\":"+num(g.getXMin()+(w?x0:0)*res)+
        ",\"y_min\":"+num(g.getYMin()+(h?y0:0)*res)+",\"res\":"+num(res)+
        ",\"w\":"+std::to_string(w)+",\"h\":"+std::to_string(h)+",\"cells\":\"";
    for(unsigned y=0;y<h;++y) for(unsigned x=0;x<w;++x)
    {
        if(x || y) out+=',';
        float p=g.getCell(x0+x,y0+y);
        out+=cell_unknown(p)?"-1":std::to_string(static_cast<int>(std::lround((1-p)*100)));
    }
    return out+"\"}";
}
static std::string pose_fields(const mrpt::poses::CPose2D& p)
{ return "{\"x\":"+num(p.x())+",\"y\":"+num(p.y())+",\"yaw\":"+num(p.phi())+"}"; }
struct Diagnostics
{
    size_t input=0,used=0,ref=0;
    bool accepted=false, mapping_paused=false;
    std::string tracking="degraded",reason="insufficient_points";
    std::optional<double> goodness,residual,correspondence_threshold;
    std::optional<size_t> correspondences;
    std::optional<unsigned> iterations;
    std::string covariance="null",correction="null";
    std::string json(double ms) const
    {
        return "\"tracking\":\""+tracking+"\",\"diagnostics\":{\"input_points\":"+std::to_string(input)+
            ",\"used_points\":"+std::to_string(used)+",\"reference_points\":"+std::to_string(ref)+
            ",\"accepted\":"+(accepted?"true":"false")+",\"mapping_paused\":"+(mapping_paused?"true":"false")+",\"reason\":\""+reason+
            "\",\"goodness\":"+(goodness?num(*goodness):"null")+
            ",\"correspondences\":"+(correspondences?std::to_string(*correspondences):"null")+
            ",\"correspondence_threshold_m\":"+(correspondence_threshold?num(*correspondence_threshold):"null")+
            ",\"iterations\":"+(iterations?std::to_string(*iterations):"null")+
            ",\"duration_ms\":"+num(ms)+",\"correction\":"+correction+
            ",\"covariance\":"+covariance+",\"residual_rms\":"+(residual?num(*residual):"null")+"}";
    }
};
static void insert_keyframe(const Scan& scan, const mrpt::poses::CPose2D& p)
{
    auto sf=mrpt::obs::CSensoryFrame::Create(); sf->insert(scan.obs);
    simplemap.insert(mrpt::poses::CPose3DPDFGaussian::Create(mrpt::poses::CPose3D(p)),sf);
    reference.insertAnotherMap(&scan.points,mrpt::poses::CPose3D(p));
    grid->insertObservationPtr(scan.obs,std::optional<const mrpt::poses::CPose3D>(mrpt::poses::CPose3D(p)));
    insertion_pose=p;
}
static Diagnostics update_mapping(const Scan& scan, const mrpt::poses::CPose2D& delta, bool compensated=false)
{
    Diagnostics d; d.input=scan.input_points; d.used=scan.points.size(); d.ref=reference.size();
    const auto predicted=last_pose+delta;
    if (reference.empty() || (reference.size()<6 && scan.points.size()>=6))
    {
        last_pose=predicted;
        insert_keyframe(scan,last_pose);
        d.tracking="initializing"; d.reason="bootstrap";
        return d;
    }
    if (scan.points.size()<6 || reference.size()<6) { last_pose=predicted; return d; }
    mrpt::slam::CICP icp;
    icp.setVerbosityLevel(std::getenv("MRPT_ADAPTER_DEBUG")?mrpt::system::LVL_DEBUG:mrpt::system::LVL_ERROR);
    icp.options.maxIterations=config.iterations;
    icp.options.thresholdDist=config.threshold;
    icp.options.corresponding_points_decimation=config.decimation;
    mrpt::slam::CICP::TReturnInfo info;
    const auto pdf=icp.AlignPDF(&reference,&scan.points,mrpt::poses::CPosePDFGaussian(predicted),info);
    auto [cov,matched]=pdf->getCovarianceAndMean();
    d.goodness=info.goodness; d.iterations=info.nIterations;
    const bool finite_pose=std::isfinite(matched.x())&&std::isfinite(matched.y())&&std::isfinite(matched.phi());
    bool finite_cov=true;
    std::string covariance="[";
    for (size_t i=0;i<3;++i) for (size_t j=0;j<3;++j)
    { if(i||j) covariance+=','; covariance+=num(cov(i,j)); finite_cov &= std::isfinite(cov(i,j)); }
    covariance+=']';
    if (finite_cov) d.covariance=covariance;
    if (finite_pose)
    {
        // Correction in map axes, relative to the odometry prediction.
        d.correction=pose_fields(mrpt::poses::CPose2D(matched.x()-predicted.x(),matched.y()-predicted.y(),
                                                   std::remainder(matched.phi()-predicted.phi(),2*M_PI)));
        // Recompute all post-fit pairings at an explicit fixed metric threshold;
        // this count is not inferred from MRPT's decimated goodness ratio.
        mrpt::maps::TMatchingParams params;
        params.maxDistForCorrespondence=std::min(config.threshold,icp.options.smallestThresholdDist);
        params.decimation_other_map_points=1;
        mrpt::maps::TMatchingExtraResults extra;
        mrpt::tfest::TMatchingPairList pairs;
        reference.determineMatching2D(&scan.points,matched,pairs,params,extra);
        d.correspondences=pairs.size();
        d.correspondence_threshold=params.maxDistForCorrespondence;
        if(!pairs.empty()) d.residual=std::sqrt(extra.sumSqrDist/pairs.size());
    }
    d.accepted=finite_pose&&finite_cov&&std::isfinite(info.goodness)&&info.goodness>=config.min_goodness;
    d.reason=d.accepted?"accepted":finite_pose&&finite_cov?"low_goodness":"nonfinite_result";
    d.tracking=d.accepted?"tracking":"degraded";
    last_pose=d.accepted?matched:predicted;
    d.mapping_paused=!compensated &&
        (std::hypot(delta.x(),delta.y())>config.uncompensated_linear ||
         std::abs(delta.phi())>config.uncompensated_angular);
    if(d.accepted && d.mapping_paused)
    {
        d.reason="uncompensated_motion";
        d.tracking="degraded";
    }
    if(d.accepted && !d.mapping_paused)
    {
        const auto distance=last_pose-insertion_pose;
        if(std::hypot(distance.x(),distance.y())>=config.insertion_linear ||
           std::abs(std::remainder(distance.phi(),2*M_PI))>=config.insertion_angular)
            insert_keyframe(scan,last_pose);
    }
    return d;
}
int main()
{
    std::ios::sync_with_stdio(false);
    proto_fd=dup(1); if(proto_fd<0) return 1;
    dup2(2,1);
    mrpt::bayes::CParticleFilter pf;
    pf.m_options.PF_algorithm=mrpt::bayes::CParticleFilter::pfStandardProposal;
    std::string line;
    while(std::getline(std::cin,line))
    {
        try
        {
            if(line.size()>4*1024*1024) throw std::runtime_error("request too large");
            const auto req=JsonReader(line).parse();
            const auto op=text(req,"op");
            if(op=="ping") reply_ok("\"mode\":"+std::string(mapping?"\"mapping\"":mcl?"\"localize\"":"\"idle\""));
            else if(op=="start_mapping")
            {
                auto next_config=Config::load();
                config=next_config;
                auto fresh=new_grid();
                grid=fresh; simplemap.clear(); reference.clear(); mcl.reset();
                mapping=true; last_pose={0,0,0}; insertion_pose=last_pose;
                reply_ok("\"mode\":\"mapping\"");
            }
            else if(op=="load_map")
            {
                const auto path=text(req,"path");
                if(path.empty()) throw std::runtime_error("load_map needs a path");
                const auto next_config=Config::load();
                mrpt::maps::CSimpleMap sm;
                if(!sm.loadFromFile(path) || sm.empty()) throw std::runtime_error("cannot load nonempty simplemap");
                config=next_config;
                mrpt::maps::COccupancyGridMap2D::Ptr loaded;
                grid_from_simplemap(sm,loaded);
                auto next=std::make_unique<mrpt::slam::CMonteCarloLocalization2D>(300);
                next->options.metricMap=std::static_pointer_cast<const mrpt::maps::CMetricMap>(loaded);
                next->resetUniformFreeSpace(loaded.get(),.7f,-1);
                mcl=std::move(next); grid=loaded; mapping=false;
                simplemap.clear(); reference.clear(); last_pose={0,0,0};
                reply_ok("\"mode\":\"localize\"");
            }
            else if(op=="update")
            {
                const auto start=std::chrono::steady_clock::now();
                const auto scan=read_scan(req);
                const auto& odom=req.get("odom");
                if(odom.type!=Json::Null && odom.type!=Json::Object) throw std::runtime_error("invalid odom");
                const double dx=number(odom,"dx",0),dy=number(odom,"dy",0),dyaw=number(odom,"dyaw",0);
                mrpt::poses::CPose2D delta(dx,dy,std::remainder(dyaw,2*M_PI));
                Diagnostics d;
                std::optional<double> localization_quality;
                const auto& compensated=req.get("motion_compensated");
                if(compensated.type!=Json::Null && compensated.type!=Json::Boolean)
                    throw std::runtime_error("motion_compensated must be boolean");
                if(mapping) d=update_mapping(scan,delta,compensated.type==Json::Boolean && compensated.number!=0);
                else if(mcl)
                {
                    mrpt::obs::CActionCollection acts;
                    auto move=mrpt::obs::CActionRobotMovement2D::Create();
                    move->computeFromOdometry(delta,mrpt::obs::CActionRobotMovement2D::TMotionModelOptions()); acts.insertPtr(move);
                    mrpt::obs::CSensoryFrame sf; sf.insert(scan.obs);
                    pf.executeOn(*mcl,&acts,&sf); mcl->getMean(last_pose);
                    // Preserve the legacy MCL spread proxy; it is not ICP
                    // goodness and must not be copied into that diagnostic.
                    const auto [cov,mean]=mcl->getCovarianceAndMean();
                    const double spread=std::sqrt(cov(0,0)+cov(1,1)+cov(2,2));
                    if(std::isfinite(spread)) localization_quality=1-std::min(1.0,spread/.4);
                    d.input=scan.input_points; d.used=scan.points.size();
                    d.tracking="tracking"; d.reason="mcl_update";
                }
                else throw std::runtime_error("no mode selected: send start_mapping or load_map first");
                const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
                reply_ok("\"pose\":"+pose_fields(last_pose)+",\"quality\":"+(d.goodness?num(*d.goodness):"null")+
                         (mcl?",\"localization_quality\":"+(localization_quality?num(*localization_quality):"null"):"")+","+d.json(ms));
            }
            else if(op=="map")
            {
                if(!grid) throw std::runtime_error("no map yet");
                reply_ok(grid_json(*grid));
            }
            else if(op=="save_map")
            {
                const auto path=text(req,"path");
                if(path.empty()) throw std::runtime_error("save_map needs a path");
                if(!mapping) throw std::runtime_error("save_map is only available in mapping mode");
                if(!simplemap.saveToFile(path)) throw std::runtime_error("cannot write simplemap");
                reply_ok();
            }
            else if(op=="local_reset" || op=="global_reset")
            {
                if(!mcl) throw std::runtime_error("not in localize mode");
                if(op=="global_reset") mcl->resetUniformFreeSpace(grid.get(),.7f,-1);
                else mcl->resetUniformFreeSpace(grid.get(),.7f,-1,last_pose.x()-.5f,last_pose.x()+.5f,
                                                last_pose.y()-.5f,last_pose.y()+.5f,-M_PI,M_PI);
                reply_ok();
            }
            else throw std::runtime_error("unknown op: "+op);
        }
        catch(const std::exception& e) { reply_err(e.what()); }
    }
    return 0;
}
