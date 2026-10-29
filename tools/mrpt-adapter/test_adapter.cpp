// FEATURE: Explicit ray geometry, honest timing, strict JSON and incremental grid.
// SPEC: Real XY rays survive even when angular bins collide. Use supplied finite
// time, otherwise current time, never a synthetic 50 ms clock. Malformed numeric
// measurements are not zeros. Map exports cannot replay or mutate the history.
// IMPLEMENTATION: Direct tests of the production reader and controlled pipeline.
// RESULT: IMPLEMENTED — timestamps, ray coordinates, JSON, persistent occupancy.
// STATUS: DONE
#define main adapter_protocol_main
#include "adapter.cpp"
#undef main
#include <cassert>

static void test_timing()
{
    auto obs=scan_from(R"({"scan":{"angles":[0],"ranges":[2]},"stamp":123.25})");
    assert(std::abs(mrpt::Clock::toDouble(obs->timestamp)-123.25)<1e-6);
    auto a=scan_from(R"({"scan":{"angles":[0],"ranges":[2]}})");
    auto b=scan_from(R"({"scan":{"angles":[0],"ranges":[2]}})");
    assert(std::abs(mrpt::Clock::toDouble(b->timestamp)-mrpt::Clock::toDouble(a->timestamp))<0.02);
}
static void test_json()
{
    for(const auto& raw:{
        R"({"scan":{"angles":["bad"],"ranges":[2]}})",
        R"({"scan":{"angles":[null],"ranges":[2]}})",
        R"({"scan":{"angles":[0],"ranges":[2]},"stamp":null})",
        R"({"scan":{"angles":[0],"ranges":[2]},"stamp":1e300})",
        R"({"scan":{"angles":[1e999],"ranges":[2]}})",
        R"({"scan":{"angles":[0],"ranges":[2,]}})",
        R"({"scan":{"angles":[0],"ranges":[2]},"scan":{}})",
        R"({"scan":{"angles":[0],"ranges":[2]}} trailing)"})
    {
        bool rejected=false;
        try { scan_from(raw); } catch(const std::exception&) { rejected=true; }
        assert(rejected);
    }
    auto j=JsonReader(R"({"op":"ping","path":"a\"b\\c\tА😀"})").parse();
    assert(text(j,"path")=="a\"b\\c\tА\xf0\x9f\x98\x80");
    assert(json_escape("\r\t\n")=="\\u000d\\u0009\\u000a");
}
static void test_real_xy_rays()
{
    auto s=read_scan(JsonReader(R"({"scan":{"angles":[0,0.001,0.002,1.123],"ranges":[2,3,4,2]},"sensor_x":-0.1})").parse());
    assert(s.points.size()==4);
    const auto& xs=s.points.getPointsBufferRef_x();
    const auto& ys=s.points.getPointsBufferRef_y();
    assert(std::abs(xs[0]-1.9)<1e-6);
    assert(std::abs(xs[3]-(-.1+2*std::cos(1.123)))<1e-6);
    assert(std::abs(ys[3]-2*std::sin(1.123))<1e-6);
    size_t valid=0;
    for(size_t i=0;i<SLOTS;++i) valid+=s.obs->getScanRangeValidity(i);
    assert(valid==2); // Occupancy bins may collide; matching rays may not.
}
static void test_incremental_grid()
{
    grid=new_grid(); reference.clear(); simplemap.clear(); last_pose={0,0,0};
    auto s=read_scan(JsonReader(R"({"scan":{"angles":[-2,-1,0,1,2,3],"ranges":[2,2,2,2,2,2]}})").parse());
    auto d=update_mapping(s,{0,0,0});
    assert(d.reason=="bootstrap" && !d.accepted);
    assert(reference.size()==6 && simplemap.size()==1);
    auto* identity=grid.get();
    const auto before=grid_json(*grid);
    for(int i=0;i<10;++i) assert(grid_json(*grid)==before);
    auto bad=read_scan(JsonReader(R"({"scan":{"angles":[0],"ranges":[0.4]}})").parse());
    d=update_mapping(bad,{.1,0,0});
    assert(!d.accepted && d.reason=="insufficient_points");
    assert(grid.get()==identity && grid_json(*grid)==before);
    assert(reference.size()==6 && simplemap.size()==1);
    // Explicitly trusted keyframes update the same persistent grid once.
    insert_keyframe(s,{.2,0,0});
    assert(grid.get()==identity && reference.size()==12 && simplemap.size()==2);
    mrpt::maps::COccupancyGridMap2D::Ptr rebuilt;
    grid_from_simplemap(simplemap,rebuilt);
    assert(grid_json(*rebuilt)==grid_json(*grid));
}
int main()
{
    test_timing(); test_json(); test_real_xy_rays(); test_incremental_grid();
    std::cerr << "4 C++ adapter tests passed\n";
}
