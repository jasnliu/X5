// Depth-only full-FOV camera bridge. No robot interface; no camera property writes.
#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

static volatile std::sig_atomic_t running = 1;
static void stop(int) { running = 0; }
static double monotonic() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string quote(const char *s) {
    std::ostringstream out;
    out << '"';
    for (; *s; ++s) {
        unsigned char c = *s;
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << '?';
        else out << c;
    }
    out << '"';
    return out.str();
}
static std::shared_ptr<ob::VideoStreamProfile> profile(ob::Pipeline &p, OBSensorType sensor,
                                                     int w, int h, OBFormat format, int fps) {
    auto list = p.getStreamProfileList(sensor);
    for (uint32_t i = 0; i < list->getCount(); ++i) {
        auto v = list->getProfile(i)->as<ob::VideoStreamProfile>();
        if (v->getWidth() == unsigned(w) && v->getHeight() == unsigned(h)
            && v->getFormat() == format && v->getFps() == unsigned(fps)) return v;
    }
    throw std::runtime_error("Requested full-FOV profile unavailable. No cropped fallback was selected.");
}
static std::string parameters(const std::shared_ptr<ob::VideoStreamProfile> &p) {
    auto k = p->getIntrinsic(); auto d = p->getDistortion();
    std::ostringstream o; o << std::setprecision(12);
    o << "{\"width\":" << p->getWidth() << ",\"height\":" << p->getHeight()
      << ",\"fps\":" << p->getFps() << ",\"format_enum\":" << p->getFormat()
      << ",\"intrinsic_width\":" << k.width << ",\"intrinsic_height\":" << k.height
      << ",\"K\":[[" << k.fx << ",0," << k.cx << "],[0," << k.fy << ',' << k.cy << "],[0,0,1]]"
      << ",\"distortion_model\":" << d.model
      << ",\"D_opencv_order\":[" << d.k1 << ',' << d.k2 << ',' << d.p1 << ',' << d.p2
      << ',' << d.k3 << ',' << d.k4 << ',' << d.k5 << ',' << d.k6 << "]}";
    return o.str();
}
static std::string extrinsic(const OBExtrinsic &e) {
    // Ask the SDK to transform a translated basis, avoiding any assumption
    // about rot[] storage order. The SDK rejects points with z == 0, even for
    // this 3D-to-3D operation. All four test points must have positive depth.
    // Anchor p0=(0,0,1000) mm; differences give R, then t=q0-R*p0.
    OBPoint3f p[4] = {{0,0,1000}, {1000,0,1000}, {0,1000,1000}, {0,0,2000}}, q[4];
    for (int i=0; i<4; ++i)
        if (!ob::CoordinateTransformHelper::transformation3dto3d(p[i], e, &q[i]))
            throw std::runtime_error("SDK could not transform the depth/color basis");
    double a[4][3];
    for (int i=0; i<4; ++i) { a[i][0]=q[i].x; a[i][1]=q[i].y; a[i][2]=q[i].z; }
    std::ostringstream o; o << std::setprecision(12) << '[';
    for (int r=0; r<3; ++r) {
        o << '[';
        for (int c=1; c<=3; ++c) o << (a[c][r]-a[0][r])/1000.0 << ',';
        o << (2*a[0][r]-a[3][r])/1000.0 << "],";
    }
    o << "[0,0,0,1]]"; return o.str();
}
int main(int argc, char **argv) {
    if (argc != 2) { std::cerr << "usage: depth_capture SERIAL\n"; return 2; }
    std::signal(SIGINT, stop); std::signal(SIGTERM, stop);
    try {
        ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_OFF);
        ob::Context::setLoggerToFile(OB_LOG_SEVERITY_WARN, "depth_sdk_logs");
        ob::Context context; context.setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
        auto devices = context.queryDeviceList(); std::shared_ptr<ob::Device> device;
        for (uint32_t i=0; i<devices->getCount(); ++i)
            if (std::string(devices->getSerialNumber(i)) == argv[1]) device = devices->getDevice(i);
        if (!device) throw std::runtime_error("Calibrated Gemini 2 not found or busy");
        auto info = device->getDeviceInfo();
        if (std::string(info->getName()).find("Gemini 2") == std::string::npos)
            throw std::runtime_error("Expected Gemini 2");
        ob::Pipeline pipeline(device);
        auto mode = device->getCurrentDepthWorkMode();
        auto depth = profile(pipeline, OB_SENSOR_DEPTH, 1280, 800, OB_FORMAT_Y16, 10);
        auto color = profile(pipeline, OB_SENSOR_COLOR, 1280, 720, OB_FORMAT_YUYV, 15);
        auto config = std::make_shared<ob::Config>(); config->enableStream(depth);
        pipeline.start(config);
        std::string initial; unsigned long long sequence=0; int misses=0;
        while (running) {
            auto frames = pipeline.waitForFrameset(1000);
            if (!frames || !frames->getDepthFrame()) {
                if (++misses >= 5) throw std::runtime_error("No depth frames for five seconds");
                continue;
            }
            misses=0;
            auto frame = frames->getDepthFrame(); double received=monotonic();
            auto actual = frame->getStreamProfile()->as<ob::VideoStreamProfile>();
            auto params = parameters(actual);
            if (actual->getWidth()!=1280 || actual->getHeight()!=800 || actual->getFormat()!=OB_FORMAT_Y16
                || actual->getFps()!=10 || frame->getDataSize()!=1280*800*2)
                throw std::runtime_error("Partial or wrong full-FOV Y16 frame; no fallback");
            if (initial.empty()) {
                initial=params;
                std::cout << "{\"protocol\":1,\"serial\":" << quote(info->getSerialNumber())
                    << ",\"name\":" << quote(info->getName()) << ",\"firmware\":" << quote(info->getFirmwareVersion())
                    << ",\"depth_work_mode\":" << quote(mode.name)
                    << ",\"depth\":" << params << ",\"color\":" << parameters(color)
                    << ",\"T_color_depth_m\":" << extrinsic(actual->getExtrinsicTo(color))
                    << ",\"depth_stream_enabled\":true,\"color_stream_enabled\":false,\"alignment\":\"native_depth\"}\n" << std::flush;
            } else if (initial!=params) throw std::runtime_error("Depth profile changed during capture");
            std::cout << std::setprecision(17) << "{\"n\":" << frame->getDataSize()
                << ",\"sequence\":" << ++sequence << ",\"received_monotonic\":" << received
                << ",\"device_timestamp_us\":" << frame->getTimeStampUs()
                << ",\"value_scale_m\":" << frame->getValueScale()*.001 << "}\n";
            std::cout.write(reinterpret_cast<const char*>(frame->getData()),frame->getDataSize());
            std::cout.flush(); if (!std::cout) break;
        }
        pipeline.stop(); return 0;
    } catch (const ob::Error &e) { std::cerr << "Orbbec: " << e.getMessage() << '\n'; }
      catch (const std::exception &e) { std::cerr << e.what() << '\n'; }
    return 1;
}
