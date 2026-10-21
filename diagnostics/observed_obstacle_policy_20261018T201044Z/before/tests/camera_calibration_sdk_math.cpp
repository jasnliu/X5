// Offline regression: actual SDK pure-math API, no Context/devices/CAN/streams.
// Include the production converter without invoking its camera entry point.
#define main unused_camera_entry_point
#include "../camera_calibration/orbbec_capture.cpp"
#undef main

int main() {
    const OBExtrinsic cases[] = {
        {{1,0,0, 0,1,0, 0,0,1}, {0,0,0}},
        {{0,-1,0, 1,0,0, 0,0,1}, {10,-20,30}},
        {{1,0,0, 0,.8660254038f,-.5f, 0,.5f,.8660254038f}, {-24.25f,1.75f,-3.5f}}
    };
    try {
        std::cout << '[';
        for (int i=0; i<3; ++i) {
            if (i) std::cout << ',';
            std::cout << "{\"matrix\":" << extrinsic(cases[i]) << ",\"points\":[";
            const OBPoint3f points[] = {{50,-30,750}, {-100,200,1200}};
            for (int j=0; j<2; ++j) {
                OBPoint3f q{};
                if (!ob::CoordinateTransformHelper::transformation3dto3d(points[j], cases[i], &q))
                    throw std::runtime_error("Offline test point rejected");
                if (j) std::cout << ',';
                std::cout << std::setprecision(12) << "{\"input_m\":[" << points[j].x/1000.0 << ','
                          << points[j].y/1000.0 << ',' << points[j].z/1000.0
                          << "],\"output_m\":[" << q.x/1000.0 << ',' << q.y/1000.0 << ',' << q.z/1000.0 << "]}";
            }
            std::cout << "]}";
        }
        std::cout << "]\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
