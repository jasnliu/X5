#include <libobsensor/ObSensor.hpp>
#include <libobsensor/hpp/Utils.hpp>
#include <iostream>
#include <iomanip>
int main() {
 ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_OFF);
 OBCameraIntrinsic k{}; OBCameraDistortion d{}; OBExtrinsic e{};
 std::cin >> k.fx >> k.fy >> k.cx >> k.cy; k.width=1280;k.height=720;
 std::cin >> d.k1 >> d.k2 >> d.p1 >> d.p2 >> d.k3 >> d.k4 >> d.k5 >> d.k6;
 d.model=OB_DISTORTION_BROWN_CONRADY_K6;
 e.rot[0]=e.rot[4]=e.rot[8]=1;
 int n;std::cin>>n;
 for(int i=0;i<n;++i) {
  OBPoint3f p; OBPoint2f pixel; std::cin>>p.x>>p.y>>p.z;
  bool ok=ob::CoordinateTransformHelper::transformation3dto2d(p,k,d,e,&pixel);
  std::cout<<i<<' '<<ok<<' '<<std::setprecision(12)<<pixel.x<<' '<<pixel.y<<'\n';
 }
}
