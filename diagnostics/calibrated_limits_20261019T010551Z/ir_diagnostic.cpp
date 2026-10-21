// Diagnostic infrared snapshot only. No CAN or camera property writes.
#include <libobsensor/ObSensor.hpp>
#include <fstream>
#include <iostream>
int main(int argc,char**argv) {
 try {
  if(argc!=2)return 2;
  ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_OFF);
  ob::Context::setLoggerToFile(OB_LOG_SEVERITY_WARN,"ir_sdk_logs");
  ob::Context ctx;ctx.setUvcBackendType(OB_UVC_BACKEND_TYPE_LIBUVC);
  auto list=ctx.queryDeviceList();std::shared_ptr<ob::Device> dev;
  for(unsigned i=0;i<list->getCount();i++)if(std::string(list->getSerialNumber(i))=="AY6R463004K")dev=list->getDevice(i);
  if(!dev)throw std::runtime_error("Calibrated camera unavailable");
  ob::Pipeline pipe(dev);std::shared_ptr<ob::VideoStreamProfile> profile;
  for(auto sensor:{OB_SENSOR_IR_LEFT,OB_SENSOR_IR}) {
   try {
    auto profiles=pipe.getStreamProfileList(sensor);
    for(unsigned i=0;i<profiles->getCount();i++) {
     auto p=profiles->getProfile(i)->as<ob::VideoStreamProfile>();
     std::cerr<<sensor<<" "<<p->getWidth()<<"x"<<p->getHeight()<<" fmt "<<p->getFormat()<<" fps "<<p->getFps()<<"\n";
     if(p->getWidth()==1280&&p->getHeight()==800&&p->getFormat()==OB_FORMAT_Y8&&p->getFps()<=30)
      if(!profile||p->getFps()<profile->getFps())profile=p;
    }
   } catch(const ob::Error&e){std::cerr<<e.getMessage()<<"\n";}
   if(profile)break;
  }
  if(!profile)throw std::runtime_error("No full-FOV Y8 IR profile");
  auto cfg=std::make_shared<ob::Config>();cfg->enableStream(profile);pipe.start(cfg);
  int got=0;
  for(int i=0;i<20;i++) {
   auto frames=pipe.waitForFrameset(1000);if(!frames)continue;
   auto f=frames->getFrame(OB_FRAME_IR_LEFT);if(!f)f=frames->getFrame(OB_FRAME_IR);
   if(!f||f->getDataSize()!=1280*800)continue;
   if(++got<8)continue;
   std::ofstream out(argv[1],std::ios::binary);out<<"P5\n1280 800\n255\n";
   out.write(reinterpret_cast<const char*>(f->getData()),f->getDataSize());out.close();
   pipe.stop();std::cout<<"Saved full-FOV IR snapshot; no CAN, no property writes\n";return 0;
  }
  pipe.stop();throw std::runtime_error("IR frame timeout");
 }catch(const ob::Error&e){std::cerr<<e.getMessage()<<"\n";}catch(const std::exception&e){std::cerr<<e.what()<<"\n";}
 return 1;
}
