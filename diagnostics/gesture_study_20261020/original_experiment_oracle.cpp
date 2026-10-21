// Device-free, persistent FCL oracle for software experiments. Reuse EXACT
// model, self-filter, hard margins and edge checks from the production backend.
#define main x5_unused_file_backend_main
#include "moveit_plan.cpp"
#undef main
struct ExperimentOracle {
  Checker c;
  Q start,goal;
  std::map<std::string,double> nominal;
  std::vector<std::string> ids;
  ExperimentOracle(moveit::core::RobotModelPtr m,const Json::Value& r):c(m,r),start(vector7(r["start"])),
    goal(vector7(r["goal"])),nominal(c.scene.getCollisionEnv()->getLinkPadding()),
    ids(c.scene.getWorld()->getObjectIds()) {
    if(!c.initial_bounds["valid"].asBool())throw std::runtime_error("Saved start violates calibrated joint limits");
  }
  void restore() {
    c.self_clearance=0.;
    c.scene.getCollisionEnvNonConst()->setLinkPadding(nominal);
    for(const auto& id:ids)c.scene.getWorldNonConst()->setObjectPose(id,Eigen::Isometry3d::Identity());
  }
  void perturb(const Json::Value& r) {
    c.self_clearance=0.;
    double extra=r.get("extra_padding",0.).asDouble(),tool=r.get("tool_padding",0.).asDouble();
    if(!std::isfinite(extra)||extra<0||extra>.3||!std::isfinite(tool)||tool<0||tool>.1)
      throw std::runtime_error("Invalid experimental padding");
    auto pads=nominal;for(auto& p:pads)p.second+=extra+(p.first=="right_drumstick"?tool:0.);
    c.scene.getCollisionEnvNonConst()->setLinkPadding(pads);
    if(r.isMember("world_transform")) {
      auto a=r["world_transform"];if(a.size()!=4)throw std::runtime_error("Expected SE3 world transform");
      Eigen::Isometry3d t=Eigen::Isometry3d::Identity();
      for(int i=0;i<4;i++)for(int j=0;j<4;j++)t.matrix()(i,j)=a[i][j].asDouble();
      if(!t.matrix().allFinite()||(t.linear().transpose()*t.linear()-Eigen::Matrix3d::Identity()).norm()>1e-6||
         std::abs(t.linear().determinant()-1)>1e-6||t.translation().norm()>.3)
        throw std::runtime_error("Invalid world transform");
      for(const auto& id:ids)c.scene.getWorldNonConst()->setObjectPose(id,t);
    }else for(const auto& id:ids)c.scene.getWorldNonConst()->setObjectPose(id,Eigen::Isometry3d::Identity());
  }
  Json::Value plan(const Json::Value& r) {
    restore();Q a=r.isMember("start")?vector7(r["start"]):start,b=r.isMember("goal")?vector7(r["goal"]):goal;
    double seconds=r.get("seconds",3.).asDouble();
    if(!std::isfinite(seconds)||seconds<=0||seconds>60)throw std::runtime_error("Invalid search time");
    auto valid=[&](const Q& q){return c.valid(q);};
    Json::Value out;if(!valid(a)||!valid(b)){out["status"]="blocked_endpoint";out["details"]=c.last;restore();return out;}
    auto space=std::make_shared<ob::RealVectorStateSpace>(7);ob::RealVectorBounds bounds(7);
    auto names=c.group->getVariableNames();for(size_t i=0;i<7;i++){auto v=c.model->getVariableBounds(names[i]);bounds.setLow(i,v.min_position_);bounds.setHigh(i,v.max_position_);}
    space->setBounds(bounds);og::SimpleSetup ss(space);ss.setStateValidityChecker([&](const ob::State* s){return valid(fromState(s));});
    struct MarginEdges:ob::MotionValidator {
      std::function<bool(const Q&)> valid;
      MarginEdges(ob::SpaceInformationPtr si,std::function<bool(const Q&)> v):ob::MotionValidator(si),valid(v){}
      bool checkMotion(const ob::State* a,const ob::State* b)const override {
        Q x=fromState(a),y=fromState(b);double d=0;for(int j=0;j<7;j++)d+=std::abs(x[j]-y[j]);
        int n=std::max(1,int(std::ceil(d/EDGE_L1)));for(int i=1;i<=n;i++){Q q(7);for(int j=0;j<7;j++)q[j]=x[j]+(y[j]-x[j])*i/n;if(!valid(q))return false;}return true;
      }
      bool checkMotion(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>& last)const override {
        if(checkMotion(a,b))return true;
        last.second=0;if(last.first)si_->copyState(last.first,a);return false;
      }
    };
    ss.getSpaceInformation()->setMotionValidator(std::make_shared<MarginEdges>(ss.getSpaceInformation(),valid));
    auto planner=std::make_shared<og::RRTConnect>(ss.getSpaceInformation());planner->setRange(.25);ss.setPlanner(planner);
    ob::ScopedState<> x(space),y(space);for(int j=0;j<7;j++){x[j]=a[j];y[j]=b[j];}ss.setStartAndGoalStates(x,y,1e-9);
    if(ss.solve(seconds)==ob::PlannerStatus::EXACT_SOLUTION) {
      ss.simplifySolution(.1);auto path=ss.getSolutionPath();out["status"]="planned";
      for(size_t i=0;i<path.getStateCount();i++)out["path"].append(jsonQ(i==0?a:(i+1==path.getStateCount()?b:fromState(path.getState(i)))));
    }else out["status"]="no_path";
    restore();return out;
  }
  Json::Value command(const Json::Value& r) {
    std::string op=r["op"].asString();Json::Value out;
    if(op=="metadata") {
      for(const auto& name:c.group->getVariableNames()){auto b=c.model->getVariableBounds(name);out["lower"].append(b.min_position_);out["upper"].append(b.max_position_);}
      out["joint_limit_check"]=c.initial_bounds;out["self_filter"]=c.self_filter;
      out["right_zone_constraint"]=c.right_zone.metadata;
    }else if(op=="zone") {
      if(!r["q"].isArray()||r["q"].size()>10000)throw std::runtime_error("Zone diagnostic batch must contain <=10000 states");
      for(const auto& q:r["q"]){Json::Value row;c.right_zone.check(vector7(q),&row,true);out["rows"].append(row);}
    }else if(op=="plan")out=plan(r);
    else if(op=="style") {restore();out["rows"]=styleFeatures(c,r["paths"]);restore();}
    else if(op=="check") {
      if(!r["q"].isArray()||r["q"].size()>10000)throw std::runtime_error("Check batch must contain <=10000 states");
      perturb(r);
      c.self_clearance=r.get("self_margin",0.).asDouble();
      if(!std::isfinite(c.self_clearance)||c.self_clearance<0||c.self_clearance>.3)throw std::runtime_error("Invalid self margin");
      Q previous;out["valid"]=Json::Value(Json::arrayValue);
      for(const auto& a:r["q"]){Q q=vector7(a);bool ok=c.valid(q,true);
        if(ok&&r.get("edges",false).asBool()&&!previous.empty())ok=c.edge(previous,q);
        out["valid"].append(ok);if(!ok){out["details"]=c.last;if(r.get("stop",false).asBool())break;}previous=q;}
      restore();
    }else throw std::runtime_error("Unknown oracle command");
    out["checks"]=Json::UInt64(c.checks);return out;
  }
};
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  setenv("ROS_DOMAIN_ID","197",1);setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);rclcpp::init(argc,argv);
  try {
    Json::Value r;std::ifstream f(argv[1]);if(!f||!(f>>r)||r["schema"]!="x5-moveit-request-v1"||r["execute"]!=false)throw std::runtime_error("Invalid software request");
    ompl::RNG::setSeed(r.get("seed",19).asUInt());
    auto u=urdf::parseURDF(textfile(r["urdf"].asString()));auto s=std::make_shared<srdf::Model>();
    if(!u||!s->initString(*u,textfile(r["srdf"].asString())))throw std::runtime_error("Invalid robot model");
    ExperimentOracle oracle(std::make_shared<moveit::core::RobotModel>(u,s),r);
    std::string line;Json::StreamWriterBuilder w;w["indentation"]="";
    std::cout<<"@x5 {\"ready\":true}"<<std::endl;
    while(std::getline(std::cin,line)) {
      Json::Value out;
      try {Json::Value q;std::istringstream input(line);if(!(input>>q))throw std::runtime_error("Invalid JSON");out=oracle.command(q);}
      catch(const std::exception& e){oracle.restore();out["error"]=e.what();}
      std::cout<<"@x5 "<<Json::writeString(w,out)<<std::endl;
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
