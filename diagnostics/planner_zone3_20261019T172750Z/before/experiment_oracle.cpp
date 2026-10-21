// Device-free, persistent FCL oracle for software experiments. Reuse EXACT
// model, self-filter, hard margins and edge checks from the production backend.
#define main x5_unused_file_backend_main
#include "moveit_plan.cpp"
#undef main
#include <pluginlib/class_loader.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.hpp>
#include <chomp_motion_planner/chomp_planner.hpp>
#include <chomp_motion_planner/chomp_parameters.hpp>

const std::set<std::string> distal_links={"openarmx_right_link5","openarmx_right_link6",
 "openarmx_right_link7","openarmx_right_hand","openarmx_right_left_finger",
 "openarmx_right_right_finger","right_drumstick"};

struct ExperimentOracle {
  Checker c;
  Q start,goal;
  std::map<std::string,double> nominal;
  std::vector<std::string> ids;
  collision_detection::AllowedCollisionMatrix cost_acm;
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> stomp_loader;
  rclcpp::Node::SharedPtr node;
  ExperimentOracle(moveit::core::RobotModelPtr m,const Json::Value& r):c(m,r),start(vector7(r["start"])),
    goal(vector7(r["goal"])),nominal(c.scene.getCollisionEnv()->getLinkPadding()),
    ids(c.scene.getWorld()->getObjectIds()),cost_acm(c.scene.getAllowedCollisionMatrix()) {
    if(!c.initial_bounds["valid"].asBool())throw std::runtime_error("Saved start violates calibrated joint limits");
    for(auto link:m->getLinkModelsWithCollisionGeometry())if(!distal_links.count(link->getName()))
      for(const auto& id:ids)cost_acm.setEntry(link->getName(),id,true);
    node=std::make_shared<rclcpp::Node>("x5_experiment_software_only");
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
  Json::Value features(const Json::Value& qs) {
    if(qs.size()>4096)throw std::runtime_error("Feature batch exceeds bounded size");
    Json::Value out;out["rows"]=Json::Value(Json::arrayValue);
    auto env=c.scene.getCollisionEnvNonConst();auto saved=env->getLinkPadding();
    auto stick=c.model->getLinkModel("right_drumstick");
    double half=static_cast<const shapes::Cylinder*>(stick->getShapes()[0].get())->length*.5;
    std::vector<moveit::core::RobotState> states;std::vector<bool> found(qs.size(),false);
    for(const auto& value:qs) {
      Q q=vector7(value);bool valid=c.valid(q);Json::Value row;row["valid"]=valid;
      Eigen::Vector3d wrist=c.state.getGlobalLinkTransform("openarmx_right_hand").translation();
      Eigen::Vector3d tip=c.state.getCollisionBodyTransform(stick,0)*Eigen::Vector3d(0,0,half);
      row["wrist"]=jsonQ(Q{wrist.x(),wrist.y(),wrist.z()});row["tip"]=jsonQ(Q{tip.x(),tip.y(),tip.z()});
      row["clearance_m"]=.16;out["rows"].append(row);states.push_back(c.state);
    }
    // Padding changes rebuild collision shapes. Group ALL samples by band so
    // geometry is rebuilt 33 times per batch, not six times per state.
    collision_detection::CollisionRequest query;query.group_name="right_arm";
    for(int band=0;band<=32;band++) {
      double pad=.005*band;auto pads=saved;
      for(const auto& name:distal_links)pads[name]=pad;
      env->setLinkPadding(pads);
      for(size_t i=0;i<states.size();i++) {
        if(found[i])continue;
        collision_detection::CollisionResult res;
        env->checkRobotCollision(query,res,states[i],cost_acm);
        if(res.collision){found[i]=true;out["rows"][Json::ArrayIndex(i)]["clearance_m"]=std::max(0.,pad-.005);}
      }
    }
    env->setLinkPadding(saved);return out;
  }
  Json::Value plan(const Json::Value& r) {
    restore();Q a=r.isMember("start")?vector7(r["start"]):start,b=r.isMember("goal")?vector7(r["goal"]):goal;
    double margin=r.get("margin",0.).asDouble();bool taper=r.get("taper",false).asBool();
    double seconds=r.get("seconds",3.).asDouble();
    if(!std::isfinite(margin)||margin<0||margin>.25||!std::isfinite(seconds)||seconds<=0||seconds>60)
      throw std::runtime_error("Invalid search parameters");
    auto valid=[&](const Q& q){
      double added=margin;
      if(taper){double da=0,db=0;for(size_t i=0;i<7;i++){da+=pow(q[i]-start[i],2);db+=pow(q[i]-goal[i],2);}
        added*=std::min(1.,std::min(std::sqrt(da),std::sqrt(db))/.6);}
      auto pads=nominal;for(auto& p:pads)if(!taper||distal_links.count(p.first))p.second+=added;
      c.scene.getCollisionEnvNonConst()->setLinkPadding(pads);return c.valid(q);
    };
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
  Json::Value native(const Json::Value& r) {
    restore();std::string method=r["method"].asString();double seconds=r.get("seconds",20.).asDouble();
    if((method!="chomp"&&method!="stomp")||!std::isfinite(seconds)||seconds<=0||seconds>120)throw std::runtime_error("Invalid native planner request");
    c.state.setJointGroupPositions(c.group,start);c.state.update();c.scene.setCurrentState(c.state);
    // Native CHOMP tests every static joint with zero encoder tolerance. Match
    // ONLY the production checker's already accepted static-left allowance in
    // an isolated optimizer model, keeping measured FK exact and moving-right
    // limits unchanged. Final FCL uses the original untouched Checker model.
    auto nmodel=std::make_shared<moveit::core::RobotModel>(c.model->getURDF(),c.model->getSRDF());
    for(int i=1;i<=7;i++) {
      auto name="openarmx_left_joint"+std::to_string(i);auto bounds=nmodel->getVariableBounds(name);
      double value=c.state.getVariablePosition(name);
      if(value<bounds.min_position_||value>bounds.max_position_) {
        double outside=std::max(bounds.min_position_-value,value-bounds.max_position_);
        if(outside>STATIC_LEFT_ENCODER_MARGIN)throw std::runtime_error("Invalid static-left native shim");
        bounds.min_position_=std::min(bounds.min_position_,value);bounds.max_position_=std::max(bounds.max_position_,value);
        nmodel->getJointModel(name)->setVariableBounds(name,bounds);
      }
    }
    moveit::core::RobotState nstate(nmodel);nstate.setVariablePositions(c.state.getVariablePositions());nstate.update();
    auto ngroup=nmodel->getJointModelGroup("right_arm");
    planning_interface::MotionPlanRequest req;req.group_name="right_arm";req.allowed_planning_time=seconds;
    req.num_planning_attempts=1;req.max_velocity_scaling_factor=1.;req.max_acceleration_scaling_factor=1.;
    moveit::core::robotStateToRobotStateMsg(nstate,req.start_state);
    auto gs=nstate;gs.setJointGroupPositions(ngroup,goal);gs.update();
    req.goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(gs,ngroup,1e-8));
    Json::Value out;robot_trajectory::RobotTrajectoryPtr trajectory;int error=0;
    auto seed=std::make_shared<robot_trajectory::RobotTrajectory>(nmodel,"right_arm");
    for(const auto& value:r["seed_path"]) {
      auto q=vector7(value);auto s=nstate;s.setJointGroupPositions(ngroup,q);s.update();seed->addSuffixWayPoint(s,.1);
      req.trajectory_constraints.constraints.push_back(kinematic_constraints::constructGoalConstraints(s,ngroup,1e-8));
    }
    auto native_scene=std::make_shared<planning_scene::PlanningScene>(nmodel,std::make_shared<collision_detection::World>(*c.scene.getWorld()));
    native_scene->setCurrentState(nstate);native_scene->setAllowedCollisionMatrix(c.scene.getAllowedCollisionMatrix());
    native_scene->getCollisionEnvNonConst()->setLinkPadding(nominal);
    if(method=="chomp") {
      auto scene=native_scene;scene->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create());
      chomp::ChompParameters p;p.planning_time_limit_=seconds;p.max_iterations_=150;p.max_iterations_after_collision_free_=8;
      p.min_clearance_=.10;p.collision_threshold_=.02;p.trajectory_initialization_method_="fillTrajectory";
      p.use_stochastic_descent_=false;p.enable_failure_recovery_=false;
      chomp::ChompPlanner planner;planning_interface::MotionPlanDetailedResponse res;res.trajectory.push_back(seed);
      planner.solve(scene,req,p,res);error=res.error_code.val;
      if(error==1&&!res.trajectory.empty())trajectory=res.trajectory[0];
    }else {
      std::cerr<<"STOMP: loading native plugin"<<std::endl;
      // Keep plugin code loaded until every returned trajectory/control block
      // has been destroyed. A stack loader here unloads its shared_ptr deleters.
      if(!stomp_loader)stomp_loader=std::make_unique<pluginlib::ClassLoader<planning_interface::PlannerManager>>("moveit_core","planning_interface::PlannerManager");
      auto manager=stomp_loader->createSharedInstance("stomp_moveit/StompPlanner");
      // The installed MoveIt STOMP accepts a from-scratch seed via trajectory_constraints.
      std::cerr<<"STOMP: initializing manager"<<std::endl;
      manager->initialize(nmodel,node,"stomp");moveit_msgs::msg::MoveItErrorCodes code;
      std::cerr<<"STOMP: creating planning context"<<std::endl;
      auto context=manager->getPlanningContext(native_scene,req,code);planning_interface::MotionPlanResponse res;
      std::cerr<<"STOMP: solving"<<std::endl;
      if(context)context->solve(res);
      std::cerr<<"STOMP: solve returned"<<std::endl;
      error=context?res.error_code.val:code.val;trajectory=res.trajectory;
    }
    out["moveit_error_code"]=error;out["implementation"]="native MoveIt "+method;
    if(error!=1||!trajectory||trajectory->empty()){out["status"]="no_path";return out;}
    out["status"]="planned";
    for(size_t i=0;i<trajectory->getWayPointCount();i++){Q q;trajectory->getWayPoint(i).copyJointGroupPositions(ngroup,q);out["path"].append(jsonQ(q));}
    return out;
  }
  Json::Value command(const Json::Value& r) {
    std::string op=r["op"].asString();Json::Value out;
    if(op=="metadata") {
      for(const auto& name:c.group->getVariableNames()){auto b=c.model->getVariableBounds(name);out["lower"].append(b.min_position_);out["upper"].append(b.max_position_);}
      out["joint_limit_check"]=c.initial_bounds;out["self_filter"]=c.self_filter;
    }else if(op=="plan")out=plan(r);
    else if(op=="native")out=native(r);
    else if(op=="self_clearance") {
      double margin=r["distance"].asDouble();
      if(!std::isfinite(margin)||margin<0||margin>.3)throw std::runtime_error("Invalid self margin");
      c.self_clearance=margin;out["valid"]=true;
      for(const auto& a:r["q"])if(!c.valid(vector7(a))){out["valid"]=false;out["details"]=c.last;break;}
      c.self_clearance=0.;
    }else if(op=="features"){perturb(r);out=features(r["q"]);restore();}
    else if(op=="scenarios") {
      if(!r["scenarios"].isArray()||r["scenarios"].size()>16)throw std::runtime_error("At most 16 scenarios per batch");
      out["rows"]=Json::Value(Json::arrayValue);
      for(const auto& scenario:r["scenarios"]) {
        Json::Value paths=scenario["paths"];
        bool multiple=scenario.isMember("paths");
        if(!multiple){paths=Json::Value(Json::arrayValue);paths.append(scenario["q"]);}
        if(paths.empty()||paths.size()>4)throw std::runtime_error("At most four risk paths");
        for(const auto& path:paths)if(path.size()<2||path.size()>1000)throw std::runtime_error("Invalid scenario path size");
        // The same uncertainty realization applies to each candidate. Rebuild
        // geometry once per scenario/band, not once per candidate. No samples,
        // triangles or collision checks are omitted by this amortization.
        perturb(scenario);std::vector<bool> failed(paths.size(),false),endpoint_failed(paths.size(),false);
        for(size_t k=0;k<paths.size();k++) {
          const auto& path=paths[Json::ArrayIndex(k)];
          for(size_t i=0;i<path.size();i++)if(!c.valid(vector7(path[Json::ArrayIndex(i)]))) {
            failed[k]=true;if(i==0||i+1==path.size())endpoint_failed[k]=true;
          }
        }
        auto near=scenario;near["extra_padding"]=scenario.get("extra_padding",0.).asDouble()+.02;
        perturb(near);Json::Value results(Json::arrayValue);
        for(size_t k=0;k<paths.size();k++) {
          unsigned int hits=0;const auto& path=paths[Json::ArrayIndex(k)];
          for(const auto& v:path)if(!c.valid(vector7(v)))hits++;
          Json::Value row;row["failed"]=bool(failed[k]);row["endpoint_failed"]=bool(endpoint_failed[k]);
          row["loss"]=(failed[k]?10.:0.)+double(hits)/path.size();results.append(row);
        }
        out["rows"].append(multiple?results:results[0]);
      }
      restore();
    }
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
