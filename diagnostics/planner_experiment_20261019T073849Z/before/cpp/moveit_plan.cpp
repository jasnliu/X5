// Standalone MoveIt PlanningScene/FCL + OMPL RRTConnect. Only a private-domain
// MoveIt logging node; NO controller, joint publisher, CAN or torque API.
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <octomap/OcTree.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <json/json.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <functional>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Q = std::vector<double>;
constexpr double EDGE_L1 = .003;  // <= 4.5 mm conservative lever-arm bound at 1.5 m.
// Measurement acceptance ONLY for the stationary, non-commanded left arm.
// Same three-count resolution allowance as X5 smooth_playback/trajectory.py.
// Never changes URDF limits, right-arm sampling/commands, or measured geometry.
constexpr double STATIC_LEFT_ENCODER_MARGIN = 3. * 25.14 / 65535.;

std::string textfile(const std::string& path) {
  std::ifstream f(path); if (!f) throw std::runtime_error("Cannot read " + path);
  return std::string(std::istreambuf_iterator<char>(f), {});
}
Q vector7(const Json::Value& a) {
  if (!a.isArray() || a.size()!=7) throw std::runtime_error("Expected seven joint angles");
  Q q;
  for (auto& v:a) {
    if (!v.isNumeric() || !std::isfinite(v.asDouble())) throw std::runtime_error("Nonfinite/non-numeric angle");
    q.push_back(v.asDouble());
  }
  return q;
}
Json::Value jsonQ(const Q& q) { Json::Value a(Json::arrayValue); for (double v:q) a.append(v); return a; }
Q fromState(const ob::State* s) {
  auto p=s->as<ob::RealVectorStateSpace::StateType>()->values; return Q(p,p+7);
}
template<class T> std::vector<T> binary(const std::string& path) {
  std::ifstream f(path,std::ios::binary|std::ios::ate);
  if (!f) throw std::runtime_error("Cannot open binary scene data");
  auto n=f.tellg(); if (n<0 || size_t(n)%sizeof(T)) throw std::runtime_error("Bad binary scene length");
  std::vector<T> data(size_t(n)/sizeof(T)); f.seekg(0);
  f.read(reinterpret_cast<char*>(data.data()),n);
  if (!f) throw std::runtime_error("Truncated scene data");
  return data;
}

struct Probe { const moveit::core::LinkModel* link; size_t shape; Eigen::Vector3d center; double radius; };
struct Checker {
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScene scene;
  moveit::core::RobotState state;
  const moveit::core::JointModelGroup* group;
  std::vector<Probe> probes;
  std::vector<float> depth;
  Eigen::Isometry3d camera_from_base=Eigen::Isometry3d::Identity();
  double fx=0,fy=0,cx=0,cy=0,padding=.02,self_clearance=0;
  bool strict=false;
  std::string collision_policy;
  size_t checks=0, masked=0;
  Json::Value last, initial_bounds, self_filter;
  Json::Value positionBounds() const {
    Json::Value report;
    report["reason"]="joint_limits";report["valid"]=true;
    report["angles_clamped"]=false;report["moving_right_limits_unchanged"]=true;
    report["static_left_encoder_tolerance_rad"]=STATIC_LEFT_ENCODER_MARGIN;
    report["violations"]=Json::Value(Json::arrayValue);
    report["accepted_static_encoder_overshoot"]=Json::Value(Json::arrayValue);
    for(const auto& name:model->getVariableNames()) {
      const auto& bounds=model->getVariableBounds(name);
      double value=state.getVariablePosition(name);
      bool static_left=false;
      for(int i=1;i<=7;i++) if(name=="openarmx_left_joint"+std::to_string(i))static_left=true;
      double tolerance=static_left?STATIC_LEFT_ENCODER_MARGIN:0.;
      double outside=bounds.position_bounded_?
        std::max({0.,bounds.min_position_-value,value-bounds.max_position_}):0.;
      if(std::isfinite(value) && outside==0.)continue;
      Json::Value row;row["joint"]=name;
      if(std::isfinite(value))row["measured_rad"]=value;
      row["lower_rad"]=bounds.min_position_;row["upper_rad"]=bounds.max_position_;
      row["overshoot_rad"]=outside;row["measurement_tolerance_rad"]=tolerance;
      if(std::isfinite(value) && outside<=tolerance) {
        row["handling"]="Static left measurement retained exactly for FK/collision checking; no commands or clipping";
        report["accepted_static_encoder_overshoot"].append(row);
      } else {
        report["valid"]=false;report["violations"].append(row);
      }
    }
    return report;
  }
  Checker(moveit::core::RobotModelPtr m,const Json::Value& r):model(m),scene(m),state(m) {
    // New requests explicitly select the observed-obstacle policy. Preserve the
    // meaning of historical requests without this field when replayed offline.
    collision_policy=r.get("collision_policy",r.get("occupied_only_preview",false).asBool()?
        "observed_obstacles":"observed_free_space").asString();
    if(collision_policy!="observed_obstacles" && collision_policy!="observed_free_space")
      throw std::runtime_error("Unknown collision policy");
    if(collision_policy=="observed_free_space" && r.get("occupied_only_preview",false).asBool())
      throw std::runtime_error("Conflicting collision policies");
    group=model->getJointModelGroup("right_arm");
    if (!group || group->getVariableCount()!=7) throw std::runtime_error("Missing seven-joint right_arm group");
    state.setToDefaultValues();
    auto left=vector7(r["left"]);
    for (size_t i=0;i<7;i++) state.setVariablePosition("openarmx_left_joint"+std::to_string(i+1),left[i]);
    state.setVariablePosition("openarmx_left_finger_joint1",r["left_opening_m"].asDouble());
    state.setJointGroupPositions(group,vector7(r["start"])); state.update();
    initial_bounds=positionBounds();
    // A bounds failure is a normal blocked result with joint-level evidence,
    // not a construction exception that loses the result schema/diagnostics.
    if(!initial_bounds["valid"].asBool())return;
    const auto& sc=r["scene"];
    // Legacy requests retain their original 5 mm self-mask. New requests use
    // an explicit bounded allowance for the thin stick's depth returns plus
    // voxel-center quantization. This affects ONLY the initial-pose perception
    // mask; every candidate still uses the complete stick collision body.
    double stick_mask=r.get("drumstick_self_filter_padding_m",.005).asDouble();
    if(!std::isfinite(stick_mask)||stick_mask<.005||stick_mask>.029)
      throw std::runtime_error("Invalid drumstick self-filter padding; require 5-29 mm");
    self_filter["body_padding_m"]=.005;self_filter["drumstick_padding_m"]=stick_mask;
    self_filter["initial_pose_only"]=true;self_filter["swept_path_masking"]=false;
    self_filter["collision_geometry_changed"]=false;
    bool filter_shadow=r.get("filter_drumstick_depth_shadow",false).asBool();
    self_filter["nominal_stick_depth_shadow_filter"]=filter_shadow;
    self_filter["shadow_points_are_unobserved_not_proven_free"]=true;
    self_filter["shadow_voxels"]=Json::UInt64(0);
    self_filter["ambiguous_near_stick_points_excluded"]=stick_mask>.005;
    self_clearance=r.get("minimum_self_clearance_m",0.).asDouble();
    if (!std::isfinite(self_clearance) || self_clearance<0 || self_clearance>.10)
      throw std::runtime_error("Invalid self clearance requirement");
    padding=sc["obstacle_padding_m"].asDouble();
    if (!std::isfinite(padding) || padding<.02 || padding>.10) throw std::runtime_error("Obstacle padding must be 20-100 mm");
    // World padding includes point voxelization and finite edge sampling. Self
    // collision uses exact nominal bodies; attachments remain whole-body links.
    scene.getCollisionEnvNonConst()->setPadding(padding + .009 + .005);
    if (sc.get("synthetic",false).asBool()) {
      for (const auto& box:sc["boxes"]) {
        if (box["size"].size()!=3 || box["xyz"].size()!=3) throw std::runtime_error("Bad synthetic box");
        Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();
        for(int j=0;j<3;j++) pose.translation()[j]=box["xyz"][j].asDouble();
        shapes::ShapeConstPtr shape(new shapes::Box(box["size"][0].asDouble(),box["size"][1].asDouble(),box["size"][2].asDouble()));
        scene.getWorldNonConst()->addToObject(box["id"].asString(),shape,pose);
      }
    } else {
      auto points=binary<float>(sc["points_file"].asString());
      if (points.empty() || points.size()%3) throw std::runtime_error("No usable camera obstacle points");
      auto tree=std::make_shared<octomap::OcTree>(.01);
      std::vector<std::pair<std::string,bodies::BodyPtr>> robot;
      bodies::BodyPtr nominal_stick;
      Eigen::Vector3d camera_origin=Eigen::Vector3d::Zero();
      if(filter_shadow) {
        const auto& transform=sc["T_base_depth"];
        if(transform.size()!=4)throw std::runtime_error("Depth shadow filter requires camera transform");
        for(int i=0;i<3;i++) {
          if(transform[i].size()!=4 || !transform[i][3].isNumeric())
            throw std::runtime_error("Invalid depth shadow camera origin");
          camera_origin[i]=transform[i][3].asDouble();
        }
        if(!camera_origin.allFinite())throw std::runtime_error("Nonfinite camera origin");
      }
      for (auto link:model->getLinkModelsWithCollisionGeometry()) {
        for (size_t s=0;s<link->getShapes().size();s++) {
          bodies::BodyPtr body(bodies::createBodyFromShape(link->getShapes()[s].get()));
          body->setPose(state.getCollisionBodyTransform(link,s));
          body->setPadding(link->getName()=="right_drumstick"?stick_mask:.005);
          robot.emplace_back(link->getName(),body);
          if(filter_shadow && link->getName()=="right_drumstick") {
            nominal_stick.reset(bodies::createBodyFromShape(link->getShapes()[s].get()));
            nominal_stick->setPose(state.getCollisionBodyTransform(link,s));
            nominal_stick->setPadding(0.);  // NO dilation of the shadow silhouette
          }
        }
      }
      // Only remove points in the explicitly padded START robot geometry, not
      // arbitrary nearest-neighbor bands around a trajectory or obstacles.
      for(size_t i=0;i<points.size();i+=3) {
        Eigen::Vector3d p(points[i],points[i+1],points[i+2]);
        if (!p.allFinite()) throw std::runtime_error("Nonfinite camera point");
        bool own=false;
        for(auto& body:robot) { if(body.second->containsPoint(p)) {
          own=true;
          auto& count=self_filter["removed_voxels_per_link"][body.first];
          count=Json::UInt64(count.asUInt64()+1);break;
        } }
        if(!own && nominal_stick) {
          // A return behind the known opaque stick on the SAME camera ray is
          // self-occluded depth, not a separately visible obstacle. This is an
          // initial-pose sensor shadow, never a swept motion-space exemption.
          // Use the NOMINAL (unexpanded) stick silhouette; preserve foreground
          // returns and every neighboring ray that does not cross the stick.
          Eigen::Vector3d ray=p-camera_origin;double range=ray.norm();
          if(range>1e-9) {
            ray/=range;EigenSTL::vector_Vector3d intersections;
            if(nominal_stick->intersectsRay(camera_origin,ray,&intersections)) {
              for(const auto& hit:intersections) {
                double distance=(hit-camera_origin).dot(ray);
                if(distance>0. && distance<range-1e-6) {
                  own=true;
                  self_filter["shadow_voxels"]=Json::UInt64(self_filter["shadow_voxels"].asUInt64()+1);
                  break;
                }
              }
            }
          }
        }
        if(own) {masked++;continue;}
        tree->updateNode(octomap::point3d(p.x(),p.y(),p.z()),true,true);
      }
      tree->updateInnerOccupancy();
      if (tree->size()==0) throw std::runtime_error("No obstacles remain after self-mask; refuse empty live scene");
      scene.getWorldNonConst()->addToObject("gemini_native_depth",shapes::ShapeConstPtr(new shapes::OcTree(tree)),Eigen::Isometry3d::Identity());
      // FCL still checks every moving body (including the wrist, custom claw and
      // stick) against all retained occupied voxels. Only the separate demand
      // for camera-proven free space is omitted in observed-obstacle mode.
      strict=collision_policy=="observed_free_space";
      if(strict) {
        depth=binary<float>(sc["free_depth_file"].asString());
        if(depth.size()!=1280*800) throw std::runtime_error("Bad full-FOV known-space image");
        auto K=sc["camera"]["depth"]["K"];
        fx=K[0][0].asDouble();fy=K[1][1].asDouble();cx=K[0][2].asDouble();cy=K[1][2].asDouble();
        Eigen::Isometry3d T=Eigen::Isometry3d::Identity();
        for(int i=0;i<4;i++) for(int j=0;j<4;j++) T.matrix()(i,j)=sc["T_base_depth"][i][j].asDouble();
        camera_from_base=T.inverse();
      }
    }
    // Cover every moving collision shape by small enclosing spheres. This is a
    // conservative visibility test: missing returns/occlusion/outside FOV reject.
    if(strict) for(auto link:group->getUpdatedLinkModelsWithGeometry()) {
      for(size_t s=0;s<link->getShapes().size();s++) {
        auto shape=link->getShapes()[s].get();
        Eigen::Vector3d size=shapes::computeShapeExtents(shape),center=Eigen::Vector3d::Zero();
        if(shape->type==shapes::MESH) {
          auto mesh=static_cast<const shapes::Mesh*>(shape);
          Eigen::Vector3d low=Eigen::Vector3d::Constant(INFINITY),high=-low;
          for(unsigned int k=0;k<mesh->vertex_count;k++) {
            Eigen::Vector3d p(mesh->vertices[k*3],mesh->vertices[k*3+1],mesh->vertices[k*3+2]);
            low=low.cwiseMin(p);high=high.cwiseMax(p);
          }
          center=(low+high)*.5;size=high-low;
        }
        Eigen::Array3i n=(size.array()/.05).ceil().cast<int>().max(1);
        Eigen::Vector3d step=size.cwiseQuotient(n.cast<double>().matrix());
        for(int x=0;x<n[0];x++) for(int y=0;y<n[1];y++) for(int z=0;z<n[2];z++) {
          Eigen::Vector3d c=center-size*.5+step.cwiseProduct(Eigen::Vector3d(x+.5,y+.5,z+.5));
          probes.push_back({link,s,c,step.norm()*.5+.005});
        }
      }
    }
  }
  bool knownFree(const Probe& p) {
    Eigen::Vector3d c=camera_from_base*(state.getCollisionBodyTransform(p.link,p.shape)*p.center);
    double radius=p.radius+padding;
    if(c.z()-radius<.15) {
      last["visibility"]["reason"]="body_or_clearance_buffer_too_near_or_behind_camera";
      last["visibility"]["nearest_camera_z_m"]=c.z()-radius;return false;
    }
    double u0=INFINITY,u1=-INFINITY,v0=INFINITY,v1=-INFINITY;
    for(int a:{-1,1}) for(int b:{-1,1}) for(int d:{-1,1}) {
      Eigen::Vector3d q=c+radius*Eigen::Vector3d(a,b,d);
      double u=fx*q.x()/q.z()+cx,v=fy*q.y()/q.z()+cy;
      u0=std::min(u0,u);u1=std::max(u1,u);v0=std::min(v0,v);v1=std::max(v1,v);
    }
    int x0=std::floor(u0),x1=std::ceil(u1),y0=std::floor(v0),y1=std::ceil(v1);
    if(x0<0||y0<0||x1>=1280||y1>=800) {
      last["visibility"]["reason"]="body_or_clearance_buffer_outside_depth_fov";
      for(int value:{x0,y0,x1,y1})last["visibility"]["pixel_bounds_xyxy"].append(value);
      return false;
    }
    for(int y=y0;y<=y1;y++) for(int x=x0;x<=x1;x++) {
      float observed=depth[y*1280+x];
      if(!std::isfinite(observed) || observed<=c.z()+radius) {
        last["visibility"]["reason"]=(!std::isfinite(observed)||observed<=0)?
          "missing_or_invalid_depth":"observed_surface_occludes_required_clearance";
        last["visibility"]["pixel_xy"].append(x);last["visibility"]["pixel_xy"].append(y);
        if(std::isfinite(observed))last["visibility"]["observed_depth_m"]=observed;
        last["visibility"]["required_clear_depth_m"]=c.z()+radius;return false;
      }
    }
    return true;
  }
  Json::Value visibilityReport(const Q& q) {
    // Diagnostic only: inspect ALL moving links, rather than making the first
    // failed shoulder probe look like the only problem. Never changes validity,
    // depth, geometry, padding, collision exemptions or the unknown-space policy.
    Q previous;state.copyJointGroupPositions(group,previous);
    auto saved_last=last;
    state.setJointGroupPositions(group,q);state.update();
    std::map<std::string,Json::Value> links;
    for(const auto& probe:probes) {
      auto& row=links[probe.link->getName()];row["link"]=probe.link->getName();
      row["probe_count"]=row.get("probe_count",0).asUInt()+1;
      last=Json::Value(Json::objectValue);
      if(!knownFree(probe)) {
        row["blocked_probe_count"]=row.get("blocked_probe_count",0).asUInt()+1;
        std::string reason=last["visibility"]["reason"].asString();
        row["failure_counts"][reason]=row["failure_counts"].get(reason,0).asUInt()+1;
        if(!row["examples"].isMember(reason))row["examples"][reason]=last["visibility"];
      }
    }
    // For the actual planner's mesh/cylinder shapes, also give nominal surface
    // z extrema with NO buffer. This distinguishes genuine near-range geometry
    // from conservatism in the enclosing-sphere visibility cover.
    for(auto link:group->getUpdatedLinkModelsWithGeometry()) {
      double low=INFINITY,high=-INFINITY;
      for(size_t s=0;s<link->getShapes().size();s++) {
        auto shape=link->getShapes()[s].get();
        Eigen::Isometry3d t=camera_from_base*state.getCollisionBodyTransform(link,s);
        if(shape->type==shapes::MESH) {
          auto mesh=static_cast<const shapes::Mesh*>(shape);
          for(unsigned int k=0;k<mesh->vertex_count;k++) {
            Eigen::Vector3d p(mesh->vertices[k*3],mesh->vertices[k*3+1],mesh->vertices[k*3+2]);
            double z=(t*p).z();low=std::min(low,z);high=std::max(high,z);
          }
        } else if(shape->type==shapes::CYLINDER) {
          auto cylinder=static_cast<const shapes::Cylinder*>(shape);
          double extent=cylinder->radius*std::hypot(t(2,0),t(2,1))+
                        cylinder->length*.5*std::abs(t(2,2));
          low=std::min(low,t.translation().z()-extent);high=std::max(high,t.translation().z()+extent);
        }
      }
      if(std::isfinite(low) && std::isfinite(high)) {
        links[link->getName()]["nominal_geometry_camera_z_min_m"]=low;
        links[link->getName()]["nominal_geometry_camera_z_max_m"]=high;
      }
    }
    Json::Value out;out["diagnostic_only"]=true;out["collision_checked"]=false;
    out["accepted_near_depth_m"]=.15;out["depth_image_size_xy"].append(1280);out["depth_image_size_xy"].append(800);
    out["unknown_space_policy"]="blocked; no exemptions or clearance changes";
    out["links"]=Json::Value(Json::arrayValue);unsigned int blocked=0;
    for(auto& pair:links) {
      auto& row=pair.second;
      row["blocked_probe_count"]=row.get("blocked_probe_count",0).asUInt();
      row["coverage_complete"]=row["blocked_probe_count"].asUInt()==0;
      if(!row["coverage_complete"].asBool())blocked++;
      out["links"].append(row);
    }
    out["moving_links_checked"]=Json::UInt(links.size());out["links_with_unobserved_clearance"]=blocked;
    state.setJointGroupPositions(group,previous);state.update();last=saved_last;
    return out;
  }
  bool valid(const Q& q,bool details=false) {
    checks++; last=Json::Value(Json::objectValue);
    state.setJointGroupPositions(group,q);state.update();
    auto bounds=positionBounds();
    if(!bounds["valid"].asBool()) {last=bounds;return false;}
    collision_detection::CollisionRequest req; collision_detection::CollisionResult res;
    req.group_name="right_arm";req.contacts=details;req.max_contacts=20;req.max_contacts_per_pair=1;
    scene.checkCollision(req,res,state);
    if(res.collision) {
      last["reason"]="collision";
      for(auto& pair:res.contacts) { Json::Value a(Json::arrayValue);a.append(pair.first.first);a.append(pair.first.second);last["contacts"].append(a); }
      return false;
    }
    if(self_clearance>0) {
      collision_detection::CollisionRequest near_req;
      collision_detection::CollisionResult near_res;
      near_req.group_name="right_arm";near_req.distance=true;
      scene.checkSelfCollision(near_req,near_res,state);
      if(near_res.collision || near_res.distance<self_clearance) {
        last["reason"]="insufficient_self_clearance";
        last["distance_m"]=near_res.distance;return false;
      }
    }
    if(strict) for(auto& probe:probes) if(!knownFree(probe)) {
      last["reason"]="unknown_or_occluded_space";last["link"]=probe.link->getName();return false;
    }
    last["reason"]="valid";return true;
  }
  bool edge(const Q& a,const Q& b) {
    double d=0;for(size_t i=0;i<7;i++)d+=std::abs(b[i]-a[i]);
    size_t n=std::max(size_t(1),size_t(std::ceil(d/EDGE_L1)));
    for(size_t k=1;k<=n;k++) {Q q(7);for(size_t i=0;i<7;i++)q[i]=a[i]+(b[i]-a[i])*double(k)/n;if(!valid(q))return false;}
    return true;
  }
};

struct Edges:ob::MotionValidator {
  Checker& checker;
  Edges(const ob::SpaceInformationPtr& si,Checker& c):ob::MotionValidator(si),checker(c) {}
  bool checkMotion(const ob::State* a,const ob::State* b) const override {return checker.edge(fromState(a),fromState(b));}
  bool checkMotion(const ob::State* a,const ob::State* b,std::pair<ob::State*,double>& lastValid) const override {
    if(checkMotion(a,b)) return true;
    lastValid.second=0.;if(lastValid.first)si_->copyState(lastValid.first,a);return false;
  }
};

// SOFT route scoring only. This runs in a separate backend invocation from
// hard validation: temporary padding probes NEVER authorize a path or change
// its obstacle map, collision geometry, ACM, or validation margins.
Json::Value styleFeatures(Checker& checker,const Json::Value& paths) {
  if(!paths.isArray() || paths.empty() || paths.size()>64)
    throw std::runtime_error("Expected 1-64 style paths");
  const std::set<std::string> distal={"openarmx_right_link5","openarmx_right_link6",
      "openarmx_right_link7","openarmx_right_hand","openarmx_right_left_finger",
      "openarmx_right_right_finger","right_drumstick"};
  auto cost_acm=checker.scene.getAllowedCollisionMatrix();
  const auto world_ids=checker.scene.getWorld()->getObjectIds();
  for(auto link:checker.model->getLinkModelsWithCollisionGeometry())
    if(!distal.count(link->getName()))for(const auto& id:world_ids)cost_acm.setEntry(link->getName(),id,true);
  struct Sample {moveit::core::RobotState state;size_t path;size_t index;bool found=false;};
  std::vector<Sample> samples;
  Json::Value output(Json::arrayValue);
  auto stick=checker.model->getLinkModel("right_drumstick");
  if(!stick || stick->getShapes().size()!=1 || stick->getShapes()[0]->type!=shapes::CYLINDER)
    throw std::runtime_error("Style scoring requires the actual stick collision cylinder");
  double half=static_cast<const shapes::Cylinder*>(stick->getShapes()[0].get())->length*.5;
  for(size_t k=0;k<paths.size();k++) {
    const auto& points=paths[Json::ArrayIndex(k)];
    if(!points.isArray() || points.size()!=801)throw std::runtime_error("Style path requires 801 joint-arc samples");
    Json::Value row;row["joint_bounds_valid"]=true;
    for(size_t i=0;i<points.size();i++) {
      checker.state.setJointGroupPositions(checker.group,vector7(points[Json::ArrayIndex(i)]));checker.state.update();
      auto bounds=checker.positionBounds();
      if(!bounds["valid"].asBool()) {row["joint_bounds_valid"]=false;row["bounds_failure"]=bounds;}
      Eigen::Vector3d wrist=checker.state.getGlobalLinkTransform("openarmx_right_hand").translation();
      Eigen::Vector3d tip=checker.state.getCollisionBodyTransform(stick,0)*Eigen::Vector3d(0,0,half);
      row["wrist_xyz_m"].append(jsonQ(Q{wrist.x(),wrist.y(),wrist.z()}));
      row["tip_xyz_m"].append(jsonQ(Q{tip.x(),tip.y(),tip.z()}));
      if(i%4==0) {
        row["distal_clearance_estimate_m"].append(.10);
        samples.push_back({checker.state,k,i/4,false});
      }
    }
    output.append(row);
  }
  if(world_ids.empty())return output;
  collision_detection::CollisionRequest query;query.group_name="right_arm";
  // Estimate the same 0-100 mm comfort distance using cheap, cached FCL
  // overlap probes, not expensive exact mesh/OctoMap distance queries. Each
  // positive estimate is the midpoint of a 5 mm padding band. It is a SOFT
  // padding-distance estimate, not certified Euclidean clearance. World-only:
  // full-arm and self-collision requirements remain in the hard check call.
  auto env=checker.scene.getCollisionEnvNonConst();
  const auto original_padding=env->getLinkPadding();
  for(int band=0;band<=20;band++) {
    double padding=.005*band;std::map<std::string,double> pads;
    for(const auto& name:distal)pads[name]=padding;
    env->setLinkPadding(pads);
    for(auto& sample:samples) {
      if(sample.found || !output[Json::ArrayIndex(sample.path)]["joint_bounds_valid"].asBool())continue;
      collision_detection::CollisionResult collision;
      env->checkRobotCollision(query,collision,sample.state,cost_acm);
      if(collision.collision) {
        output[Json::ArrayIndex(sample.path)]["distal_clearance_estimate_m"][Json::ArrayIndex(sample.index)]=
            band==0?0.:padding-.0025;
        sample.found=true;
      }
    }
  }
  env->setLinkPadding(original_padding);
  return output;
}

Json::Value run(const Json::Value& r) {
  if(r["schema"]!="x5-moveit-request-v1" || r.get("execute",false).asBool()) throw std::runtime_error("Software-only planning request required");
  auto u=urdf::parseURDF(textfile(r["urdf"].asString()));if(!u)throw std::runtime_error("Invalid URDF");
  auto srdf=std::make_shared<srdf::Model>();if(!srdf->initString(*u,textfile(r["srdf"].asString())))throw std::runtime_error("Invalid SRDF");
  auto model=std::make_shared<moveit::core::RobotModel>(u,srdf);
  Checker checker(model,r);Q start=vector7(r["start"]),goal=vector7(r["goal"]);
  Json::Value out;out["schema"]="x5-moveit-result-v1";out["motion_authorized"]=false;
  out["backend"]="MoveIt PlanningScene/FCL + OMPL RRTConnect + TOTG seed";
  out["collision_policy"]=checker.collision_policy;
  out["strict_unknown_space"]=checker.strict;
  out["unobserved_space_blocks_motion"]=checker.strict;
  out["joint_limit_check"]=checker.initial_bounds;
  if(!checker.initial_bounds["valid"].asBool()) {
    out["status"]="blocked_start";out["details"]=checker.initial_bounds;return out;
  }
  out["self_filtered_voxels"]=Json::UInt64(checker.masked);
  out["robot_self_filter"]=checker.self_filter;
  out["collision_edge_l1_step_rad"]=EDGE_L1;
  if(r.isMember("style_paths")) {
    if(r.isMember("check_states"))throw std::runtime_error("Style scoring and hard validation must be separate");
    out["style_features"]=styleFeatures(checker,r["style_paths"]);
    out["status"]="scored";out["hard_collision_checked"]=false;
    out["style_clearance_method"]="distal world-only FCL padding bands; 5 mm bands, saturated at 100 mm";
    return out;
  }
  if(r.isMember("check_states")) {
    Q previous;
    for(auto& value:r["check_states"]) {
      Q q=vector7(value);
      bool ok=checker.valid(q,true);
      if(ok && r.get("check_path",false).asBool() && !previous.empty()) ok=checker.edge(previous,q);
      auto row=checker.last;row["valid"]=ok;previous=q;
      Eigen::Vector3d tool=checker.state.getGlobalLinkTransform("right_drumstick").translation();
      row["stick_center_world"]=jsonQ(Q{tool.x(),tool.y(),tool.z()});out["checks"].append(row);
      // Best-of-N ranking only needs the first failure to reject a candidate.
      // Successful candidates still require every state and edge. Diagnostic
      // callers retain the historical full report unless explicitly requested.
      if(!ok && r.get("stop_on_invalid",false).asBool())break;
    }
    out["status"]="checked";return out;
  }
  auto blocked=[&](const std::string& status) {
    out["status"]=status;out["details"]=checker.last;
    if(checker.strict) {
      out["visibility_coverage"]["start"]=checker.visibilityReport(start);
      out["visibility_coverage"]["goal"]=checker.visibilityReport(goal);
    }
    return out;
  };
  if(!checker.valid(start,true))return blocked("blocked_start");
  if(!checker.valid(goal,true))return blocked("blocked_goal");
  ompl::RNG::setSeed(r.get("seed",19).asUInt());
  auto space=std::make_shared<ob::RealVectorStateSpace>(7);ob::RealVectorBounds bounds(7);
  const auto& names=checker.group->getVariableNames();
  for(size_t i=0;i<7;i++){auto b=model->getVariableBounds(names[i]);bounds.setLow(i,b.min_position_);bounds.setHigh(i,b.max_position_);}
  space->setBounds(bounds);og::SimpleSetup setup(space);
  setup.setStateValidityChecker([&](const ob::State* s){return checker.valid(fromState(s));});
  setup.getSpaceInformation()->setMotionValidator(std::make_shared<Edges>(setup.getSpaceInformation(),checker));
  auto planner=std::make_shared<og::RRTConnect>(setup.getSpaceInformation());planner->setRange(.2);setup.setPlanner(planner);
  ob::ScopedState<> a(space),b(space);for(size_t i=0;i<7;i++){a[i]=start[i];b[i]=goal[i];}
  setup.setStartAndGoalStates(a,b,1e-9);
  double seconds=r.get("planning_seconds",8.).asDouble();
  if(!std::isfinite(seconds)||seconds<=0||seconds>60)throw std::runtime_error("Invalid planning timeout");
  auto status=setup.solve(seconds);
  if(status!=ob::PlannerStatus::EXACT_SOLUTION){out["status"]="no_path";return out;}
  setup.simplifySolution(.15);
  auto path=setup.getSolutionPath();
  // Explicit endpoint restoration is allowed only with collision rechecking;
  // no goal clipping or approximate-goal acceptance.
  robot_trajectory::RobotTrajectory trajectory(model,"right_arm");
  for(size_t i=0;i<path.getStateCount();i++) {
    Q q=fromState(path.getState(i));if(i==0)q=start;if(i+1==path.getStateCount())q=goal;
    out["path_waypoints"].append(jsonQ(q));
    checker.state.setJointGroupPositions(checker.group,q);checker.state.update();
    trajectory.addSuffixWayPoint(checker.state,0.);
  }
  std::unordered_map<std::string,double> velocity,acceleration;
  for(auto& name:names){velocity[name]=.6;acceleration[name]=1.2;}
  trajectory_processing::TimeOptimalTrajectoryGeneration timing(.002,.01,.0001);
  if(!timing.computeTimeStamps(trajectory,velocity,acceleration)) {
    // A poor path from one random seed must not cancel other candidate routes.
    out["status"]="failed_time_parameterization";out["details"]="Time parameterization failed";return out;
  }
  Q previous=start;
  for(size_t k=0;k<trajectory.getWayPointCount();k++) {
    auto& s=trajectory.getWayPoint(k);Q q,v,acc;
    s.copyJointGroupPositions(checker.group,q);s.copyJointGroupVelocities(checker.group,v);s.copyJointGroupAccelerations(checker.group,acc);
    if(!checker.valid(q,true)||!checker.edge(previous,q)) {
      out["status"]="blocked_after_smoothing";out["details"]=checker.last;return out;
    }
    Json::Value row;row["time_s"]=trajectory.getWayPointDurationFromStart(k);
    row["positions_rad"]=jsonQ(q);row["velocities_rad_s"]=jsonQ(v);row["accelerations_rad_s2"]=jsonQ(acc);
    out["outward"].append(row);previous=q;
  }
  if((Eigen::Map<Eigen::VectorXd>(previous.data(),7)-Eigen::Map<Eigen::VectorXd>(goal.data(),7)).cwiseAbs().maxCoeff()>1e-9) {
    out["status"]="invalid_smoothed_goal";out["details"]="Smoothed goal changed";return out;
  }
  out["status"]="planned";out["state_validity_checks"]=Json::UInt64(checker.checks);
  out["duration_outward_s"]=trajectory.getDuration();out["hold_at_goal_s"]=2.;
  out["velocity_limit_rad_s"]=.6;out["acceleration_limit_rad_s2"]=1.2;out["jerk_limit_rad_s3"]=6.;
  out["warning"]="Software preview only. Sampled collision checks are not physical collision certification. No controllers are connected.";
  return out;
}
int main(int argc,char** argv) {
  if(argc!=3){std::cerr<<"usage: moveit_plan request.json result.json\n";return 2;}
  // MoveIt 2.12 creates a logger node even when used as a standalone library.
  // Initialize it cleanly, isolated from the workspace's control domain 92.
  setenv("ROS_DOMAIN_ID","197",1);
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE","LOCALHOST",1);
  rclcpp::init(argc,argv);
  Json::Value out;
  // Even parsing/model-construction exceptions must be recognizable reports.
  out["schema"]="x5-moveit-result-v1";
  out["motion_authorized"]=false;out["hardware_execution_available"]=false;
  try{std::ifstream f(argv[1]);Json::Value r;if(!f||!(f>>r))throw std::runtime_error("Cannot parse request");out=run(r);}
  catch(const std::exception& e){out["status"]="error";out["error"]=e.what();out["motion_authorized"]=false;}
  std::ofstream f(argv[2]);if(!f){std::cerr<<"Cannot write result\n";return 2;}
  Json::StreamWriterBuilder builder;builder["indentation"]="  ";f<<Json::writeString(builder,out)<<'\n';
  rclcpp::shutdown();
  return out["status"]=="planned"||out["status"]=="checked"||out["status"]=="scored"?0:1;
}
