#ifndef DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
#define DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H

#include "../bits/types.h"
#include "../bits/concepts.h"
#include "../mechanics/sphere_vs_fixed_plane_detection.h"
#include "../mechanics/sphere_vs_fixed_plane_response.h"
#include "../mechanics/sphere_vs_sphere_detection.h"
#include "../mechanics/sphere_vs_sphere_response.h"
#include "../mechanics/compute_trajectory.h"
#include <set>
#include <vector>
#include <algorithm>


namespace dte3607::physengine::solver_dev::level3
{

  struct Params {
    types::Vector3 const          F;                // External forces (environment)
    types::HighResolutionTP const t_0;              // Start of simulation
    types::Duration const         dt;               // Timestep, duration of simulation
  };



  struct SphereGeomDataBlock {
    types::Point3                 p;                // Position
    types::ValueType const        r;                // Radius
    types::Vector3                v;                // Velocity
    types::ValueType const        f;                // Friction coefficent
    types::ValueType const        m;                // Mass
    types::RBState                s;                // State
    types::HighResolutionTP       t_c;
  };
  using SphereGeomData = std::vector<SphereGeomDataBlock>;



  struct InfPlaneGeomDataBlock {
    types::Point3 const           p;                // Position
    types::Vector3 const          n;                // Normal vector
    types::ValueType const        f;                // Friction coefficent
  };
  using InfPlaneGeomData = std::vector<InfPlaneGeomDataBlock>;



  struct IntersectDetProcDataBlock {
    size_t                        sphere1_id;
    std::optional<size_t>         sphere2_id;
    std::optional<size_t>         plane_id;
    types::HighResolutionTP       col_tp;
  };
  using IntersectDetProcData = std::vector<IntersectDetProcDataBlock>;



  struct AttachedDataBlock {
    size_t                        sphere_id;
    size_t                        plane_id;
  };
  using AttachedData = std::vector<AttachedDataBlock>;



  // Modify vector for sphere attached to a plane (Sliding)
  types::Vector3 vectorAlongPlane(
    types::Vector3 const& vec,
    types::Vector3 const& p_normal)
  {

    auto n = blaze::normalize(p_normal);
    auto vec_n = blaze::inner(vec, n) * n;

    return vec - vec_n;
  }



  std::pair<types::Vector3, types::Vector3> slidingSphereTrajectory(
    Params&              params,
    SphereGeomDataBlock& sphere,
    InfPlaneGeomData&    planes,
    AttachedDataBlock&   attached)
  {

    auto traj = mechanics::computeLinearTrajectory(
      sphere.v,
      params.F,
      (params.t_0 + params.dt) - sphere.t_c
      );

    size_t p_id = attached.plane_id;

    return {
      vectorAlongPlane(traj.first, planes[p_id].n),
      vectorAlongPlane(traj.second, planes[p_id].n)
    };
  }



  AttachedDataBlock findAttached(AttachedData& attached, size_t s_id) {
    return *std::find_if(attached.begin(), attached.end(), [s_id](auto& a){return a.sphere_id == s_id;});
  }



  types::Vector3 findAttachedPlaneNormal(
    InfPlaneGeomData&     planes,
    AttachedData&         attached,
    size_t                s_id)
  {

    auto s_attached = *std::find_if(attached.begin(), attached.end(), [s_id](auto& a){return a.sphere_id == s_id;});
    size_t p_id = s_attached.plane_id;

    return blaze::normalize(planes[p_id].n);
  }



  void detectCollision(
    Params&               params,
    IntersectDetProcData& intersections,
    SphereGeomData&       spheres,
    InfPlaneGeomData&     planes,
    AttachedData&         attached,
    size_t                cur_s_id,
    std::set<size_t>&     exclude_sphere_idx,
    std::set<size_t>&     exclude_plane_idx)
  {

    IntersectDetProcData collisions;
    auto sphere = spheres[cur_s_id];

    if (sphere.s == types::RBState::Resting) return;

    // If sphere is attached(Sliding)
    std::optional<AttachedDataBlock> s1_attached;
    std::optional<types::Vector3> ds;

    if (sphere.s == types::RBState::Sliding) {
      s1_attached = *std::find_if(attached.begin(), attached.end(), [cur_s_id](auto& a){return a.sphere_id == cur_s_id;});
      ds = slidingSphereTrajectory(params, sphere, planes, s1_attached.value()).first;
    }

    // Collision with other planes
    for (size_t p_id=0; p_id < planes.size(); p_id++) {
      if (exclude_plane_idx.contains(p_id)) continue;

      auto plane = planes[p_id];
      auto tc = mechanics::detectCollisionSphereFixedPlane(
        sphere.t_c,
        sphere.p,
        sphere.r,
        sphere.v,
        plane.p,
        plane.n,
        params.F,
        params.t_0,
        params.dt,
        ds
        );

      if (tc.has_value()) {               // Add possible collision
        intersections.push_back({
          cur_s_id,
          std::nullopt,
          p_id,
          tc.value()
        });
      }
    }

    // Collision with other spheres
    for (size_t s_id=0; s_id < spheres.size(); s_id++ ) {
      if (exclude_sphere_idx.contains(s_id)) continue;

      auto sphere2 = spheres[s_id];
      std::optional<AttachedDataBlock> s2_attached;

      types::HighResolutionTP t2 = sphere.t_c;
      types::HighResolutionTP t1 = sphere2.t_c;
      types::HighResolutionTP t_min = std::max(t1, t2);   // Starting timestamp to detect collision between spheres.

      // 'Move' spheres forward until tc = t_min. Temporary, not actual simulation
      auto [ds_1, dv_1] = mechanics::computeLinearTrajectory(
        sphere.v,
        params.F,
        t_min - sphere.t_c);

      if (sphere.s == types::RBState::Sliding) {
        size_t p_id = s1_attached.value().plane_id;
        ds_1 = vectorAlongPlane(ds_1, planes[p_id].n);
        dv_1 = vectorAlongPlane(dv_1, planes[p_id].n);
      }

      auto [ds_2, dv_2] = mechanics::computeLinearTrajectory(
        sphere2.v,
        params.F,
        t_min - sphere2.t_c);

      if (sphere2.s == types::RBState::Sliding) {
        s2_attached = *std::find_if(attached.begin(), attached.end(), [s_id](auto& a){return a.sphere_id == s_id;});
        size_t p_id = s2_attached.value().plane_id;

        ds_2 = vectorAlongPlane(ds_2, planes[p_id].n);
        dv_2 = vectorAlongPlane(dv_2, planes[p_id].n);
      }

      auto new_s1_p = sphere.p + ds_1;
      auto new_s1_v = sphere.v + dv_1;
      auto new_s2_p = sphere2.p + ds_2;
      auto new_s2_v = sphere2.v + dv_2;

      // Find new ds_1 & ds_2, if Sliding
      std::optional<types::Vector3> new_ds_1;
      std::optional<types::Vector3> new_ds_2;

      if (sphere.s == types::RBState::Sliding) {
        new_ds_1 = mechanics::computeLinearTrajectory(
          new_s1_v,
          params.F,
          (params.t_0 + params.dt) - t_min).first;
      }

      if (sphere2.s == types::RBState::Sliding) {
        new_ds_2 = mechanics::computeLinearTrajectory(
          new_s2_v,
          params.F,
          (params.t_0 + params.dt) - t_min).first;
      }

      auto tc = mechanics::detectCollisionSphereSphere(
        t_min,
        new_s1_p,
        sphere.r,
        new_s1_v,
        t_min,
        new_s2_p,
        sphere2.r,
        new_s2_v,
        params.F,
        params.t_0,
        params.dt,
        new_ds_1,
        new_ds_2
        );

      if (tc.has_value()) {               // Add possible collision
        intersections.push_back({
          cur_s_id,
          s_id,
          std::nullopt,
          tc.value()
        });
      }
    }
  }



  void detectAllCollisions(
    Params&               params,
    IntersectDetProcData& intersections,
    SphereGeomData&       spheres,
    InfPlaneGeomData&     planes,
    AttachedData&         attached)
  {

    for (size_t s_id=0; s_id < spheres.size(); s_id++) {
      std::set<size_t> exclude_sphere_idx{s_id};
      std::set<size_t> exclude_plane_idx;

      detectCollision(
        params,
        intersections,
        spheres,
        planes,
        attached,
        s_id,
        exclude_sphere_idx,
        exclude_plane_idx);

    }
  }



  void sortAndReduce(IntersectDetProcData& intersections) {

    std::ranges::sort(intersections, [](auto& intersection1, auto& intersection2){
      return intersection1.col_tp < intersection2.col_tp;
    });

    std::set<size_t> visited;
    std::erase_if(intersections, [&visited](auto& intersection){
      if (visited.contains(intersection.sphere1_id)) {
        return true;
      }
      else if (intersection.sphere2_id.has_value() && visited.contains(intersection.sphere2_id.value())) {
        return true;
      }
      else {
        visited.insert(intersection.sphere1_id);
        if (intersection.sphere2_id.has_value()) {
          visited.insert(intersection.sphere2_id.value());
        }
        return false;
      }
    });

    // Reverse intersections. Pop first collision from end of vector
    std::reverse(intersections.begin(), intersections.end());
  }



  void updateStateAndAttachment(
    SphereGeomData&       spheres,
    AttachedData&         attached,
    size_t                s_id,
    std::optional<size_t> p_id,
    types::RBState        new_state)
  {

    spheres[s_id].s = new_state;

    if (new_state == types::RBState::Free) { // Delete old attachment

      std::erase_if(attached, [s_id](auto& a){
        if (a.sphere_id == s_id) return true;
        else return false;
      });
      return;
    }

    auto it = std::find_if(attached.begin(), attached.end(), [s_id](auto& a){return a.sphere_id == s_id;});

    if (it != attached.end()){    // Update attachment
      auto& attach = *it;
      attach.plane_id = p_id.value();
    }
    else {                        // New attachment
      attached.push_back({
        s_id, p_id.value()
      });
    }
  }



  // State change after sphere vs plane collision
  std::optional<types::RBState>
  detectStateChangeOneSphere(
    Params&           params,
    SphereGeomData&   spheres,
    InfPlaneGeomData& planes,
    size_t            s_id,
    size_t            p_id)
  {

    types::RBState old_state = spheres[s_id].s;

    auto n = blaze::normalize(planes[p_id].n);

    types::Vector3 ds = mechanics::computeLinearTrajectory(
      spheres[s_id].v,
      params.F,
      (params.t_0 + params.dt) - spheres[s_id].t_c
      ).first;

    types::ValueType ds_n = blaze::inner(ds, n);   // size of ds normal to plane, along n
    types::Vector3   ds_p = ds - ds_n * n;         // ds parallell to plane
    auto             eps = 1e-2;

    // Free -> Sliding / Resting
    if (old_state == types::RBState::Free) {

      if (ds_n > 0) {
        return std::nullopt;
      }

      // Sphere is attached
      if (blaze::length(ds_p) > eps) {
        return types::RBState::Sliding;
      }
      else {
        return types::RBState::Resting;
      }
    }

    // Attached to Free / Sliding / Resting  ~~TODO~~
    std::optional<types::RBState> new_state;

    if (ds_n > 0) {
      return types::RBState::Free;
    }

    // Sphere stays attached
    if (blaze::length(ds_p) > eps) {
      new_state = types::RBState::Sliding;
    }
    else {
      new_state = types::RBState::Resting;
    }

    if (old_state == new_state) {
      return std::nullopt;
    }

    return new_state;
  }



  // State change after sphere vs sphere collision
  std::pair<std::optional<types::RBState>, std::optional<types::RBState>>
  detectStateChangeTwoSpheres(
    Params&           params,
    SphereGeomData&   spheres,
    InfPlaneGeomData& planes,
    AttachedData&     attached,
    size_t            s1_id,
    size_t            s2_id)
  {

    types::RBState old_state1 = spheres[s1_id].s;
    types::RBState old_state2 = spheres[s2_id].s;

    std::optional<types::RBState> new_state1;
    if (old_state1 == types::RBState::Free) {
      new_state1 = std::nullopt;
    }
    else {
      size_t p_id = findAttached(attached, s1_id).plane_id;

      new_state1 = detectStateChangeOneSphere(
        params, spheres, planes, s1_id, p_id);
    }

    std::optional<types::RBState> new_state2;
    if (old_state2 == types::RBState::Free) {
      new_state2 = std::nullopt;
    }
    else {
      size_t p_id = findAttached(attached, s2_id).plane_id;

      new_state2 = detectStateChangeOneSphere(
        params, spheres, planes, s2_id, p_id);
    }

    return {new_state1, new_state2};
  }



  void handleCollision(
    Params&               params,
    IntersectDetProcData& intersections,
    SphereGeomData&       spheres,
    InfPlaneGeomData&     planes,
    AttachedData&         attached)
  {

    auto collision = intersections.back();
    intersections.pop_back();

    size_t                           s1_id = collision.sphere1_id;
    std::optional<size_t>            s2_id = collision.sphere2_id;
    std::optional<size_t>            p_id = collision.plane_id;
    types::ValueType                 f; // friction (dampening in collision)
    bool                             two_sphere_col = s2_id.has_value();

    types::Vector3                   ds1;
    std::optional<types::Vector3>    ds2;
    types::Vector3                   dv1;
    std::optional<types::Vector3>    dv2;
    types::Vector3                   new_v1;
    std::optional<types::Vector3>    new_v2;

    std::optional<AttachedDataBlock> s1_attached;
    std::optional<AttachedDataBlock> s2_attached;

    std::set<size_t>                 exclude_sphere_idx{s1_id};
    std::set<size_t>                 exclude_plane_idx{};

    //  Simulate Object
    std::tie(ds1, dv1) = mechanics::computeLinearTrajectory(
      spheres[s1_id].v,
      params.F,
      collision.col_tp - spheres[s1_id].t_c
      );

    if (spheres[s1_id].s == types::RBState::Sliding) {
      s1_attached = findAttached(attached, s1_id);
      std::tie(ds1, dv1) = slidingSphereTrajectory(
        params, spheres[s1_id], planes, s1_attached.value());
    }

    if (two_sphere_col) {
      std::tie(ds2, dv2) = mechanics::computeLinearTrajectory(
        spheres[s2_id.value()].v,
        params.F,
        collision.col_tp - spheres[s2_id.value()].t_c
        );

      if (spheres[s2_id.value()].s == types::RBState::Sliding) {
        s2_attached = findAttached(attached, s2_id.value());
        std::tie(ds2, dv2) = slidingSphereTrajectory(
          params, spheres[s2_id.value()], planes, s2_attached.value());
      }
    }

    // ImpactResponse
    if (two_sphere_col) {
      auto s1 = spheres[s1_id];
      auto s2 = spheres[s2_id.value()];
      std::tie(new_v1, new_v2) = mechanics::computeImpactResponseSphereSphere(
        s1.p + ds1,
        s1.v + dv1,
        1.0,
        s2.p + ds2.value(),
        s2.v + dv2.value(),
        1.0);

      f = s1.f * s2.f;

      new_v1 *= (1 - f);
      new_v2.value() *= (1 - f);
    }
    else {
      new_v1 = mechanics::computeImpactResponseSphereFixedPlane(
        spheres[s1_id].v + dv1,
        planes[p_id.value()].n
        );

      f = spheres[s1_id].f * planes[p_id.value()].f;
      f = std::min(1., f);  // time adjusted total friction coefficient ???

      new_v1 *= (1 - f);
    }

    // UpdateCacheProperties
    spheres[s1_id].p += ds1;
    spheres[s1_id].v = new_v1;
    spheres[s1_id].t_c = collision.col_tp;

    if (two_sphere_col) {
      spheres[s2_id.value()].p += ds2.value();
      spheres[s2_id.value()].v = new_v2.value();
      spheres[s2_id.value()].t_c = collision.col_tp;
      exclude_sphere_idx.insert(s2_id.value());
    }
    else {
      exclude_plane_idx.insert(p_id.value());
    }



    // Detect change in state sphere vs plane
    if (!two_sphere_col) {
      auto state = detectStateChangeOneSphere(
        params,
        spheres,
        planes,
        s1_id,
        p_id.value());

      if (state.has_value()) {
        updateStateAndAttachment(
          spheres,
          attached,
          s1_id,
          p_id.value(),
          state.value());
      }
    }

    // Detect change in state sphere vs sphere
    if (two_sphere_col) {
      auto [state_1, state_2] = detectStateChangeTwoSpheres(
        params,
        spheres,
        planes,
        attached,
        s1_id,
        s2_id.value());

      if (state_1.has_value()) {
        std::optional<size_t> p1_id;

        if (!(state_1.value() == types::RBState::Free)) {
          p1_id = findAttached(attached, s1_id).plane_id;
        }

        updateStateAndAttachment(
          spheres,
          attached,
          s1_id,
          p1_id,
          state_1.value()
          );

      }
      if (state_2.has_value()) {
        std::optional<size_t> p2_id;

        if (!(state_2.value() == types::RBState::Free)) {
          p2_id = findAttached(attached, s2_id.value()).plane_id;
        }

        updateStateAndAttachment(
          spheres,
          attached,
          s2_id.value(),
          p2_id,
          state_2.value()
          );
      }
    }

    // Detect further collision. Add to collisions
    detectCollision(
      params,
      intersections,
      spheres,
      planes,
      attached,
      s1_id,
      exclude_sphere_idx,
      exclude_plane_idx);

    if (two_sphere_col) {
      detectCollision(
        params,
        intersections,
        spheres,
        planes,
        attached,
        s2_id.value(),
        exclude_sphere_idx,
        exclude_plane_idx);
    }
  }



  void simulateObjects(
    Params&           params,
    SphereGeomData&   spheres,
    InfPlaneGeomData& planes,
    AttachedData&     attached)
  {

    for (size_t s_id = 0; s_id<spheres.size(); s_id++) {

      if (spheres[s_id].s == types::RBState::Resting) continue;

      types::Duration timestep = (params.t_0 + params.dt) - spheres[s_id].t_c;
      // types::Duration timestep = params.dt - (spheres[i].t_c - params.t_0);

      auto [ds, dv] = mechanics::computeLinearTrajectory(
        spheres[s_id].v,
        params.F,
        timestep
        );

      if (spheres[s_id].s == types::RBState::Sliding) {
        auto n = findAttachedPlaneNormal(planes, attached, s_id);

        ds = vectorAlongPlane(ds, n);
        dv = vectorAlongPlane(dv, n);
      }

      spheres[s_id].p += ds;
      spheres[s_id].v += dv;
    }
  }



  std::set<size_t> activeSpheres(IntersectDetProcData& intersections){

    std::set<size_t> active;
    for (auto col : intersections) {
      active.insert(col.sphere1_id);
      if (col.sphere2_id.has_value()){
        active.insert(col.sphere2_id.value());
      }
    }

    return active;
  }



  template <concepts::SolverFixtureLevel3 Fixture_T>
  void solve(
    Fixture_T&         scenario,
    types::NanoSeconds timestep)
  {

    Params params = {
      scenario.getGravity(),
      types::HighResolutionClock::now(),
      timestep
    };

    SphereGeomData       spheres;
    InfPlaneGeomData     planes;
    IntersectDetProcData intersections;
    AttachedData         attached;

    auto sphere_idx = scenario.nonFixedSphereRBs();
    auto plane_idx = scenario.fixedInfPlaneRBs();
    auto attachemets = scenario.getAttached();

    // Spheres
    for (size_t i = 0; i < sphere_idx.size(); i++) {

      size_t idx = sphere_idx[i];
      spheres.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbSphereRadius(idx),
        scenario.rbSphereVelocity(idx),
        scenario.rbFrictionCoef(idx),
        scenario.rbMass(idx),
        scenario.rbState(idx),
        params.t_0
      });
    }

    // Planes
    for (size_t i = 0; i < plane_idx.size(); i++) {

      size_t idx = plane_idx[i];
      planes.push_back({
        scenario.globalFramePosition(idx),
        scenario.rbPlaneNormal(idx),
        scenario.rbFrictionCoef(idx),
      });
    }

    // Attachements
    for (auto [s_idx, p_idx] : attachemets) {
      size_t s_id = std::find(sphere_idx.begin(), sphere_idx.end(), s_idx) - sphere_idx.begin();
      size_t p_id = std::find(plane_idx.begin(), plane_idx.end(), p_idx) - plane_idx.begin();

      attached.push_back({s_id, p_id});
    }

    detectAllCollisions(params, intersections, spheres, planes, attached);
    sortAndReduce(intersections);

    while (!intersections.empty()) {
      handleCollision(params, intersections, spheres, planes, attached);

      std::set<size_t> active_before = activeSpheres(intersections);

      sortAndReduce(intersections);

      std::set<size_t> active_after = activeSpheres(intersections);

      bool new_collisions_added = false;
      // New collision for spheres in active_before, but not in active_after
      for (auto id : active_before){
        if (!active_after.contains(id)){
          new_collisions_added = true;
          std::set<size_t> exclude_sphere_idx{id};
          std::set<size_t> exclude_plane_idx{};

          detectCollision(
            params,
            intersections,
            spheres,
            planes,
            attached,
            id,
            exclude_sphere_idx,
            exclude_plane_idx);
        }
      }
      if (new_collisions_added) sortAndReduce(intersections);
    }

    simulateObjects(params, spheres, planes, attached);

    // Update scenario
    for (size_t i = 0; i<spheres.size(); i++) {
      size_t sphere_id = sphere_idx[i];

      scenario.setGlobalFramePosition(sphere_id, spheres[i].p);
      scenario.setSphereVelocity(sphere_id, spheres[i].v);
      scenario.setRbState(sphere_id, spheres[i].s);
    }

    scenario.clearAttached();

    for (auto [s_id, p_id] : attached) {
      scenario.addAttached(
        sphere_idx[s_id],
        plane_idx[p_id]);
    }

  }
}   // namespace dte3607::physengine::solver_dev::level3


#endif // DTE3607_PHYSENGINE_SOLVER_DEVELOPMENT_LEVEL3_H
