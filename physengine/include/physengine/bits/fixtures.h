#ifndef DTE3607_PHYSENGINE_FIXTURES_H
#define DTE3607_PHYSENGINE_FIXTURES_H

#include "types.h"

// stl
#include <variant>

namespace dte3607::physengine::fixtures
{

  struct FixtureLevel1 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;



    /*** API concept required methods ***/

    // Global properties
    size_t noRigidBodies() const {
      return m_spheres.size();
    }

    void   setGravity([[maybe_unused]] Forces G) {
      m_gravity = G;
    }

    // RB properties
    types::Point3 globalFramePosition([[maybe_unused]] size_t rid) const {
      return m_spheres[rid].translation;
    }

    void setGlobalFramePosition(size_t rid, const Vector3& pos) {
      m_spheres[rid].translation = pos;
    }

    types::Point3 globalFrameVelocity([[maybe_unused]] size_t rid) const {
      return m_spheres[rid].velocity;
    }

    Forces getGravity() const {
      return m_gravity;
    }

    /*** Fixture unit-test setup API ***/
    size_t createSphere([[maybe_unused]] ValueType radius      = 1.,
                        [[maybe_unused]] Vector3   velocity    = {0, 0, 0},
                        [[maybe_unused]] Vector3   translation = {0, 0, 0})
    {
      // Same as: m_spheres.push_back(Sphere(radius, velocity, translation));
      m_spheres.push_back({radius, velocity, translation});
      return m_spheres.size() - 1;
    }

    /*** END API requirements ***/

    struct Sphere {
      types::ValueType radius;
      types::Vector3   velocity;
      types::Vector3   translation;
    };

    /*** Members ***/
    std::vector<Sphere> m_spheres;
    Forces m_gravity;
  };



  struct FixtureLevel2 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;


    /*** API concept required methods ***/

    // Environment
    void      setGravity(Forces G) {
      m_gravity = G;
    }

    Forces    getGravity() const {
      return m_gravity;
    }

    // RBs
    size_t    noRigidBodies() const {
      return m_rigid_bodies.size();
    }

    std::vector<size_t> nonFixedSphereRBs() const {
      return m_sphere_idx;
    }

    std::vector<size_t> fixedInfPlaneRBs() const {
      return m_plane_idx;
    }

    // RB properties
    ValueType rbSphereRadius(size_t s_rid) const {
      return m_rigid_bodies[s_rid].radius.value();
    }

    Vector3   rbSphereVelocity(size_t s_rid) const {
      return m_rigid_bodies[s_rid].velocity.value();
    }

    Vector3   rbPlaneNormal(size_t p_rid) const {
      return m_rigid_bodies[p_rid].normal.value();
    }

    Point3    globalFramePosition(size_t rid) const {
      return m_rigid_bodies[rid].translation;
    }

    void      setGlobalFramePosition(size_t rid, Vector3 pos) {
      m_rigid_bodies[rid].translation = pos;
    }



    /*** Fixture unit-test setup API ***/

    size_t createSphere(ValueType radius      = 1.,
                        Vector3   velocity    = {0, 0, 0},
                        Vector3   translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({
        radius,
        velocity,
        translation,
        std::nullopt
      });

      auto id = m_rigid_bodies.size() - 1;
      m_sphere_idx.push_back(id);

      return id;
    }

    size_t createFixedInfPlane(Vector3 normal      = {0, 1, 0},
                               Vector3 translation = {0, 0, 0})
    {
      m_rigid_bodies.push_back({
        std::nullopt,
        std::nullopt,
        translation,
        normal
      });

      auto id = m_rigid_bodies.size() - 1;
      m_plane_idx.push_back(id);

      return id;
    }

    /*** END API requirements ***/

    struct RigidBody {
      std::optional<ValueType> radius;
      std::optional<Vector3>   velocity;
      Vector3                  translation;
      std::optional<Vector3>   normal;
    };

    /*** Members ***/
    std::vector<RigidBody>     m_rigid_bodies;
    std::vector<size_t>        m_sphere_idx;
    std::vector<size_t>        m_plane_idx;
    Forces                     m_gravity;
  };



  struct FixtureLevel3 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;


    /*** API concept required methods ***/

    // Environment
    void      setGravity(Forces G) {
      m_gravity = G;
    }

    Forces    getGravity() const {
      return m_gravity;
    }

    // RBs
    size_t    noRigidBodies() const {
      return m_rigid_bodies.size();
    }

    std::vector<size_t> nonFixedSphereRBs() const {
      return m_sphere_idx;
    }

    std::vector<size_t> fixedInfPlaneRBs() const {
      return m_plane_idx;
    }

    // RB properties
    void      setGlobalFramePosition(size_t rid, Vector3 pos) {
      m_rigid_bodies[rid].translation = pos;
    }

    Point3    globalFramePosition(size_t rid) const {
      return m_rigid_bodies[rid].translation;
    }

    ValueType rbSphereRadius(size_t s_rid) const {
      return m_rigid_bodies[s_rid].radius.value();
    }

    void setSphereVelocity(size_t rid, Vector3 v) {
      m_rigid_bodies[rid].velocity = v;
    }

    Vector3   rbSphereVelocity(size_t s_rid) const {
      return m_rigid_bodies[s_rid].velocity.value();
    }

    RBState rbInitialState(size_t s_rid) const {
      return m_rigid_bodies[s_rid].initial_state.value();
    }

    ValueType rbFrictionCoef (size_t rid) const {
      return m_rigid_bodies[rid].friction_coef;
    }

    ValueType rbMass (size_t s_rid) const {
      return m_rigid_bodies[s_rid].mass.value();
    }

    Vector3   rbPlaneNormal(size_t p_rid) const {
      return m_rigid_bodies[p_rid].normal.value();
    }

    ValueType rbPlaneMaxFrictionCoef() const {
      /*
      // ValueType max_friction = 0.;
      // for (size_t id : m_plane_idx) {
      //   if (m_rigid_bodies[id].friction_coef > max_friction) {
      //     max_friction = m_rigid_bodies[id].friction_coef;
      //   }
      // }
      // return max_friction; */
      return m_max_friction;
    }

    ValueType rbSphereMaxFrictionCoef() const {
      /*
      //ValueType max_friction = 0.;
      // for (size_t id : m_sphere_idx) {
      //   if (m_rigid_bodies[id].friction_coef > max_friction) {
      //     max_friction = m_rigid_bodies[id].friction_coef;
      //   }
      // }
      // return max_friction; */
      return m_max_friction;
    }

    void      setRbState (size_t rid, RBState state) {
      m_rigid_bodies[rid].initial_state = state;
    }

    RBState   rbState(size_t rid) const {

      if (m_rigid_bodies[rid].initial_state.has_value()) {
        return m_rigid_bodies[rid].initial_state.value();
      }
      return {};
    }

    // Attachements
    void clearAttached() {
      m_attached.clear();
    }

    std::vector<std::pair<size_t, size_t>>
    getAttached() {
      return m_attached;
    }

    void addAttached(size_t s_id, size_t p_id) {
      m_attached.push_back({s_id, p_id});
    }



    /*** Fixture unit-test setup API ***/

    size_t createSphere(ValueType radius        = 1.,
                        Vector3   velocity      = {0, 0, 0},
                        Vector3   translation   = {0, 0, 0},
                        RBState   initial_state = RBState::Free,
                        ValueType friction_coef = 0.,
                        ValueType mass          = 1.)
    {
      m_rigid_bodies.push_back({
        radius,
        velocity,
        translation,
        initial_state,
        friction_coef,
        mass,
        std::nullopt
      });

      auto id = m_rigid_bodies.size() - 1;
      m_sphere_idx.push_back(id);

      return id;
    }

    size_t createFixedInfPlane(Vector3   normal        = {0, 1, 0},
                               Vector3   translation   = {0, 0, 0},
                               ValueType friction_coef = 0.)
    {
      m_rigid_bodies.push_back({
        std::nullopt,
        std::nullopt,
        translation,
        std::nullopt,
        friction_coef,
        std::nullopt,
        normal
      });

      auto id = m_rigid_bodies.size() - 1;
      m_plane_idx.push_back(id);

      return id;
    }

    /*** END API requirements ***/

    struct RigidBody {
      std::optional<ValueType> radius;
      std::optional<Vector3>   velocity;
      Vector3                  translation;
      std::optional<RBState>   initial_state;
      ValueType                friction_coef;
      std::optional<ValueType> mass;
      std::optional<Vector3>   normal;
    };

    /*** Members ***/
    std::vector<RigidBody>     m_rigid_bodies;
    std::vector<size_t>        m_sphere_idx;
    std::vector<size_t>        m_plane_idx;
    Forces                     m_gravity;
    ValueType                  m_max_friction = 1;
    std::vector<std::pair<size_t, size_t>> m_attached;
  };



  struct FixtureLevel4 {

    /*** API concept required types ***/

    // Types
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;

    // Fixture types
    using Forces = types::Vector3;

    // RigidBody types
    using RBState = types::RBState;



    /*** END API requirements ***/
  };


}   // namespace dte3607::physengine::fixtures


#endif   // DTE3607_PHYSENGINE_FIXTURES_H
