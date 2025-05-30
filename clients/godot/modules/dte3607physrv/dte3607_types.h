#ifndef DTE3607_TYPES_H
#define DTE3607_TYPES_H

// DTE-3607 PhysEngine
#include <physengine/bits/types.h>

// physrv2
#include "physics_server_sw.h"

// godot
#include "core/variant.h"

// stl
#include <string>
#include <memory>
#include <vector>


namespace frb
{

  namespace types = dte3607::physengine::types;

  struct SpaceObjectBase {

    // Concept requirements
    using ValueType   = types::Point3::ElementType;
    using Point3      = types::Point3;
    using Point3H     = types::Point3H;
    using Vector3     = types::Vector3;
    using Vector3H    = types::Vector3H;
    using SpaceObject = types::ProjectiveSpaceObject;
    using Timepoint   = types::HighResolutionTP;


    // Common properties
    SpaceObject m_object;

    // Lin. alg. prop. access
    virtual SpaceObject&       spaceObject();
    virtual SpaceObject const& spaceObject() const;

    typename SpaceObject::ASFrameH pFrame() const;
    typename SpaceObject::Frame    vFrame() const;

    Point3 globalFramePosition() const;
  };


  namespace rb_shapes
  {


    struct GodotShape : SpaceObjectBase {
      GodotShape(PhysicsServer::ShapeType p_shape_type);
      virtual ~GodotShape() = default;

      RID                      m_shape_rid;
      PhysicsServer::ShapeType m_shape_type;

      virtual void setDataFromGodot(Variant const& p_data) = 0;
    };



    struct Shape : GodotShape {
      using GodotShape::GodotShape;

      virtual Point3    centerOfMass() const = 0;
      virtual Point3    point() const = 0;
      virtual ValueType radius() const { return 0; };
      virtual Vector3   normal() const { return {}; };
    };



    struct SphereRB : Shape {
      SphereRB(ValueType const& radius = 1.);

      // Sphere prop. access
      Point3    point() const override;
      ValueType radius() const override;
      Point3    centerOfMass() const override;

      // Concept requirements END

      void setDataFromGodot(Variant const& p_data) override;

    private:
      ValueType m_radius{1.0};
    };



    struct PlaneRB : Shape {
      PlaneRB(types::Vector3 const& n = {.0, 1., .0});

      // Plane prop. access
      Point3  point() const override;
      Vector3 normal() const override;
      Point3  centerOfMass() const override;

      void setDataFromGodot(Variant const& p_data) override;

    private:
      Vector3 m_n{0, 1, 0};
    };
  }   // namespace rb_shapes



  struct RigidBodyPart : SpaceObjectBase {

    using Shape = rb_shapes::Shape;

    // Class type constants
    static constexpr ValueType frictionCoefMin{0.0};
    static constexpr ValueType frictionCoefMax{1.0};

    ValueType frictionCoef() const;
    void      setFrictionCoef(ValueType const& friction_coef);

    ValueType mass() const;
    void      setMass(ValueType const& mass);
    ValueType m_mass{1.0};

    virtual Point3 centerOfMass() const;

    Shape* shape();

    Shape* m_shape;

  private:
    ValueType m_friction_coef{0.0};   // 0 == no friction
  };



  struct GodotRigidBody : SpaceObjectBase {
    RID m_body_rid;
  };

  struct RigidBody : GodotRigidBody {

    RigidBody(const std::string& name = "");

    // Concept requirements
    using ValueType        = types::Point3::ElementType;
    using Point3           = types::Point3;
    using Point3H          = types::Point3H;
    using Vector3          = types::Vector3;
    using Vector3H         = types::Vector3H;
    using SpaceObjectFrame = types::ProjectiveSpaceObject;
    using Timepoint        = types::HighResolutionTP;



    // Mechanics property access
    Vector3 velocity() const;

    /** a; in the "parent" spacial frame */
    void addAcceleration(types::Vector3 const& a);

    /** v; in the "parent" spacial frame */
    void setVelocity(types::Vector3 const& v);


    // Propagated mechanics property access
    Vector3 centerOfMass();

    // States
    enum class Mode { NonFixed, Fixed, Hmm };
    Mode& mode();

    enum class State { Free, Resting, Sliding, Rolling };
    State& state();

    //    RID& bodyRid() { return m_body_rid; }

    //  private:
    Timepoint m_timepoint;
    Vector3   m_velocity{0, 0, 0};
    Mode      m_mode{Mode::NonFixed};
    State     m_state{State::Free};


  public:
    std::string m_name;

    using Part    = RigidBodyPart;
    using PartPtr = std::unique_ptr<RigidBodyPart>;
    using Parts   = std::vector<PartPtr>;

    Parts m_parts;
  };





  struct Fixture {


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
    using RigidBodyPtr = std::unique_ptr<RigidBody>;
    using RigidBodies  = std::vector<RigidBodyPtr>;

    using RBShape    = rb_shapes::Shape;
    using RBShapePtr = std::unique_ptr<RBShape>;
    using RBShapes   = std::vector<RBShapePtr>;

    using RBPart  = RigidBodyPart;
    using RBMode  = RigidBody::Mode;
    using RBState = RigidBody::State;



    /*** API concept required methods ***/

    // Environment
    Forces              getGravity() const;
    void                setGravity(Forces);

    // RBs
    size_t              noRigidBodies() const;
    std::vector<size_t> nonFixedSphereRBs() const;
    std::vector<size_t> fixedInfPlaneRBs() const;


    // RB properties
    ValueType           rbSphereRadius(size_t rid) const;
    Vector3             rbSphereVelocity(size_t rid) const;
    Vector3             rbPlaneNormal(size_t rid) const;
    types::Point3       globalFramePosition(size_t rid) const;

    // Set RB properties
    void                setGlobalFramePosition(size_t rid, Vector3 position);
    void                setSphereVelocity(size_t rid, Vector3 velocity);

    // Mode & state
    RigidBody::Mode     mode(size_t rid) const;

    // Construction methods
    size_t createSphere(ValueType radius      = 1.,
                        Vector3   velocity    = {0, 0, 0},
                        Vector3   translation = {0, 0, 0});

    size_t createFixedInfPlane(Vector3 normal      = {0, 1, 0},
                               Vector3 translation = {0, 0, 0});




    /*** END API requirements ***/



    /*** Members ***/

    RigidBodies         m_rigid_bodies;
    RBShapes            m_rb_shapes;

    Forces              m_forces;

    std::vector<size_t> m_sphere_idx;
    std::vector<size_t> m_plane_idx;
  };

}   // namespace frb

#endif   // DTE3607_TYPES_H
