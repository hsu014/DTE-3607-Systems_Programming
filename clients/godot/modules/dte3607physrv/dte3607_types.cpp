#include "dte3607_types.h"


namespace frb
{

  SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject()
  {
    return m_object;
  }

  const SpaceObjectBase::SpaceObject& SpaceObjectBase::spaceObject() const
  {
    return m_object;
  }

  SpaceObjectBase::SpaceObject::ASFrameH SpaceObjectBase::pFrame() const
  {
    return spaceObject().pSpaceFrameParent();
  }

  SpaceObjectBase::SpaceObject::Frame SpaceObjectBase::vFrame() const
  {
    return spaceObject().vSpaceFrameParent();
  }

  SpaceObjectBase::Point3 SpaceObjectBase::globalFramePosition() const
  {
    return m_object.frameOriginParent();
  }

  rb_shapes::GodotShape::GodotShape(PhysicsServer::ShapeType p_shape_type)
    : m_shape_type{p_shape_type}
  {
  }

  rb_shapes::SphereRB::SphereRB(const ValueType& radius)
    : Shape(PhysicsServer::ShapeType::SHAPE_SPHERE), m_radius{radius}
  {
  }

  SpaceObjectBase::Point3 rb_shapes::SphereRB::point() const
  {
    return spaceObject().frameOriginParent();
  }

  SpaceObjectBase::ValueType rb_shapes::SphereRB::radius() const
  {
    return m_radius;
  }

  SpaceObjectBase::Point3 rb_shapes::SphereRB::centerOfMass() const
  {
    return point();
  }

  void rb_shapes::SphereRB::setDataFromGodot(const Variant& p_data)
  {
    real_t radius = p_data;
    m_radius      = radius;

    // std::cout << "setting radius of " << m_shape_rid.get_id() << " to "
    //           << m_radius << '\n';
  }

  rb_shapes::PlaneRB::PlaneRB(const types::Vector3& n)
    : Shape(PhysicsServer::ShapeType::SHAPE_PLANE), m_n{n}
  {
  }

  SpaceObjectBase::Point3 rb_shapes::PlaneRB::point() const
  {
    return spaceObject().frameOriginParent();
  }

  SpaceObjectBase::Vector3 rb_shapes::PlaneRB::normal() const
  {
    return spaceObject().vSpaceFrameParent() * m_n;
  }

  SpaceObjectBase::Point3 rb_shapes::PlaneRB::centerOfMass() const
  {
    return point();
  }

  void rb_shapes::PlaneRB::setDataFromGodot(const Variant& p_data) {
    Plane plane = p_data;
    auto const& p_n = plane.normal;
    m_n[0] = p_n[0];
    m_n[1] = p_n[1];
    m_n[2] = p_n[2];

    // std::cout << "setting normal of " << m_shape_rid.get_id() << " to ("
    //           << m_n[0] << ", " << m_n[1] << ", " << m_n[2] << ")" << '\n';

  }

  SpaceObjectBase::ValueType RigidBodyPart::frictionCoef() const
  {
    return m_friction_coef;
  }

  void RigidBodyPart::setFrictionCoef(const ValueType& friction_coef)
  {
    m_friction_coef
      = std::clamp(friction_coef, frictionCoefMin, frictionCoefMax);
  }

  SpaceObjectBase::ValueType RigidBodyPart::mass() const { return m_mass; }

  void RigidBodyPart::setMass(const ValueType& mass) { m_mass = mass; }

  SpaceObjectBase::Point3 RigidBodyPart::centerOfMass() const
  {

    return m_shape->centerOfMass();   // Multiplied with frame or something
  }

  RigidBodyPart::Shape* RigidBodyPart::shape() { return m_shape; }

  RigidBody::RigidBody(const std::string& name) : m_name{name} {}

  RigidBody::Vector3 RigidBody::velocity() const { return m_velocity; }

  size_t RigidBody::nextGoal() const { return m_next_goal; }

  bool RigidBody::isRaised() const { return m_raised; }

  void RigidBody::addAcceleration(const types::Vector3& a) { m_velocity += a; }

  void RigidBody::setVelocity(const types::Vector3& v) { m_velocity = v; }

  void RigidBody::setNextGoal(const size_t goal) { m_next_goal = goal; }

  void RigidBody::setRaisedState(const bool raised) { m_raised = raised; }

  RigidBody::Vector3 RigidBody::centerOfMass()
  {

    // Descr
    // Sum of com. of parts avgd. wrt. individual weight
    auto com = Vector3(0);
    for (auto const& p : m_parts) com += p->centerOfMass();
    return vFrame() * com;
  }

  RigidBody::Mode& RigidBody::mode() { return m_mode; }

  RigidBody::State& RigidBody::state() { return m_state; }







  // Fixture
  size_t Fixture::noRigidBodies() const
  {
    return m_rigid_bodies.size();
  }

  Fixture::Forces Fixture::getGravity() const
  {
    return m_forces;
  }

  void Fixture::setGravity(Forces G) { m_forces = G; }

  types::ValueType Fixture::getMaxSpeed() const {
    return m_max_speed;
  }

  std::vector<types::Point3> Fixture::getPath() const {
    return m_path;
  }

  std::vector<size_t> Fixture::nonFixedSphereRBs() const{
    return m_sphere_idx;
  }

  std::vector<size_t> Fixture::fixedSphereRBs() const{
    return m_static_sphere_idx;
  }

  std::vector<size_t> Fixture::fixedInfPlaneRBs() const{
    return m_plane_idx;
  }

  types::ValueType Fixture::rbSphereRadius(size_t rid) const{
    return m_rigid_bodies.at(rid)->m_parts.at(0)->shape()->radius();
  }

  types::Vector3 Fixture::rbSphereVelocity(size_t rid) const{
    return m_rigid_bodies.at(rid)->velocity();
  }

  size_t Fixture::rbSphereNextGoal(size_t rid) const{
    return m_rigid_bodies.at(rid)->nextGoal();
  }

  bool Fixture::rbSphereIsRaised(size_t rid) const{
    return m_rigid_bodies.at(rid)->isRaised();
  }

  types::Vector3   Fixture::rbPlaneNormal(size_t rid) const{
    return m_rigid_bodies.at(rid)->m_parts.at(0)->shape()->normal();
  }

  Fixture::Point3 Fixture::globalFramePosition(size_t rid) const
  {
    return m_rigid_bodies[rid]->globalFramePosition();
  }

  void Fixture::setGlobalFramePosition(size_t rid, types::Vector3 position){
    auto p0 = globalFramePosition(rid);
    auto lin_trajectory = position - p0;
    m_rigid_bodies[rid]->m_object.translateParent(lin_trajectory);
  }

  void Fixture::setSphereVelocity(size_t rid, types::Vector3 velocity){
    m_rigid_bodies[rid]->setVelocity(velocity);
  }

  void Fixture::setSphereNextGoal(size_t rid, size_t goal){
    m_rigid_bodies[rid]->setNextGoal(goal);
  }

  void Fixture::setSphereRaisedState(size_t rid, bool raised){
    m_rigid_bodies[rid]->setRaisedState(raised);
  }

  RigidBody::Mode Fixture::mode(size_t rid) const
  {
    return m_rigid_bodies[rid]->mode();
  }


}   // namespace frb
