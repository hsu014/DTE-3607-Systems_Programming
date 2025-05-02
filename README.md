# DTE-3607-Systems_Programming


# Venv, SCons
Update clients\godot\modules\dte3607physrv\SCsub (line 29)
  env_module.Append(CPPPATH=["C:/vcpkg/installed/x64-windows/include"])


# In clients\godot (terminal)
py -3.10 -m venv .venv          (must be earlier than 3.12)
.venv\scripts\activate
pip install SCons
scons


# Godot
In clients\godot\bin 
  run godot.windows.tools.64.exe

Or setup shortcut in QT:
  Projects tab
  Under Build & Run -> Run
  Run Settings -> Run -> Add...
    Executable:             "path"\clients\godot\bin\godot.windows.tools.64.exe
    Command line arguments: -e "filename".tscn            # Godot file to open
    Working directory:      "path"\Godot_projects         # Godot project directory



# Files used
dte3607_entry_point.h   (clients\godot\modules\dte3607physrv)
dte3607_entry_point.cpp (clients\godot\modules\dte3607physrv)
  includes: 
    "physengine/api.h"
    "dte3607_types.h"
  # The code that calls your solver

api.h (physengine\include\physengine)
  includes: 
    "solvers/solver_dev_level2_3.h" (select version)
    "bits/concepts.h"
  # Defines which solver to use


dte3607_types.h   
dte3607_types.cpp 
  # Creates the fixture used by Godot and your solver


# Godot plane normals
Add the file plane_normal.gd to godot project folder
In RBPlane -> CollisionShape add the script plane_normal.gd (drag and drop)


# Add to dte3607_entry_point.cpp
At the end of function void Dte3607EntryPoint::body_add_shape() (~line 266), add the code:

  // Extract and add indexes for non-fixed or fixed RBs
  for (auto body_id = 0; body_id < m_fixture.m_rigid_bodies.size(); ++body_id) {
    if (m_fixture.m_rigid_bodies.at(body_id).get() == rb) {
      ShapeSW* shape_sw = m_server.shape_owner.get(p_shape);
      switch (shape_sw->get_type()) {
        case PhysicsServerSW::ShapeType::SHAPE_PLANE:
          m_fixture.m_plane_idx.emplace_back(body_id);
          break;

        case PhysicsServerSW::ShapeType::SHAPE_SPHERE:
          m_fixture.m_sphere_idx.emplace_back(body_id);
          break;

        default:
          break;
      }
    }
  }



# Notes
distutils is deprecated with removal planned for Python 3.12 (Used by scons)