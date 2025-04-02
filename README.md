# DTE-3607-Systems_Programming


# Venv, SCons
Update clients\godot\modules\dte3607physrv\SCsub
  env_module.Append(CPPPATH=["C:/vcpkg/installed/x64-windows/include"])

py -3.10 -m venv .venv          (must be earlier than 3.12)
.venv\scripts\activate
pip install SCons
scons


# Godot
In clients\godot\bin 
  run godot.windows.tools.64.exe


# Notes
distutils is deprecated with removal planned for Python 3.12 (Used by scons)