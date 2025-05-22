#ifndef TEST_SETUP_H
#define TEST_SETUP_H

#include <physengine/bits/types.h>

using namespace dte3607::physengine;

namespace dte3607::benchmarking::setup{


  std::vector<std::tuple<types::Vector3, types::Vector3>>
  createSphereData(int _x, int _y, int _z)
  {
    std::vector<std::tuple<types::Vector3, types::Vector3>> sphere_data;

    auto const p = types::Vector3{1.0, 1.0, 1.0};
    auto const v = types::Vector3{0, -100, 0};

    sphere_data.reserve(_x * _y * _z);
    for (auto x = 0; x < _x; ++x) {
      for (auto y = 0; y < _y; ++y) {
        for (auto z = 0; z < _z; ++z) {

          sphere_data.emplace_back(
            v, p + types::Vector3{2. * x, 2. * y, 2. * z});
        }
      }
    }

    return sphere_data;
  }


}



#endif   // TEST_SETUP_H
