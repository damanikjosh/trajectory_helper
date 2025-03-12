# Trajectory Helper

C++ implementation of [trajectory_planning_helpers](https://github.com/TUMFTM/trajectory_planning_helpers.git)

## Implemented

- [X] `calc_head_curv_an.py` -> [calc_spline_curvatures.hpp](include/trajectory_helper/calc_spline_curvatures.hpp), [calc_spline_headings.hpp](include/trajectory_helper/calc_spline_headings.hpp)
- [X] `calc_splines.py` -> [calc_splines.hpp](include/trajectory_helper/calc_splines.hpp)
- [X] `calc_spline_lengths.py` -> [calc_spline_lengths.hpp](include/trajectory_helper/calc_spline_lengths.hpp)
- [X] `calc_head_curv_num.py` -> [calc_track.hpp](include/trajectory_helper/calc_track.hpp)
- [X] `interp_track.py` -> [interp_track.hpp](include/trajectory_helper/interp_track.hpp)
- [X] `spline_approximation.py` -> [smooth_track.hpp](include/trajectory_helper/smooth_track.hpp)
- [X] `normalize_psi.py` -> [utils.hpp](include/trajectory_helper/utils.hpp)

## Dependencies

- cmake
- build-essential
- Eigen3

Install the dependencies using this command
```bash
sudo apt install cmake build-essential libeigen3-dev
```

## Installation

To install the package, clone the repository
```bash
git clone https://github.com/damanikjosh/trajectory_helper.git
```

Then build using CMake

```bash
cd trajectory_helper
mkdir build
cd build
cmake ..
make
make install
```


## License

Released with the same [LGPL-3.0 license](LICENSE) as original.
