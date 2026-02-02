Simulation of Kinect Measurements
=============

> **Note**: This is a modernized fork of [jbohg/render_kinect](https://github.com/jbohg/render_kinect) with CGAL 6.x support, Python bindings, and modern C++ (C++17). See [Changes from upstream](#changes-from-upstream) for details.

This C++ project implements the Kinect sensor model as described in

**BlenSor: Blender Sensor Simulation Toolbox.** *Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang. In Advances in Visual Computing. Lecture Notes in Computer Science. pp 199--208. 2011.*

and as implemented in python as a Blender plugin ([BlenSor Webpage](http://www.blensor.org)).

The simulated measurement shows typical artifacts of a kinect, e.g., occlusion boundaries due to distance between IR projector and IR camera, 8 bit quantisation, smoothing within a 9x9 pixel correlation window. This implementation also includes options for adding either Gaussian, Perlin or Simplex noise.

This implementation is simplified in that it accepts only a single rigid object as input.

If you are using this code, please cite our work:

**Robot Arm Pose Estimation through Pixel-Wise Part Classification.** *Jeannette Bohg, Javier Romero, Alexander Herzog and Stefan Schaal. Proceedings of the 2014 IEEE International Conference on Robotics and Automation. pp 3143--3150. 2014.*

Requirements
----------
The following libraries are required to compile the code:

* OpenCV (image I/O, filtering)
* CGAL 6.x (Fast Intersection Queries)
* Eigen3 (Linear Algebra)
* assimp (Mesh I/O)

The following are bundled:

* libnoise (Generation of Different Noise Types) - included in `lib/libnoise/`

The following libraries are optional:

* OpenMP (Parallelization)
* PCL (Point cloud I/O)

For Python bindings:

* Cython >= 3.0
* NumPy

Installation
------------

### C++ Library

```bash
mkdir build
cd build
cmake ..
make -j
```

### Python Extension

```bash
pip install .
```

Or for development:

```bash
pip install -e .
```

Python Usage
------------

```python
from render_kinect import KinectSim, NoiseType
import numpy as np

# Load or create mesh
vertices = np.array([...], dtype=np.float32)  # (N, 3)
faces = np.array([...], dtype=np.int32)       # (M, 3)

# Create simulator
sim = KinectSim()

# Simulate depth image
depth = sim.simulate(
    vertices, faces,
    width=640, height=480,
    fx=582.7, fy=582.7,
    cx=320.8, cy=245.3,
    z_near=0.5, z_far=4.0,
    baseline=0.075,
    noise=NoiseType.PERLIN
)
```

C++ Testing
------------
There is a small test program that will render the simulated kinect measurements of a wheel at a number of slightly perturbed transformations relative to the camera.

```bash
cd bin
./render_object wheel.obj
```

This should store a number of depth and labeled images in /tmp. If you have PCL installed, it also stores point clouds as pcl files.

Point clouds generated from a simulated kinect measurement taken from a wheel in 10 slightly different poses. This measurement does not expose additional noise.
![](data/Wheels.png?raw=true)

Changes from upstream
---------------------

This fork ([hummat/render_kinect](https://github.com/hummat/render_kinect)) includes the following improvements over the original [jbohg/render_kinect](https://github.com/jbohg/render_kinect):

**CGAL 6.x Compatibility**
- Updated from deprecated CGAL 5.x API to modern CGAL 6.x
- Uses `std::variant` for intersection results instead of `CGAL::Object`

**Modern C++**
- C++17 standard
- `std::unique_ptr` instead of `boost::shared_ptr`
- Proper exception handling instead of `exit(-1)`

**Build System**
- CMake 3.10+ with modern target-based configuration
- Bundled libnoise (no external dependency)
- PCL is truly optional

**Python Bindings**
- Cython-based Python extension
- Configurable camera parameters (fx, fy, cx, cy, z_near, z_far, baseline)
- Selectable noise type (None, Gaussian, Perlin, Simplex)
- Direct NumPy array input/output

**API Enhancements**
- Constructor accepting raw vertex/face arrays (not just mesh files)
- Configurable depth range and baseline
- Debug mode for intersection visualization
