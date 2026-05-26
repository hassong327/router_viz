# qcar2_msgs

ROS 2 interface package for QCar2-specific custom message definitions.
This package provides `.msg` interfaces only. It does not include runtime nodes or libraries.

## Overview

`qcar2_msgs` centralizes message contracts shared across perception, planning, and control modules.
The package is built with `rosidl_default_generators` and exports `rosidl_default_runtime`.

## Message Definitions

- `msg/Detection2D.msg`
  - Single 2D detection output (class, confidence, 2D bounding box).
- `msg/Detection2DArray.msg`
  - Timestamped list of 2D detections.
- `msg/Object3D.msg`
  - Single 3D localized object result (class, confidence, position, distance).
- `msg/Object3DArray.msg`
  - Timestamped list of 3D localized objects.
- `msg/LateralGuidance.msg`
  - Lateral guidance/control diagnostics and status values.
- `msg/PhQuintic.msg`
  - One PH quintic curve segment (G1 Hermite-based representation).
- `msg/PhQuinticPath.msg`
  - Ordered sequence of PH quintic segments for a full path.
- `msg/BezierCurve.msg`
  - Legacy Bezier curve representation.

## BezierCurve Status

- `BezierCurve.msg` is planned to be unused in upcoming QCar2 workflows.
- It is currently retained to preserve compatibility with existing integrations.

## Dependencies

- Build tool: `ament_cmake`, `rosidl_default_generators`
- Message dependencies: `std_msgs`, `geometry_msgs`
- Runtime export: `rosidl_default_runtime`

## Maintainer Notes

- Package metadata (`description`, `license`) in `package.xml` still contains TODO placeholders.
- If message schemas change, update:
  - `msg/*.msg`
  - `CMakeLists.txt` (`rosidl_generate_interfaces`)
  - dependent publishers/subscribers in other packages
