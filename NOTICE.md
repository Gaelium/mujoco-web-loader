# NOTICE

mujoco-web-loader
Copyright (c) 2026 Gaelium

Licensed under the Apache License, Version 2.0.

This product builds on the following third-party work:

## MuJoCo

MuJoCo (Multi-Joint dynamics with Contact), by Google DeepMind.
Licensed under the Apache License, Version 2.0.
https://github.com/google-deepmind/mujoco

This package drives MuJoCo through the official `@mujoco/mujoco` WebAssembly
build and includes workarounds specific to that build (see the module
documentation in `src/physics/mujocoWorker.ts`).

## AM-ARM200 robot model

The example model referenced by this repo's demo, the AM-ARM200, is by
Li Yiteng & Wu Zhiyong, licensed under the Apache License, Version 2.0
(see the LICENSE file distributed alongside the model where it is bundled).

**Divergence note:** the URDF used by the originating platform diverges from
the upstream release — the jaw links carry *measured multi-box collision
geometry* replacing the original mesh collisions (calibrated against the
physical gripper), plus a visual-frame fix for the moving jaw. Where this
repo distributes the model, it must be described as "AM-ARM200, with
measured collision refinements", not as an unmodified upstream copy.
