# Model provenance

The model is mechanically extracted from the supplied official
`AuboRobot/aubo_robot` Noetic archive:

- Archive: `/mnt/c/Users/zhj/AppData/Local/Temp/aubo_robot_noetic_model_audit.zip`
- Archive SHA-256: `944fbd961adf50b4a0357d70e0e17976022572bcd11357a4c0c2a03626cb5af5`.
- Source URDF: `aubo_robot-Noetic/aubo_robot/aubo_description/urdf/aubo_i5.urdf`
- Source mesh directories:
  `aubo_robot-Noetic/aubo_robot/aubo_description/meshes/aubo_i5/visual` and
  `aubo_robot-Noetic/aubo_robot/aubo_description/meshes/aubo_i5/collision`
- Upstream license: BSD-3-Clause, reproduced verbatim in `LICENSE` from
  `aubo_robot-Noetic/aubo_robot/LICENSE`.

Extraction on 2026-10-14 copied only the URDF and the fourteen meshes referenced
by it. The archive's unreferenced `collision/pedestal.STL` was excluded.

The only model-content transformation is replacing each mesh URI prefix
`package://aubo_description/` with `package://aubo_i5_description/`. Link names,
joint names, origins, axes, inertials, and all source joint-limit values are
otherwise preserved. In particular, no velocity or effort limit has been
invented or changed.
