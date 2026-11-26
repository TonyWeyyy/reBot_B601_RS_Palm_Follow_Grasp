#!/usr/bin/env python3
"""B601-RS + Gemini 305 + 相机壳3.0 STL 双 URDF（柔性夹爪 / 官方刚性夹爪）MuJoCo 4.0+ 场景编译自检。"""

from __future__ import annotations

import os
import sys
import unittest

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TWIN_DIR = os.path.join(ROOT, "mujoco_twin")
for p in (ROOT, TWIN_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import scene_builder  # noqa: E402


class MuJoCoRSTwinTests(unittest.TestCase):
    """验证柔性夹爪 URDF 与官方刚性夹爪 URDF 均可在项目内挂载相机壳3.0 STL + Gemini 305 编译运行。"""

    def _assert_common_scene(self, variant: str) -> None:
        resolved_var, urdf_path = scene_builder.resolve_urdf_path(variant)
        self.assertEqual(resolved_var, variant)
        self.assertTrue(os.path.isfile(urdf_path))
        self.assertTrue(os.path.abspath(urdf_path).startswith(os.path.abspath(ROOT)))

        spec = scene_builder.build_b601_spec(urdf_variant=variant)
        model = spec.compile()
        data = mujoco.MjData(model)

        # 验证 6 臂关节 + 2 夹爪滑移关节齐全
        for jn in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_joint1", "gripper_joint2"):
            self.assertGreaterEqual(model.joint(jn).id, 0, f"{variant} 缺少关节 {jn}")

        # 验证 相机壳3.0 STL、Gemini 305 机身/双目镜头/光学中心 site 与 4 个 mocap 叠加体齐全
        self.assertGreaterEqual(model.geom("gemini305_mount_v3").id, 0)
        self.assertGreaterEqual(model.geom("gemini305_housing").id, 0)
        self.assertGreaterEqual(model.geom("gemini305_lens_left").id, 0)
        self.assertGreaterEqual(model.geom("gemini305_lens_right").id, 0)
        self.assertGreaterEqual(model.site("gemini305_cam").id, 0)
        self.assertGreaterEqual(model.site("tcp_site").id, 0)
        for mname in ("palm_target", "cmd_tcp_marker", "workspace_ring", "ik_target"):
            self.assertGreaterEqual(int(model.body(mname).mocapid[0]), 0)

        # 加载 home 关键帧并确认 FK 位于合法工作空间内
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        tcp = data.body("link6").xpos
        self.assertTrue(np.all(np.isfinite(tcp)))
        self.assertGreater(float(tcp[2]), 0.10)

        # 验证 0° 倾角下 Gemini 305 光轴 (+Z) 与 link6 (+Z) 平行
        link6_z = data.body("link6").xmat.reshape(3, 3)[:, 2]
        cam_z = data.site("gemini305_cam").xmat.reshape(3, 3)[:, 2]
        self.assertGreater(float(np.dot(link6_z, cam_z)), 0.99)

    def test_soft_gripper_urdf_with_gemini305_compiles(self) -> None:
        self._assert_common_scene("soft_gripper")

    def test_standard_official_urdf_with_gemini305_compiles(self) -> None:
        self._assert_common_scene("standard")


if __name__ == "__main__":
    unittest.main(verbosity=2)
