"""运行: python -m unittest discover -s modules/wheeled_mujoco/minimal_sim2sim -p test_wheeled_leg_minimal.py"""
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import wheeled_leg_minimal as runner


class Sim2SimTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(runner.MODEL_XML))
        self.order = runner.build_joint_order(self.model)
        self.sim = runner.Simulation(self.model, self.order)

    def test_source_and_compiled_config(self):
        runner.check_training_sources(verbose=False)
        runner.check_alignment(self.model, self.order, verbose=False)
        self.model.dof_damping[self.order['qvel_ids'][0]] = 1.5
        with self.assertRaisesRegex(ValueError, 'dof_damping'):
            runner.check_alignment(self.model, self.order, verbose=False)

    def test_source_drift_rejected(self):
        source = (runner.TASK / 'v2_wheeled_leg_v1.py').read_text()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'task.py'
            path.write_text(source.replace('scale=4.0,', 'scale=5.0,'))
            with self.assertRaisesRegex(ValueError, 'joint_vel.scale'):
                runner.check_training_sources(task_path=path, verbose=False)

    def test_reset_observation_and_targets(self):
        obs = self.sim.observation(np.array([.2, 0., -.3]))
        expected = np.zeros(34, dtype=np.float32)
        expected[8] = -1
        expected[9:12] = [.2, 0., -.3]
        np.testing.assert_array_equal(obs, expected)
        np.testing.assert_allclose(self.sim.data.ctrl[self.order['actuator_ids']], [.65, -.65, 1.3, -1.3, 0, 0])
        self.sim.step(np.ones(6))
        self.sim.reset()
        np.testing.assert_array_equal(self.sim.last_action, np.zeros(6))
        np.testing.assert_array_equal(self.sim.applied_effort, np.zeros(6))
        self.assertEqual(self.sim.data.time, 0)
        self.assertFalse(self.sim.contact_history)

    def test_action_affine_then_clip(self):
        np.testing.assert_allclose(runner.action_to_targets(np.array([1., -1., 1., -1., 1., -1.]), self.order),
                                   [.9, -.9, 1.55, -1.55, 4., -4.])
        np.testing.assert_allclose(runner.action_to_targets(np.array([-100., 100., -100., 100., 100., -100.]), self.order),
                                   [.3, -.3, .9, -.9, 6., -6.])
        for action in (np.zeros(5), np.full(6, np.nan)):
            with self.assertRaises(ValueError):
                runner.action_to_targets(action, self.order)

    def test_names_map_independent_of_actuator_order(self):
        root = ET.parse(runner.MODEL_XML).getroot()
        root.find('compiler').set('meshdir', str(runner.MODULE / 'asset/wheeled_leg_v2/meshes'))
        actuators = root.find('actuator')
        actuators[:] = list(reversed(list(actuators)))
        key = root.find('keyframe/key')
        key.set('ctrl', ' '.join(reversed(key.get('ctrl').split())))
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        order = runner.build_joint_order(model)
        runner.check_alignment(model, order, verbose=False)
        sim = runner.Simulation(model, order)
        action = np.array([.1, -.2, .3, -.4, .5, -.6])
        sim.step(action)
        self.sim.step(action)
        np.testing.assert_allclose(sim.data.qpos, self.sim.data.qpos, atol=1e-12)
        np.testing.assert_allclose(sim.applied_effort, self.sim.applied_effort, atol=1e-12)
        with self.assertRaises(ValueError):
            runner.indics_for_order(['HipL', 'HipL'], ['HipL'])

    def test_observation_frames_scaling_and_clipping(self):
        d = self.sim.data
        # 已知绕 y 轴 90 度；重力在基座坐标系中应为 +x。
        d.qpos[3:7] = [np.sqrt(.5), 0, np.sqrt(.5), 0]
        offset = np.array([.1, -.2, .3, -.4, 100., -200.])
        d.qpos[self.order['qpos_ids']] += offset
        dq = np.array([40., -40., 2., -3., 10., -11.])
        d.qvel[self.order['qvel_ids']] = dq
        d.qvel[:6] = [1., 2., 3., 20., -20., 4.]
        mujoco.mj_forward(self.model, d)
        last = np.array([10., -10., 1., -2., 3., -4.])
        effort = np.array([200., -200., 2., 3., 4., 5.])
        obs = runner.build_observation(self.model, d, self.order, np.zeros(3), last, effort)
        np.testing.assert_allclose(obs[6:9], [1, 0, 0], atol=1e-6)
        np.testing.assert_allclose(obs[12:16], offset[:4], atol=1e-6)
        np.testing.assert_allclose(obs[16:22], [3, -3, .2, -.3, 1, -1.1])
        np.testing.assert_allclose(obs[22:28], [100, -100, 2, 3, 4, 5])
        np.testing.assert_allclose(obs[28:34], [5, -5, 1, -2, 3, -4])
        np.testing.assert_allclose(obs[3:6], np.clip(d.sensor('imu_gyro').data, -10, 10) * .25)
        np.testing.assert_allclose(obs[:3], np.clip(d.sensor('base_com_linvel').data, -5, 5))
        self.assertGreater(np.linalg.norm(d.sensor('base_com_linvel').data - d.sensor('imu_linear_velocity').data), .01)
        self.assertLess(max(self.sim.sensor_residuals().values()), 1e-8)

    def test_substep_effort_cache_and_decimation(self):
        action = np.array([.5, -.2, .3, -.7, .6, -.4], dtype=np.float32)
        reference = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, reference, self.model.key('home').id)
        reference.ctrl[self.order['actuator_ids']] = runner.action_to_targets(action, self.order)
        mujoco.mj_forward(self.model, reference)
        for _ in range(4):
            torque = runner.expected_effort(self.model, reference, self.order)
            mujoco.mj_step(self.model, reference)
        self.sim.step(action)
        self.assertAlmostEqual(self.sim.data.time, .008)
        self.assertEqual(self.sim.steps, 4)
        np.testing.assert_allclose(self.sim.applied_effort, torque, atol=1e-12)
        obs = self.sim.observation(np.zeros(3))
        np.testing.assert_allclose(obs[22:28], torque, atol=1e-6)
        np.testing.assert_allclose(obs[28:34], action)
        self.assertGreater(np.max(abs(self.sim.data.actuator_force[self.order['actuator_ids']] - torque)), 1e-5)
        np.testing.assert_allclose(self.sim.data.qpos, reference.qpos, atol=1e-12)

    def test_termination_tilt_timeout_contact_history(self):
        self.assertEqual(self.sim.termination(), [])
        self.sim.data.qpos[3:7] = [np.sqrt(.5), np.sqrt(.5), 0, 0]
        mujoco.mj_forward(self.model, self.sim.data)
        self.assertTrue(any('tilt=' in reason for reason in self.sim.termination()))
        self.sim.reset()
        self.sim.data.time = 40
        self.assertIn('timeout', self.sim.termination())
        self.sim.data.time = 0
        force = np.zeros((self.model.nbody, 3))
        force[self.model.body('HipL').id, 2] = 2
        self.sim.contact_history.append(force)
        self.sim.contact_history.append(np.zeros_like(force))
        self.assertTrue(any('contact:HipL' in reason for reason in self.sim.termination()))


if __name__ == '__main__':
    unittest.main()
