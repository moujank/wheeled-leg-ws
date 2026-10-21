"""Independent arm sim2sim regression checks; python -m unittest discover -s ... -p test_wheeled_arm_minimal.py."""
from contextlib import redirect_stdout
import io
import tempfile
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

import wheeled_arm_minimal as arm


class ArmMigrationTests(unittest.TestCase):
    def setUp(self):
        self.model = mujoco.MjModel.from_xml_path(str(arm.MODEL_XML))
        self.order = arm.build_joint_order(self.model)
        self.sim = arm.Simulation(self.model, self.order)

    def test_source_model_and_reset(self):
        arm.audit_sources(verbose=False)
        arm.check_alignment(self.model, self.order, verbose=False)
        obs = self.sim.observation([0, 0, 0])
        expected = np.zeros(58)
        expected[8] = -1
        np.testing.assert_allclose(obs, expected)
        self.sim.step(np.ones(12))
        self.sim.reset()
        np.testing.assert_allclose(self.sim.observation([0, 0, 0]), expected)
        self.assertEqual(len(self.sim.history), 0)
        self.assertEqual(self.sim.data.time, 0)
        self.model.dof_damping[-1] = 6
        with self.assertRaises(ValueError):
            arm.check_alignment(self.model, self.order, verbose=False)

    def test_distinct_observation_action_orders(self):
        # Assign distinguishable values by NAME. Observation order interleaves arm and legs.
        for i, n in enumerate(arm.ACTION_ORDER):
            self.sim.data.qpos[self.model.joint(n).qposadr[0]] = arm.DEFAULT_Q[n] + .01*(i+1)
            self.sim.data.qvel[self.model.joint(n).dofadr[0]] = i+1
        self.sim.effort = np.arange(1, 13)*2.
        self.sim.last_action = np.arange(12)*.1
        mujoco.mj_forward(self.model, self.sim.data)
        obs = self.sim.observation([.1, 0, -.2])
        expected_pos = [.01*(arm.ACTION_ORDER.index(n)+1) for n in arm.POSITION_ORDER]
        expected_vel = [.1*(arm.ACTION_ORDER.index(n)+1) for n in arm.OBS_ORDER]
        expected_effort = [2.*(arm.ACTION_ORDER.index(n)+1) for n in arm.OBS_ORDER]
        np.testing.assert_allclose(obs[12:22], expected_pos, atol=1e-7)
        np.testing.assert_allclose(obs[22:34], expected_vel, atol=1e-7)
        np.testing.assert_allclose(obs[34:46], expected_effort)
        np.testing.assert_allclose(obs[46:], self.sim.last_action)

    def test_observation_frames_and_clip_before_scale(self):
        d = self.sim.data
        theta = .4
        d.qpos[3:7] = [np.cos(theta/2), np.sin(theta/2), 0, 0]
        d.qvel[:6] = [2, 3, 1, 1, 20, -3]
        d.qvel[self.order['v_obs']] = np.linspace(-50, 50, 12)
        self.sim.effort[:] = 110
        self.sim.last_action[:] = -7
        mujoco.mj_forward(self.model, d)
        obs = self.sim.observation([0, 0, 0])
        np.testing.assert_allclose(obs[6:9], [0, -np.sin(theta), -np.cos(theta)], atol=1e-7)
        np.testing.assert_allclose(obs[3:6], np.clip(d.sensor('imu_gyro').data, -10, 10)*.25)
        np.testing.assert_allclose(obs[22:34], np.clip(d.qvel[self.order['v_obs']], -30, 30)*.1, atol=1e-7)
        np.testing.assert_allclose(obs[34:46], 100)
        np.testing.assert_allclose(obs[46:], -5)
        self.assertGreater(np.linalg.norm(d.sensor('base_com_linvel').data-d.sensor('imu_linear_velocity').data), .1)
        self.assertLess(max(self.sim.sensor_errors().values()), 1e-10)

    def test_target_scaling_clipping_and_torque_saturation(self):
        np.testing.assert_allclose(arm.action_to_targets(np.zeros(12)), [arm.DEFAULT_Q[n] for n in arm.ACTION_ORDER])
        np.testing.assert_allclose(arm.action_to_targets(np.full(12, 100)), arm.TARGET_LIMIT[:, 1])
        np.testing.assert_allclose(arm.action_to_targets(np.full(12, -100)), arm.TARGET_LIMIT[:, 0])
        # Raw action 2 must not be clipped to 1: folded Arm_Y1 target = -.65 + .3.
        action = np.zeros(12)
        action[7] = 2
        self.assertAlmostEqual(arm.action_to_targets(action)[7], -.35)
        for sign in (-1, 1):
            self.sim.data.qvel[self.order['v_action']] = sign*1000
            mujoco.mj_forward(self.model, self.sim.data)
            np.testing.assert_allclose(self.sim.data.actuator_force[self.order['actuator']], -sign*arm.EFFORT_LIMIT)
        for value in (np.zeros(6), np.full(12, np.nan)):
            with self.assertRaises(ValueError):
                arm.action_to_targets(value)

    def test_actuator_permutation(self):
        tree = ET.parse(arm.MODEL_XML).getroot()
        tree.find('compiler').set('meshdir', str(arm.MODEL_XML.parent / '../wheeled_arm_v2/meshes'))
        actuators = tree.find('actuator')
        actuators[:] = list(actuators)[::-1]
        home = tree.find('keyframe/key')
        home.set('ctrl', ' '.join(home.get('ctrl').split()[::-1]))
        m = mujoco.MjModel.from_xml_string(ET.tostring(tree, encoding='unicode'))
        order = arm.build_joint_order(m)
        arm.check_alignment(m, order, verbose=False)
        sim = arm.Simulation(m, order)
        action = np.linspace(-1, 1, 12)
        sim.step(action)
        self.sim.step(action)
        np.testing.assert_allclose(sim.data.qpos, self.sim.data.qpos, atol=1e-12)
        np.testing.assert_allclose(sim.observation([0, 0, 0]), self.sim.observation([0, 0, 0]), atol=1e-7)

    def test_substep_cache_and_termination(self):
        action = np.linspace(-1, 1, 12)
        d = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, d, self.model.key('home').id)
        d.ctrl[self.order['actuator']] = arm.action_to_targets(action)
        for _ in range(4):
            mujoco.mj_step(self.model, d)
        cached = d.actuator_force[self.order['actuator']].copy()
        self.sim.step(action)
        self.assertAlmostEqual(self.sim.data.time, .008)
        np.testing.assert_allclose(self.sim.effort, cached, atol=1e-12)
        self.assertGreater(np.max(abs(cached-self.sim.data.actuator_force[self.order['actuator']])), 1e-4)
        self.assertIn('timeout', self.sim.termination(.008))
        self.sim.reset()
        f = np.zeros((self.model.nbody, 3))
        f[self.model.body('Arm_P2').id, 2] = 2
        self.sim.history.append(f)
        self.assertTrue(any('Arm_P2' in s for s in self.sim.termination(40)))
        self.sim.reset()
        self.sim.data.qpos[3:7] = [np.cos(.6), np.sin(.6), 0, 0]
        mujoco.mj_forward(self.model, self.sim.data)
        self.assertTrue(any('tilt' in s for s in self.sim.termination(40)))

    def test_safe_yaml_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)/'data.yaml'
            p.write_text('x: !!python/tuple [1, 2]\ny: !!python/object/apply:builtins.slice [null, null, null]\n')
            self.assertEqual(arm.read_yaml(p)['x'], (1, 2))
            p.write_text('!!python/object/apply:os.system ["exit 1"]')
            with self.assertRaises(Exception):
                arm.read_yaml(p)
            p.write_text('task=Wheeled-Arm-Play-v1\nstep_dt=0.008\njoint_names='+','.join(arm.ACTION_ORDER))
            with self.assertRaises(ValueError):
                arm.check_metadata(arm.POLICY_PATH, p)

    def test_checkpoint_normalization_and_config_drift(self):
        with redirect_stdout(io.StringIO()):
            policy = arm.Policy(arm.POLICY_PATH)
            arm.check_metadata(arm.POLICY_PATH)
        obs = self.sim.observation([0, 0, 0])
        a = policy(obs)
        np.testing.assert_array_equal(a, policy(obs))
        self.assertEqual(a.shape, (12,))
        torch = policy.torch
        expected = (torch.from_numpy(obs)[None]-policy.mean)/(policy.std+.01)
        torch.testing.assert_close(policy.normalize(obs), expected)
        with tempfile.TemporaryDirectory() as directory:
            d = Path(directory)
            env = (policy.params_dir/'env.yaml').read_text().replace('scale: 0.15', 'scale: 0.25')
            (d/'env.yaml').write_text(env)
            (d/'agent.yaml').write_bytes((policy.params_dir/'agent.yaml').read_bytes())
            with self.assertRaises(ValueError):
                arm.Policy(arm.POLICY_PATH, d)


if __name__ == '__main__':
    unittest.main()
