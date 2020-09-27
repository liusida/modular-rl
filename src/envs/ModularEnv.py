import numpy as np
from gym import utils
# from gym.envs.mujoco import mujoco_env
from utils import *
import os

import pybullet
from pybullet_envs.gym_locomotion_envs import WalkerBaseBulletEnv
from pybullet_envs.robot_locomotors import WalkerBase
import pybullet_data

from time import sleep

class ModularRobot(WalkerBase):
    foot_list = ["torso"] # this is for detecting whether this part contacts with floor

    def __init__(self, xml):
        WalkerBase.__init__(self, xml, "torso", action_dim=0, obs_dim=0, power=0.75) # action_dim and obs_dim will be set automatically in function reset_spaces()
        self.body_parts = None

    def alive_bonus(self, z, pitch):
        # return +1 if z > self.initial_z/2 and abs(pitch) < 0.6 else -1
        return +1 if z>self.initial_z/2 else -1

    def calc_state(self):
        s = super().calc_state()

        def _get_obs_per_limb(part, joint, idx):
            b, link = part
            # instead of manually specify the mask for certain parts, simply pass the index for the module to identify itself.
            # might be not very effective comparing to one-hot, but support different number of limbs
            limb_type_vec = np.array([idx])
            limb_type_vec = []
            torso_x_pos = self.parts['torso'].current_position()[0] # WHY?
            xpos = link.current_position()
            xpos[0] -= torso_x_pos
            q_xyzw = link.current_orientation()
            q_wxyz = [q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]]
            expmap = quat2expmap(q_wxyz)
            xvel = link.speed()
            obs = np.concatenate([xpos, np.clip(xvel, -10, 10), [0,0,0], expmap, limb_type_vec]) # missing rotational velocity
               
            # include current joint angle and joint range as input
            if b == 'torso':
                angle = 0.
                joint_range = [0., 0.]
            else:
                joint_range = [joint.lowerLimit, joint.upperLimit]
                joint_range = np.degrees(joint_range) # angle in degree

                angle, _ = joint.get_state()
                angle = np.degrees(angle)
                # print(b, angle, joint_range)
                angle = (angle - joint_range[0]) / (joint_range[1] - joint_range[0])

                joint_range[0] = (180. + joint_range[0]) / 360.
                joint_range[1] = (180. + joint_range[1]) / 360.

            obs = np.concatenate([obs, [angle], joint_range])
            
            # ignore above
            linkWorldPosition, linkWorldOrientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity = \
                self._p.getLinkState(1, idx, computeLinkVelocity=1)
            jointPosition, jointVelocity, jointReactionForces, appliedJointMotorTorque = self._p.getJointState(1, idx)

            obs = np.concatenate(
                [linkWorldPosition, linkWorldOrientation, localInertialFramePosition, localInertialFrameOrientation, worldLinkFramePosition, worldLinkFrameOrientation, worldLinkLinearVelocity, worldLinkAngularVelocity,
                [jointPosition], [jointVelocity], jointReactionForces, [appliedJointMotorTorque]])

            return obs

        # Lazy init self.body_parts
        if self.body_parts is None:
            link_names = ['torso']
            for joint in self.ordered_joints:
                link_names.append(joint.joint_name.replace("_joint", ""))
            self.body_parts = []
            for name in link_names:
                self.body_parts.append((name, self.parts[name]))

        full_obs = []
        for idx, part in enumerate(self.body_parts):
            if idx>0:
                full_obs.append(_get_obs_per_limb(part, self.ordered_joints[idx-1], idx))
            else:
                full_obs.append(_get_obs_per_limb(part, None, idx))
        full_obs = np.array(full_obs)
        
        return full_obs.ravel()

class ModularEnv(WalkerBaseBulletEnv, utils.EzPickle):

    def __init__(self, xml, render=False):
        self.torso_linkId = 0
        self.robot = ModularRobot(xml)
        self.xml = xml
        WalkerBaseBulletEnv.__init__(self, self.robot, render=render)
        utils.EzPickle.__init__(self)

        self.best_record_so_far = 0.
        self.current_step = 0

        self.reset()
        self.adjust_visual()
        self.reset_spaces()
        self.step(self.action_space.sample())


    def reset(self):
        self.current_step = 0
        return super().reset()

    def reset_spaces(self):
        """After reset, we have the number of joints and body parts, so we can calculate action_space and observation_space"""
        action_dim = len(self.robot.ordered_joints)
        obs_dim = (action_dim + 1 ) * 36

        high = np.ones([action_dim])
        self.action_space = gym.spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim])
        self.observation_space = gym.spaces.Box(-high, high)
    
    def adjust_visual(self):
        self.camera_adjust()
        num_joints = self._p.getNumJoints(1)
        for link in range(1,num_joints):
            jointInfo = self._p.getJointInfo(1, link)
            if jointInfo[12]==b"torso": # make "torso"(root) red
                self.torso_linkId = link
                self._p.changeVisualShape(1,link,rgbaColor=[1, 0, 0, 1])
            else: # other parts yellow
                self._p.changeVisualShape(1,link,rgbaColor=[1, 1, 0.6, 1])

    def step(self, a):
        if self.isRender:
            sleep(0.03)
            pass
        self.current_step += 1

        # posbefore = self.sim.data.qpos[0]
        posbefore = self.robot.body_xyz[0]
        # self.do_simulation(a, self.frame_skip)
        a = np.array(a)
        # print(f"Action: {a}")

        observation, old_r, old_done, _ = super().step(a)
        # reward and done from default step will be discarded. we reconstruct them below.
        # print(f"Obs: {observation}")

        # posafter = self.sim.data.qpos[0]
        posafter = self.robot.body_xyz[0]
        # torso_height, torso_ang = self.sim.data.qpos[1:3]
        torso_height = self.robot.body_xyz[2]
        torso_ang = self.robot.body_rpy[2]

        upside_down = False
        link = self._p.getLinkState(1, self.torso_linkId)
        quaternion_i = link[1][1]
        if quaternion_i<-0.7 or quaternion_i>0.7:
            upside_down = True
        done = not (np.isfinite(observation).all() and (np.abs(observation[2:]) < 100).all()) or self.robot.feet_contact[0] or upside_down
        max_episode_steps = 1e+10
        try:
            max_episode_steps = self.spec.max_episode_steps
        except:
            pass
        # print("done ", done, "max_episode_steps ", max_episode_steps, "self.current_step: ", self.current_step)
        if self.current_step >= max_episode_steps:
            done = True

        use_old_reward_function = False
        if use_old_reward_function:
            alive_bonus = 1.0
            # reward = (posafter - posbefore) / self.dt
            reward = (posafter - posbefore) / self.stadium_scene.dt
            reward += alive_bonus
            reward -= 1e-3 * np.square(a).sum()
        else: # Since we cannot use default Gym reward function, let's come up with one new reward function for all tasks.
            reward = 0.
            if done:
                if posafter>self.best_record_so_far:
                    reward += 10. # for breaking record
                    self.best_record_so_far = posafter
                else:
                    reward -= 10. # for falling down
                print(f"{self.spec.id} done. Traveled distance: This episode: {posafter:.02f}, Best so far: {self.best_record_so_far:.02f}")
            reward += (posafter - posbefore) / self.stadium_scene.dt # for speed
            reward -= 1e-3 * np.square(a).sum() # for energy usage
        return observation, reward, done, {"x":posafter}

    def _get_obs(self):
        return self.robot.calc_state()

    def reset_model(self):
        self.set_state(
            self.init_qpos + self.np_random.uniform(low=-.005, high=.005, size=self.model.nq),
            self.init_qvel + self.np_random.uniform(low=-.005, high=.005, size=self.model.nv)
        )
        return self._get_obs()
