"""
This file contains the robosuite environment wrapper that is used
to provide a standardized environment API for training policies and interacting
with metadata present in datasets.
"""
import json
import numpy as np
from copy import deepcopy

import mujoco_py
import robosuite
from robosuite.utils.mjcf_utils import postprocess_model_xml
from robosuite.utils.camera_utils import get_camera_intrinsic_matrix, get_real_depth_map, get_xyz_from_depth

import robomimic.utils.obs_utils as ObsUtils
import robomimic.envs.env_base as EB
import robosuite.utils.transform_utils as T


class EnvRobosuite(EB.EnvBase):
    """Wrapper class for robosuite environments (https://github.com/ARISE-Initiative/robosuite)"""
    def __init__(
        self, 
        env_name, 
        render=False, 
        render_offscreen=False, 
        use_image_obs=False, 
        postprocess_visual_obs=True, 
        use_rgb_camera=False,
        use_depth_camera=False,
        use_pcd_camera=False, 
        use_segmentation_camera=False,
        args={},
        **kwargs,
    ):
        """
        Args:
            env_name (str): name of environment. Only needs to be provided if making a different
                environment from the one in @env_meta.

            render (bool): if True, environment supports on-screen rendering

            render_offscreen (bool): if True, environment supports off-screen rendering. This
                is forced to be True if @env_meta["use_images"] is True.
                
            use_image_obs (bool): if True, environment is expected to render rgb image observations
                on every env.step call. Set this to False for efficiency reasons, if image
                observations are not required.
                
            postprocess_visual_obs (bool): if True, postprocess image observations
                to prepare for learning. This should only be False when extracting observations
                for saving to a dataset (to save space on RGB images for example).
        """
        self.postprocess_visual_obs = postprocess_visual_obs
        self.use_rgb_camera = use_rgb_camera
        self.use_depth_camera = use_depth_camera
        self.use_pcd_camera = use_pcd_camera
        self.use_segmentation_camera = use_segmentation_camera

        # robosuite version check
        self._is_v1 = (robosuite.__version__.split(".")[0] == "1")
        if self._is_v1:
            assert (int(robosuite.__version__.split(".")[1]) >= 2), "only support robosuite v0.3 and v1.2+"

        kwargs = deepcopy(kwargs)

        # update kwargs based on passed arguments
        update_kwargs = dict(
            has_renderer=render,
            has_offscreen_renderer=(render_offscreen or use_image_obs),
            ignore_done=True,
            use_object_obs=True,
            use_camera_obs=use_image_obs,
            camera_depths=self.use_depth_camera or self.use_pcd_camera,
        )
        kwargs.update(update_kwargs)
        

        if self._is_v1:
            if kwargs["has_offscreen_renderer"]:
                # ensure that we select the correct GPU device for rendering by testing for EGL rendering
                # NOTE: this package should be installed from this link (https://github.com/StanfordVL/egl_probe)
                import egl_probe
                valid_gpu_devices = egl_probe.get_available_devices()
                if len(valid_gpu_devices) > 0:
                    kwargs["render_gpu_device_id"] = valid_gpu_devices[0]
        else:
            # make sure gripper visualization is turned off (we almost always want this for learning)
            kwargs["gripper_visualization"] = False
            del kwargs["camera_depths"]
            kwargs["camera_depth"] = False # rename kwarg

        self._env_name = env_name
        self._init_kwargs = deepcopy(kwargs)
        kwargs['args'] = args
        self.env = robosuite.make(self._env_name, **kwargs)

        if self._is_v1:
            # Make sure joint position observations and eef vel observations are active
            for ob_name in self.env.observation_names:
                if ("joint_pos" in ob_name) or ("eef_vel" in ob_name):
                    self.env.modify_observable(observable_name=ob_name, attribute="active", modifier=True)

    def step(self, action):
        """
        Step in the environment with an action.

        Args:
            action (np.array): action to take

        Returns:
            observation (dict): new observation dictionary
            reward (float): reward for this step
            done (bool): whether the task is done
            info (dict): extra information
        """
        obs, r, done, info = self.env.step(action)
        obs = self.get_observation(obs)
        return obs, r, self.is_done(), info

    def reset(self):
        """
        Reset environment.

        Returns:
            observation (dict): initial observation dictionary.
        """
        di = self.env.reset()
        return self.get_observation(di)

    def reset_to(self, state):
        """
        Reset to a specific simulator state.

        Args:
            state (dict): current simulator state that contains one or more of:
                - states (np.ndarray): initial state of the mujoco environment
                - model (str): mujoco scene xml
        
        Returns:
            observation (dict): observation dictionary after setting the simulator state (only
                if "states" is in @state)
        """
        should_ret = False
        if "model" in state:
            self.reset()
            xml = postprocess_model_xml(state["model"])
            self.env.reset_from_xml_string(xml)
            self.env.sim.reset()
            if not self._is_v1:
                # hide teleop visualization after restoring from model
                self.env.sim.model.site_rgba[self.env.eef_site_id] = np.array([0., 0., 0., 0.])
                self.env.sim.model.site_rgba[self.env.eef_cylinder_id] = np.array([0., 0., 0., 0.])
        if "states" in state:
            self.env.sim.set_state_from_flattened(state["states"])
            self.env.sim.forward()
            should_ret = True

        if "goal" in state:
            self.set_goal(**state["goal"])
        if should_ret:
            # only return obs if we've done a forward call - otherwise the observations will be garbage
            return self.get_observation()
        return None

    def render(self, mode="human", height=None, width=None, camera_name="agentview"):
        """
        Render from simulation to either an on-screen window or off-screen to RGB array.

        Args:
            mode (str): pass "human" for on-screen rendering or "rgb_array" for off-screen rendering
            height (int): height of image to render - only used if mode is "rgb_array"
            width (int): width of image to render - only used if mode is "rgb_array"
            camera_name (str): camera name to use for rendering
        """
        if mode == "human":
            cam_id = self.env.sim.model.camera_name2id(camera_name)
            self.env.viewer.set_camera(cam_id)
            return self.env.render()
        elif mode == "rgb_array":
            return self.env.sim.render(height=height, width=width, camera_name=camera_name)[::-1]
        else:
            raise NotImplementedError("mode={} is not implemented".format(mode))

    def get_observation(self, di=None):
        """
        Get current environment observation dictionary.

        Args:
            di (dict): current raw observation dictionary from robosuite to wrap and provide 
                as a dictionary. If not provided, will be queried from robosuite.
        """
        if di is None:
            di = self.env._get_observations(force_update=True) if self._is_v1 else self.env._get_observation()
        ret = {}
        
        if ObsUtils.OBS_KEYS_TO_MODALITIES is None:
            pass
        else:
            for k in di:
                if (k in ObsUtils.OBS_KEYS_TO_MODALITIES) and \
                    (ObsUtils.key_is_obs_modality(key=k, obs_modality="rgb") or \
                        ObsUtils.key_is_obs_modality(key=k, obs_modality="depth")):
                    ret[k] = di[k][::-1]
                    if self.postprocess_visual_obs:
                        ret[k] = ObsUtils.process_obs(obs=ret[k], obs_key=k)

        # "object" key contains object information
        ret["object"] = np.array(di["object-state"])

        if self._is_v1:
            for robot in self.env.robots:
                # add all robot-arm-specific observations. Note the (k not in ret) check
                # ensures that we don't accidentally add robot wrist images a second time
                pf = robot.robot_model.naming_prefix
                for k in di:
                    if k.startswith(pf) and (k not in ret) and (not k.endswith("proprio-state")):
                        ret[k] = np.array(di[k])
        else:
            # minimal proprioception for older versions of robosuite
            ret["proprio"] = np.array(di["robot-state"])
            ret["eef_pos"] = np.array(di["eef_pos"])
            ret["eef_quat"] = np.array(di["eef_quat"])
            ret["gripper_qpos"] = np.array(di["gripper_qpos"])

        # Adding 3D (x,y,z) points to observation dict.
        #NOTE(VS)# obs keys are: any keys in ObsUtils.OBS_KEYS_TO_MODALITIES i.e. are images, object-state, and anything 
        ## that comes from robot{i} (in that order) hence, agentview_depth is skipped 
        ## TODO(VS) cleanup comment
        for cam_name, cam_height, cam_width in zip(self.env.camera_names, self.env.camera_heights, self.env.camera_widths):
            if f"{cam_name}_image" in di:
                if f"{cam_name}_image" not in ret and self.use_rgb_camera:
                    ret[f"{cam_name}_image"] = di[f"{cam_name}_image"]
            if f"{cam_name}_depth" in di:
                if f"{cam_name}_depth" not in ret and self.use_depth_camera:
                    ret[f"{cam_name}_depth"] = di[f"{cam_name}_depth"]
                if self.use_pcd_camera:
                    ret[f"{cam_name}_xyz"] = get_xyz_from_depth(self.env.sim, di[f"{cam_name}_depth"], cam_name, cam_height, cam_width)
                    if self.postprocess_visual_obs:
                        ret[f"{cam_name}_xyz"] = ObsUtils.process_obs(obs=ret[f"{cam_name}_xyz"], obs_modality='pcd')
            if f"{cam_name}_segmentation_instance" in di:
                if f"{cam_name}_segmentation_instance" not in ret and self.use_segmentation_camera:
                    ret[f"{cam_name}_segmentation_instance"] = di[f"{cam_name}_segmentation_instance"]
            if f"{cam_name}_segmentation_class" in di:
                if f"{cam_name}_segmentation_class" not in ret and self.use_segmentation_camera:
                    ret[f"{cam_name}_segmentation_class"] = di[f"{cam_name}_segmentation_class"]
            if f"{cam_name}_segmentation_element" in di:
                if f"{cam_name}_segmentation_element" not in ret and self.use_segmentation_camera:
                    ret[f"{cam_name}_segmentation_element"] = di[f"{cam_name}_segmentation_element"]
                    
        if 'Stow' in self.env.__class__.__name__:
            ret[f"cubeA_pos"] = di[f"cubeA_pos"]
            ret[f"cubeA_quat"] = di[f"cubeA_quat"]
            # ret[f"cubeA_size"] = di[f"cubeA_size"]
            for i in range(self.env.env_config['num_obj_on_shelf']): 
                if f"cubeB_{i}_pos" not in ret:
                    ret[f"cubeB_{i}_pos"] = np.array(self.env.sim.data.body_xpos[self.env.obj_body_id[f"cubeB_{i}"]])  # di[f"cubeB_{i}_pos"]
                if f"cubeB_{i}_quat" not in ret:
                    ret[f"cubeB_{i}_quat"] = T.convert_quat(self.env.sim.data.body_xquat[self.env.obj_body_id[f"cubeB_{i}"]], to="xyzw") # di[f"cubeB_{i}_quat"]
                # if f"cubeB_{i}_size" not in ret:
                #     ret[f"cubeB_{i}_size"] = di[f"cubeB_{i}_size"]

        return ret

    def get_state(self):
        """
        Get current environment simulator state as a dictionary. Should be compatible with @reset_to.
        """
        xml = self.env.sim.model.get_xml() # model xml file
        state = np.array(self.env.sim.get_state().flatten()) # simulator state
        return dict(model=xml, states=state)

    def get_reward(self):
        """
        Get current reward.
        """
        return self.env.reward()

    def get_goal(self):
        """
        Get goal observation. Not all environments support this.
        """
        return self.get_observation(self.env._get_goal())

    def set_goal(self, **kwargs):
        """
        Set goal observation with external specification. Not all environments support this.
        """
        return self.env.set_goal(**kwargs)

    def is_done(self):
        """
        Check if the task is done (not necessarily successful).
        """

        # Robosuite envs always rollout to fixed horizon.
        return False

    def is_success(self):
        """
        Check if the task condition(s) is reached. Should return a dictionary
        { str: bool } with at least a "task" key for the overall task success,
        and additional optional keys corresponding to other task criteria.
        """
        succ = self.env._check_success()
        if isinstance(succ, dict):
            assert "task" in succ
            return succ
        return { "task" : succ }

    @property
    def action_dimension(self):
        """
        Returns dimension of actions (int).
        """
        return self.env.action_spec[0].shape[0]

    @property
    def name(self):
        """
        Returns name of environment name (str).
        """
        return self._env_name

    @property
    def type(self):
        """
        Returns environment type (int) for this kind of environment.
        This helps identify this env class.
        """
        return EB.EnvType.ROBOSUITE_TYPE

    def serialize(self):
        """
        Save all information needed to re-instantiate this environment in a dictionary.
        This is the same as @env_meta - environment metadata stored in hdf5 datasets,
        and used in utils/env_utils.py.
        """
        return dict(env_name=self.name, type=self.type, env_kwargs=deepcopy(self._init_kwargs))

    @classmethod
    def create_for_data_processing(
        cls, 
        env_name, 
        camera_names, 
        camera_height, 
        camera_width, 
        reward_shaping,
        use_rgb_camera,
        use_depth_camera,
        use_pcd_camera, 
        use_segmentation_camera=False,
        **kwargs,
    ):
        """
        Create environment for processing datasets, which includes extracting
        observations, labeling dense / sparse rewards, and annotating dones in
        transitions. 
        Args:
            env_name (str): name of environment
            camera_names (list of str): list of camera names that correspond to image observations
            camera_height (int): camera height for all cameras
            camera_width (int): camera width for all cameras
            reward_shaping (bool): if True, use shaped environment rewards, else use sparse task completion rewards
        """
        is_v1 = (robosuite.__version__.split(".")[0] == "1")
        has_camera = (len(camera_names) > 0)

        new_kwargs = {
            "reward_shaping": reward_shaping,
        }

        if has_camera:
            if is_v1:
                new_kwargs["camera_names"] = list(camera_names)
                new_kwargs["camera_heights"] = camera_height
                new_kwargs["camera_widths"] = camera_width
            else:
                assert len(camera_names) == 1
                if has_camera:
                    new_kwargs["camera_name"] = camera_names[0]
                    new_kwargs["camera_height"] = camera_height
                    new_kwargs["camera_width"] = camera_width

        kwargs.update(new_kwargs)

        # also initialize obs utils so it knows which modalities are image modalities
        image_modalities = list(camera_names)
        if is_v1:
            if use_rgb_camera:
                image_modalities = ["{}_image".format(cn) for cn in camera_names]
            if use_depth_camera:
                depth_modalities = ["{}_depth".format(cn) for cn in camera_names]
            if use_pcd_camera:
                pcd_modalities = ["{}_xyz".format(cn) for cn in camera_names]
            if use_segmentation_camera:
                seg_modalities = ["{}_seg".format(cn) for cn in camera_names]

        elif has_camera:
            # v0.3 only had support for one image, and it was named "rgb"
            assert len(image_modalities) == 1
            image_modalities = ["rgb"]
        obs_modality_specs = {
            "obs": {
                "low_dim": [], # technically unused, so we don't have to specify all of them
                "rgb": image_modalities if use_rgb_camera else [],
                "depth": depth_modalities if use_depth_camera else [],
                "pcd": pcd_modalities if use_pcd_camera else [],
                "seg": seg_modalities if use_segmentation_camera else [],
            }
        }
        ObsUtils.initialize_obs_utils_with_obs_specs(obs_modality_specs)

        # note that @postprocess_visual_obs is False since this env's images will be written to a dataset
        return cls(
            env_name=env_name,
            render=False, 
            render_offscreen=has_camera, 
            use_image_obs=has_camera, 
            postprocess_visual_obs=False,
            use_depth_camera=use_depth_camera,
            use_pcd_camera=use_pcd_camera, 
            use_segmentation_camera=use_segmentation_camera,
            **kwargs,
        )

    @property
    def rollout_exceptions(self):
        """
        Return tuple of exceptions to except when doing rollouts. This is useful to ensure
        that the entire training run doesn't crash because of a bad policy that causes unstable
        simulation computations.
        """
        return (mujoco_py.builder.MujocoException)

    def __repr__(self):
        """
        Pretty-print env description.
        """
        return self.name + "\n" + json.dumps(self._init_kwargs, sort_keys=True, indent=4)
