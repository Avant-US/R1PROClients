from core.inference.factory import create_inference_engine
from core.processor.factory import create_processor
from core.communication.ros2_bridge import Ros2Bridge
from utils.message.message_convert import actions_dict_to_trajectory 

from scheduler.instruction.instruction import InstructionManager, InstructionAction
from scheduler.trajectory.manager import TrajectoryManager, EnsembleMode
from utils.message.datatype import Trajectory, ExecutionMode
from std_msgs.msg import String
from loguru import logger

import toml
import torch
import time
import numpy as np
from omegaconf import OmegaConf
from pathlib import Path

try:
    from galaxea_fm.utils.config_resolvers import register_default_resolvers
except ImportError:
    import math

    def register_default_resolvers():
        # Keep websocket/openpi mode usable on platforms where galaxea_fm
        # and its full dependency stack are unavailable.
        OmegaConf.register_new_resolver("oc.load", lambda path, key=None: OmegaConf.select(OmegaConf.load(path), key) if key else OmegaConf.load(path), replace=True)
        OmegaConf.register_new_resolver("eval", eval, replace=True)
        OmegaConf.register_new_resolver("split", lambda s, idx: s.split('/')[int(idx)], replace=True)
        OmegaConf.register_new_resolver("max", lambda x: max(x), replace=True)
        OmegaConf.register_new_resolver("round_up", math.ceil, replace=True)
        OmegaConf.register_new_resolver("round_down", math.floor, replace=True)
        OmegaConf.register_new_resolver("sum_shapes", lambda shape_meta_list: sum(int(item["shape"]) for item in shape_meta_list) if shape_meta_list else 0, replace=True)

register_default_resolvers()

from accelerate import PartialState
distributed_state = PartialState()

class Scheduler:
    def __init__(self, config):
        self.schedule_config = config
        self.use_websocket = self.schedule_config.get("websocket", {}).get("use_websocket", False)
        # openpi 和 rldx1 都走「直接 dict 进、dict 出，不在客户端 unsqueeze batch」的 WebSocket 路径，
        # 这里复用同一个 flag。
        _remote_processors = ("openpi", "rldx1")
        self.use_openpi = self.use_websocket and self.schedule_config["model"].get("processor", "").lower() in _remote_processors
        if self.use_openpi:
            self.model_config = OmegaConf.create({})
        else:
            self.model_config = OmegaConf.load(f"{self.schedule_config['model']['ckpt_dir']}/config.yaml")

        self.cnt = 0
        self.step_mode = self.schedule_config['basic']['step_mode']
        self.step_freq = self.schedule_config['basic']['control_frequency']
        self.num_of_steps = self.schedule_config['basic']['action_steps']

        self.task_status = "idle"       # "idle" / "executing" / "completed" / "failed"
        self.task_message = ""
        self.task_step_count = 0
        self.task_start_time = 0.0
        self._task_timeout = 120.0      # 超时秒数，可在 start_task 时覆盖
        self._idle_since = 0.0          # 机器人状态开始不变的时间点
        self._idle_threshold = 5.0      # 持续多少秒不动算完成
        self._state_delta_threshold = 0.005  # 手臂关节变化小于此值视为"不动"
        self._chassis_delta_threshold = 0.015  # 底盘允许更大的微调噪声
        self._prev_state = None         # 上一次的机器人观测状态
        self._stuck_window: list[torch.Tensor] = []   # 左手关节位置滑动窗口
        self._stuck_window_size = 50                   # 窗口大小（步数）
        self._stuck_range_threshold = 0.03             # 窗口内关节极差低于此值视为卡住
        self._stuck_min_elapsed = 10.0                 # 至少运行这么久才开始检测卡住
        self._recording = False
        self._recorded_obs = []         # 录制的 obs state 列表
        self._recorded_actions = []     # 录制的 action 列表

        self._setup_all()
        self.instruction_manager.text_instruction_file.write_text("nothing")

    def start_task(self, instruction: str, timeout: float = 120.0):
        """由 HTTP 接口调用，写入指令并重置状态计数。"""
        self.instruction_manager.text_instruction_file.write_text(instruction)
        self.task_status = "executing"
        self.task_message = "正在执行"
        self.task_step_count = 0
        self.task_start_time = time.time()
        self._task_timeout = timeout
        self._idle_since = 0.0
        self._stuck_window.clear()
        self._prev_state = None
        # rldx1 等需要维护历史帧的 processor 在任务开始时清空 buffer
        reset_history = getattr(self.processor, "reset_history", None)
        if callable(reset_history):
            reset_history()
        logger.info(f"Task started: {instruction}")

    def stop_task(self):
        """由 HTTP 接口调用，清空指令使机器人停止。"""
        self.instruction_manager.text_instruction_file.write_text("nothing")
        self.task_status = "idle"
        self.task_message = "已停止"
        logger.info("Task stopped")

    def _finish_task(self, status: str, message: str):
        """内部调用，标记任务结束并清空指令。"""
        self.instruction_manager.text_instruction_file.write_text("nothing")
        self.task_status = status
        self.task_message = message
        logger.info(f"Task finished: status={status}, message={message}")

    def get_task_status(self) -> dict:
        if self.task_status in ("idle", "executing"):
            return {"done": False}
        return {"done": True, "success": self.task_status == "completed", "message": self.task_message}

    def start_recording(self):
        self._recorded_obs = []
        self._recorded_actions = []
        self._recording = True
        logger.info("Recording started")

    def stop_recording(self) -> dict:
        self._recording = False
        result = {
            "count": len(self._recorded_obs),
            "obs": self._recorded_obs,
            "actions": self._recorded_actions,
        }
        logger.info(f"Recording stopped, {len(self._recorded_obs)} frames captured")
        return result

    def _record_frame(self, obs: dict, actions: dict):
        if not self._recording:
            return
        _PARTS = ("left_arm", "right_arm", "left_gripper", "right_gripper", "chassis")
        obs_frame = {"time": time.time()}
        for part in _PARTS:
            t = obs["state"].get(part)
            if t is not None:
                obs_frame[part] = t.detach().float().flatten().tolist()

        action_frame = {"time": time.time()}
        for part in _PARTS:
            t = actions.get(part)
            if t is not None:
                action_frame[part] = t.detach().float().squeeze(0)[0].tolist()

        self._recorded_obs.append(obs_frame)
        self._recorded_actions.append(action_frame)

    def _check_task_done(self, obs: dict) -> None:
        now = time.time()
        elapsed = now - self.task_start_time

        if elapsed > self._task_timeout:
            self._finish_task("failed", f"执行超时 ({self._task_timeout:.0f}s)")
            return

        _ARM_PARTS = ("left_arm", "right_arm")
        _CHASSIS_PART = "chassis"
        current = {}
        for part in (*_ARM_PARTS, _CHASSIS_PART):
            t = obs["state"].get(part)
            if t is not None:
                current[part] = t.detach().float()

        if self._prev_state is None:
            self._prev_state = current
            return

        arm_delta = 0.0
        for part in _ARM_PARTS:
            if part in current and part in self._prev_state:
                delta = (current[part] - self._prev_state[part]).abs().max().item()
                if delta > arm_delta:
                    arm_delta = delta

        chassis_delta = 0.0
        if _CHASSIS_PART in current and _CHASSIS_PART in self._prev_state:
            chassis_delta = (current[_CHASSIS_PART] - self._prev_state[_CHASSIS_PART]).abs().max().item()

        self._prev_state = current

        # --- stuck detection (repetitive action on left arm) ---
        left = current.get("left_arm")
        if left is not None:
            self._stuck_window.append(left.flatten())
            if len(self._stuck_window) > self._stuck_window_size:
                self._stuck_window.pop(0)

            if (elapsed > self._stuck_min_elapsed
                    and len(self._stuck_window) == self._stuck_window_size):
                stacked = torch.stack(self._stuck_window)
                joint_range = (stacked.max(dim=0).values - stacked.min(dim=0).values).max().item()
                if joint_range < self._stuck_range_threshold and arm_delta > 0:
                    self._finish_task("failed",
                        f"检测到左手重复动作 (range={joint_range:.4f})，判定为卡住")
                    return

        # --- idle detection ---
        is_idle = (arm_delta < self._state_delta_threshold and
                   chassis_delta < self._chassis_delta_threshold)

        if is_idle:
            if self._idle_since == 0.0:
                self._idle_since = now
                logger.info(f"Robot stable (arm={arm_delta:.6f}, chassis={chassis_delta:.6f}), starting idle timer")
            elif now - self._idle_since >= self._idle_threshold:
                self._finish_task("completed", "执行完成")
                return
        else:
            if self._idle_since != 0.0:
                logger.info(f"Robot moving (arm={arm_delta:.6f}, chassis={chassis_delta:.6f}), resetting idle timer")
            self._idle_since = 0.0

    def run(self):
        while self.ros2_bridge.is_running():
            obs_time, obs = self.ros2_bridge.gather_obs()
            infer_start = time.time()
            actions = self.inference(obs)
            infer_cost = time.time() - infer_start
            if actions is not None and self.cnt >= 2:
                logger.info(f'Infer cost: {infer_cost}')
                self.step(actions['action'], obs_time)
                self._record_frame(obs, actions['action'])
                if self.task_status == "executing":
                    self.task_step_count += 1
                    self._check_task_done(obs)
            self.cnt += 1

    def inference(self, obs):
        if obs is None:
            if self.cnt % 100 == 0:
                logger.info("No observation")
            time.sleep(0.01)
            return

        instruct_action = self.instruction_manager.get_instruction(obs)
        if instruct_action == InstructionAction.RESET:
            self.ros2_bridge.reset()
            return
        elif instruct_action == InstructionAction.CONTINUE:
            pass
        elif instruct_action == InstructionAction.SKIP:
            # 让出 GIL 给 ros2_bridge executor 子线程, 否则它收不到回调, buffer 不更新
            time.sleep(0.01)
            return

        t0 = time.time()
        batch = self.processor.preprocess(obs)
        t1 = time.time()
        if self.use_openpi:
            response = self.inference_engine.predict_action(batch)
            t2 = time.time()
            result = self.processor.postprocess(response)
            t3 = time.time()
 
            client_total = 1000 * (t2 - t1)
            policy_timing = response.get("policy_timing", {})
            server_timing = response.get("server_timing", {})
            policy_infer = policy_timing.get("infer_ms", 0)
            server_infer = server_timing.get("infer_ms", 0)
            client_minus_server = client_total - server_infer
            server_minus_policy = server_infer - policy_infer

            logger.info(
                f'preprocess={1000*(t1-t0):.1f}ms  '
                f'postprocess={1000*(t3-t2):.1f}ms  |  '
                f'client_total={client_total:.1f}ms  '
                f'policy_infer={policy_infer:.1f}ms  '
                f'server_infer={server_infer:.1f}ms  '
                f'network={client_minus_server:.1f}ms  '
                f'server_overhead={server_minus_policy:.1f}ms'
            )
            return result

        for k, v in batch.items():
            if isinstance(v, str):
                batch[k] = [v]
            elif isinstance(v, bool):
                batch[k] = torch.tensor([v])
            else:
                batch[k] = v.unsqueeze(0)
        batch = self.inference_engine.predict_action(batch)
        batch["action"] = batch["action"].cpu()
        batch["proprio"] = batch["proprio"].cpu()
        actions = self.processor.postprocess(batch)
        return actions

    def step(self, actions: dict, obs_time: float):
        if self.step_mode == "sync":
            trajectory = actions_dict_to_trajectory(actions=actions, time_step=1/self.step_freq, num_of_steps=self.num_of_steps, timestamp=self.ros2_bridge.now())
            if len(trajectory.actions) < self.num_of_steps:
                raise ValueError(f"Trajectory actions length {len(trajectory.actions)} is less than num_of_steps {self.num_of_steps}")

            self._sync_publish(trajectory)

        elif self.step_mode == "async":
            logger.info(f'Add actions to trajectory manager.')
            self.trajectory_manager.add_actions(actions, obs_time)
        else:
            raise ValueError(f"Invalid step mode: {self.step_mode}")

    def _sync_publish(self, trajectory: Trajectory):
        for i in range(self.num_of_steps):
            self.ros2_bridge.publish_action(trajectory.actions[i])
            time.sleep(1.0 / self.step_freq)

    @logger.catch
    def _async_publish(self):
        if not self.trajectory_manager.is_ready():
            return
        now = time.time()
        action = self.trajectory_manager.get_action(now)
        if action is None:
            return
        self.ros2_bridge.publish_action(action)

    def _setup_all(self):
        self._setup_processor()
        self._setup_trajectory_manager()
        self._setup_instruction_manager()
        self._setup_ros2_bridge()
        self._setup_inference_engine()

    def _setup_inference_engine(self):
        self.inference_engine = create_inference_engine(self.schedule_config, self.model_config, use_trt=self.schedule_config['model']['use_trt'])
        self.inference_engine.load_model()

    def _setup_processor(self):
        self.processor = create_processor(self.schedule_config, self.model_config, processor_type=self.schedule_config['model']['processor'])
        if self.use_openpi:
            self.processor.initialize(None)
        else:
            self.processor.initialize(Path(f"{self.schedule_config['model']['ckpt_dir']}/dataset_stats.json"))

    def _setup_trajectory_manager(self):
        if self.schedule_config['trajectory']['ensemble_mode'] == "RTC":
            ensemble_mode = EnsembleMode.RTC
        elif self.schedule_config['trajectory']['ensemble_mode'] == "RTG":
            ensemble_mode = EnsembleMode.RTG
        elif self.schedule_config['trajectory']['ensemble_mode'] == "HATO":
            ensemble_mode = EnsembleMode.HATO
        else:
            logger.warning(f"Invalid ensemble mode:{self.schedule_config['trajectory']['ensemble_mode']}")
            ensemble_mode = EnsembleMode.NONE
        
        if self.schedule_config['trajectory']['execution_mode'] == "JOINT_STATE":
            execution_mode = ExecutionMode.JOINT_STATE
        elif self.schedule_config['trajectory']['execution_mode'] == "EE_POSE":
            execution_mode = ExecutionMode.EE_POSE
        else:
            raise ValueError(f"Invalid execution mode: {self.schedule_config['trajectory']['execution_mode']}")
        
        tau_hato = self.schedule_config['trajectory'].get('tau_hato', 0.4)
        self.trajectory_manager = TrajectoryManager(
            ensemble_mode=ensemble_mode,
            execution_mode=execution_mode,
            dt=1 / self.step_freq,
            num_of_steps=self.num_of_steps,
            tau_hato=tau_hato,
        )
        self.trajectory_manager.start()

    def _setup_instruction_manager(self):
        self.instruction_manager = InstructionManager(self.schedule_config["instruction"])

    def _setup_ros2_bridge(self):
        # HACK: use_recv_time=True to use the received time from ROS2 messages
        self.ros2_bridge = Ros2Bridge(self.schedule_config, self.model_config, use_recv_time=True)
        self.ros2_bridge.register_subscription(String, 'hs/vlm_out2vla', self.instruction_manager._ehi_instruction_callback)
        
        if self.step_mode == "async":
            self.ros2_bridge.register_publish_callback(self.step_freq, self._async_publish)
 

if __name__ == "__main__":
    config = toml.load("config.toml")
    scheduler = Scheduler(config)
    scheduler.run()
