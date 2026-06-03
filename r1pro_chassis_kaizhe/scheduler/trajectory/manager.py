from core.communication.message_queue import MessageQueue
from utils.message.datatype import RobotAction, Trajectory
from utils.message.message_convert import actions_dict_to_trajectory, actions_dict_to_array, array_to_action, get_action_time
from scheduler.trajectory.stitcher import TrajectoryStitcher
from utils.message.datatype import ExecutionMode
from enum import Enum
from typing import Optional, Literal
import threading
import time
from loguru import logger
import numpy as np
from scheduler.trajectory.hato import ensemble
class EnsembleMode(Enum):
    NONE = "NONE"
    HATO = "HATO"
    RTG = "RTG"
    RTC = "RTC"

MAX_ACTIONS_QUEUE_LENGTH = {
    EnsembleMode.NONE: 1,
    EnsembleMode.HATO: 4,
    EnsembleMode.RTG: 1,
    EnsembleMode.RTC: 1,
}

class TrajectoryManager:
    def __init__(
        self,
        ensemble_mode: Optional[EnsembleMode] = EnsembleMode.NONE,
        execution_mode: Optional[ExecutionMode] = ExecutionMode.JOINT_STATE,
        dt: float = 1 / 15,
        num_of_steps: int = 32,
        tau_hato: float = 0.4,
    ):
        self.ensemble_mode = ensemble_mode
        self.execution_mode = execution_mode
        self.tau_hato = tau_hato
        logger.info(f"Ensemble mode: {ensemble_mode}, execution mode: {execution_mode}, tau_hato: {tau_hato}")
        self.max_actions_queue_length = MAX_ACTIONS_QUEUE_LENGTH[ensemble_mode]
        self.actions_queue = MessageQueue(maxlen=self.max_actions_queue_length)
        
        self.trajectory = None
        self.stitcher = TrajectoryStitcher(execution_mode=self.execution_mode)

        self.is_actions_queue_updated = False
        # self.traj_worker_stop_event = threading.Event()
        # self.traj_worker = threading.Thread(target=self._traj_worker, daemon=True)
        self.action = None
        self.dt = dt
        self.num_of_steps = num_of_steps
        self.mode = "next"

        self.chunk_id = 0
        self.step_in_chunk = 0

    def start(self):
        pass
        # self.traj_worker.start()

    def stop(self):
        pass
        # self.traj_worker_stop_event.set()
        # self.traj_worker.join()

    def __del__(self):
        self.stop()

    def add_actions(self, actions: dict, obs_time: float = None):
        old_chunk = self.chunk_id
        old_step = self.step_in_chunk
        self.chunk_id += 1
        self.step_in_chunk = 0
        now = time.time()
        lag = now - obs_time if obs_time else 0
        anchor_time = obs_time if obs_time else now
        obs_actions_dict = {
            "obs_time": anchor_time,
            "actions": actions
        }
        self.actions_queue.append(obs_actions_dict)
        if self.ensemble_mode in [EnsembleMode.NONE, EnsembleMode.RTC]:
            logger.info(
                f'NEW chunk#{self.chunk_id}  '
                f'prev_chunk#{old_chunk} executed {old_step}/{self.num_of_steps} steps  '
                f'lag={lag:.2f}s'
            )
            self._generate_trajectory(timestamp=now)
        elif self.ensemble_mode == EnsembleMode.HATO:
            logger.info(
                f'HATO chunk#{self.chunk_id}  '
                f'prev_chunk#{old_chunk} executed {old_step} steps  '
                f'lag={lag:.2f}s  queue={len(self.actions_queue)}/{self.max_actions_queue_length}'
            )
        self.is_actions_queue_updated = True

    def get_action(self, timestamp: float = None) -> RobotAction:
        if self.ensemble_mode == EnsembleMode.HATO:
            ensembled_action = self._generate_trajectory(timestamp=timestamp)
            if ensembled_action is not None:
                self.step_in_chunk += 1
            return ensembled_action
        elif self.ensemble_mode == EnsembleMode.RTG:
            self.action = self.trajectory.actions.popleft()
            return self.action
        elif self.ensemble_mode == EnsembleMode.NONE:
            actions_length = len(self.trajectory.actions)
            for _ in range(actions_length):
                action = self.trajectory.actions.popleft()
                action_time = get_action_time(action)
                if timestamp < action_time:
                    self.step_in_chunk += 1
                    return action
        elif self.ensemble_mode == EnsembleMode.RTC:
            actions_length = len(self.trajectory.actions)
            for _ in range(actions_length):
                action = self.trajectory.actions.popleft()
                action_time = get_action_time(action)
                if timestamp < action_time:
                    return action
        else:
            raise NotImplementedError

    def get_last_action(self, timestamp: float = None):
        return self.action
    
    def is_ready(self):
        if self.ensemble_mode == EnsembleMode.RTG:
            return self.trajectory is not None and len(self.trajectory.actions) > 0
        else:
            return self.is_actions_queue_updated

    def _generate_trajectory(self, timestamp: float = None) -> Trajectory:
        if self.ensemble_mode == EnsembleMode.NONE:
            if len(self.actions_queue) == 0:
                return None
            actions = self.actions_queue.popleft()["actions"]
            self.trajectory = actions_dict_to_trajectory(
                actions=actions,
                time_step=self.dt,
                num_of_steps=self.num_of_steps,
                timestamp=timestamp,
            )
        elif self.ensemble_mode == EnsembleMode.HATO:
            idxs = []
            raw_actions = []
            snapshot = list(self.actions_queue)
            for chunk in snapshot:
                obs_time = chunk["obs_time"]
                chunk_actions = chunk["actions"]
                actions_time = obs_time + self.dt * np.arange(self.num_of_steps)
                mask = actions_time > timestamp
                if not np.any(mask):
                    continue
                idx = int(np.argmax(mask))
                idxs.append(idx)
                action_array = actions_dict_to_array(chunk_actions, self.execution_mode)[0]
                if self.mode == "next":
                    raw_actions.append(action_array[idx])
                elif self.mode == "interp":
                    if idx == 0:
                        raw_actions.append(action_array[0])
                    else:
                        t_frac = (timestamp - actions_time[idx - 1]) / (actions_time[idx] - actions_time[idx - 1])
                        raw_actions.append(action_array[idx - 1] + t_frac * (action_array[idx] - action_array[idx - 1]))
                else:
                    raise NotImplementedError
            logger.debug(f'HATO use {idxs}')
            
            if len(raw_actions) == 0:
                return None

            ensembled_action = ensemble(raw_actions, self.execution_mode, tau_hato=self.tau_hato)

            return array_to_action(ensembled_action, self.execution_mode)

        elif self.ensemble_mode == EnsembleMode.RTG:
            if self.trajectory is None or len(self.trajectory.actions) == 0:
                self.trajectory = actions_dict_to_trajectory(
                    actions=self.actions_queue.popleft()["actions"],
                    time_step=self.dt,
                    num_of_steps=self.num_of_steps,
                    timestamp=timestamp,
                )
            elif len(self.trajectory.actions) < 16:
                new_traj = actions_dict_to_trajectory(
                    actions=self.actions_queue.popleft()["actions"],
                    time_step=self.dt,
                    num_of_steps=self.num_of_steps,
                    timestamp=timestamp,
                )
                self.trajectory = self.stitcher.stitch(self.trajectory, new_traj)
        elif self.ensemble_mode == EnsembleMode.RTC:
            if len(self.actions_queue) == 0:
                return None
        else:
            logger.info(f'Not implement')

    # def _traj_worker(self):
    #     while not self.traj_worker_stop_event.is_set():
    #         if self.is_actions_queue_updated:
    #             self._generate_trajectory(timestamp=time.time())
    #             self.is_actions_queue_updated = False
    #         time.sleep(0.01)

