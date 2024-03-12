import time

import numpy as np
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase


class Vehicle:
    def __init__(self, speedup=5):
        self.speedup = speedup

    def connect(self, device):
        print(f"connecting to {device=}...")
        self.master = mavutil.mavlink_connection(device)
        self.master.wait_heartbeat()
        self.boot_time = time.time()
        print(f"connected to {device=}")

    def set_mode(self, mode):
        print(f"setting mode to {mode=}...")
        mode_id = self.master.mode_mapping()[mode]
        while not self.master.wait_heartbeat().custom_mode == mode_id:
            self.master.set_mode(mode_id)
        print(f"mode set to {mode=}")

    def _arm_ctl(self, arm, force):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1 if arm else 0,  # param1 (1 to indicate arm)
            21196 if force else 0,  # param2  (all other params meaningless)
            0,
            0,
            0,
            0,
            0,
        )

    def arm(self, force=False):
        print("arming...")
        self._arm_ctl(arm=True, force=force)
        self.master.motors_armed_wait()
        print("armed")

    def disarm(self, force=False):
        print("disarming...")
        self._arm_ctl(arm=False, force=force)
        self.master.motors_disarmed_wait()
        print("disarmed")

    def takeoff(self):
        print("takeoff...")
        self.set_mode("TAKEOFF")
        self.master.recv_match(condition="VFR_HUD.groundspeed>3", blocking=True)
        print("takeoff complete")

    def _set_attitude_target(self, roll, pitch, yaw):
        self.master.mav.set_attitude_target_send(
            # int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            # -> attitude quaternion (w, x, y, z | zero-rotation is 1, 0, 0, 0)
            QuaternionBase([np.radians(angle) for angle in (roll, pitch, yaw)]),
            0,
            0,
            0,
            0,  # roll rate, pitch rate, yaw rate, thrust
        )

    def _set_att_rate_target(self, roll_rate, pitch_rate, yaw_rate):
        self.master.mav.set_attitude_target_send(
            # int(1e3 * (time.time() - self.boot_time)),  # ms since boot
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
            and mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE,
            [0, 0, 0, 0],
            np.radians(roll_rate),
            np.radians(pitch_rate),
            np.radians(yaw_rate),
            0,  # roll rate, pitch rate, yaw rate, thrust
        )

    def _goto_attitude(self, roll, pitch, yaw, duration):
        duration = duration / self.speedup  # real time to speedup time
        print(f"going to attitude {roll=}, {pitch=}, {yaw=}...")
        self._set_attitude_target(roll, pitch, yaw)
        time.sleep(duration)
        print(f"attitude {roll=}, {pitch=}, {yaw=} reached")

    def _goto_attitude_using_rate(self, roll_rate, pitch_rate, yaw_rate, duration):
        duration = duration / self.speedup  # real time to speedup time
        self._set_att_rate_target(roll_rate, pitch_rate, yaw_rate)
        time.sleep(duration / self.speedup)

    def _do_fom_edge_att_rate(self):
        np.random.seed(int(time.time()))
        att_rate_target = {
            "roll_r": np.random.uniform(6, 8),
            "pitch_r": np.random.uniform(6, 8),
            "yaw_r": np.random.uniform(6, 8),
        }

        orders = ["roll", "pitch", "yaw"]
        np.random.seed(int(time.time()))
        np.random.shuffle(orders)
        for order in orders:
            for sub_seq in range(3):
                first = (-1) ** sub_seq
                next = -1 * first
                self._goto_attitude_using_rate(
                    first * att_rate_target["roll_r"] if order == "roll" else 0.0,
                    first * att_rate_target["pitch_r"] if order == "pitch" else 0.0,
                    first * att_rate_target["yaw_r"] if order == "yaw" else 0.0,
                    1,
                )
                self._goto_attitude_using_rate(
                    next * att_rate_target["roll_r"] if order == "roll" else 0.0,
                    next * att_rate_target["pitch_r"] if order == "pitch" else 0.0,
                    next * att_rate_target["yaw_r"] if order == "yaw" else 0.0,
                    1,
                )

    def _do_fom_edge(self):
        np.random.seed(int(time.time()))
        att_target = {
            "roll": np.random.uniform(15, 20),
            "pitch": np.random.uniform(15, 20),
            "yaw": np.random.uniform((15 + 360) % 360, (20 + 360) / 360),
        }

        orders = ["roll", "pitch", "yaw"]
        np.random.seed(int(time.time()))
        np.random.shuffle(orders)
        self._goto_attitude(0.0, 0.0, 0.0, 1)
        for order in orders:
            for sub_seq in range(3):
                first = (-1) ** sub_seq
                next = -1 * first
                self._goto_attitude(
                    first * att_target["roll"] if order == "roll" else 0.0,
                    first * att_target["pitch"] if order == "pitch" else 0.0,
                    first * att_target["yaw"] if order == "yaw" else 0.0,
                    1,
                )
                self._goto_attitude(
                    next * att_target["roll"] if order == "roll" else 0.0,
                    next * att_target["pitch"] if order == "pitch" else 0.0,
                    next * att_target["yaw"] if order == "yaw" else 0.0,
                    1,
                )
            self._goto_attitude(0.0, 0.0, 0.0, 1)

            time.sleep(3 / self.speedup)

    def _do_fom(self, init_yaw):
        pass

    def do_foms(self):
        pass
