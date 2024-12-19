import numpy as np
import multicopter_model.constants as cs
from multicopter_model.constants import FlightMode, States 
from multicopter_model.pid import PID

class QuadCopterController:
    def __init__(self):
        self.target_x = 0
        self.target_y = 0
        self.target_z = 2
        self.target_yaw = 0
        
        self.position_controller_x = PID()
        self.position_controller_y = PID()
        self.position_controller_z = PID()

        self.velocity_controller_x = PID()
        self.velocity_controller_y = PID()
        self.velocity_controller_z = PID()

        self.roll_controller = PID()
        self.pitch_controller = PID()
        self.yaw_controller = PID()

        self.roll_rate_controller = PID()
        self.pitch_rate_controller = PID()
        self.yaw_rate_controller = PID()

        # Пример простого маршрута
        self._mission = [[0, 0, 2, 0], [0, 0, 2, 1], [0, 0, 2, 0], [10, 5, 2, 0], [3, 20, 3, 0], [1, 1, 1, 0], [1, 1, 1, 0]] 
        self._current_mission_index = 0


    def set_target_position(self, x, y, z, yaw):
        self.target_x = x
        self.target_y = y
        self.target_z = z
        self.target_yaw = yaw


    def update(self, state_vector, dt) -> np.ndarray:

        #Обновление целевой точки маршрута

        if ((abs(self._mission[self._current_mission_index][0] - state_vector[States.X])) < 0.2 and
            (abs(self._mission[self._current_mission_index][1] - state_vector[States.Y])) < 0.2 and
            (abs(self._mission[self._current_mission_index][2] - state_vector[States.Z])) < 0.2 and
            (abs(self._mission[self._current_mission_index][3] - state_vector[States.YAW])) < 0.001):
            if (len(self._mission)>self._current_mission_index+1):
                self._current_mission_index += 1
                self.set_target_position(self._mission[self._current_mission_index][0],
                                         self._mission[self._current_mission_index][1],
                                         self._mission[self._current_mission_index][2],
                                         self._mission[self._current_mission_index][3])
                      

        target_vel_x = self.position_controller_x.update(state_vector[States.X], self.target_x, dt)
        target_vel_y = self.position_controller_y.update(state_vector[States.Y], self.target_y, dt)
        target_vel_z = self.position_controller_z.update(state_vector[States.Z], self.target_z, dt)

        target_roll = self.velocity_controller_y.update(state_vector[States.VX], target_vel_x, dt)
        target_pitch = self.velocity_controller_x.update(state_vector[States.VY], target_vel_y, dt)
        cmd_trust = self.velocity_controller_z.update(state_vector[States.VZ], target_vel_z, dt)

        cmd_trust *= cs.trust_scale

        cmd_trust = self.velocity_controller_z.saturation(cmd_trust, cs.min_rotors_rpm, cs.max_rotors_rpm)

        target_pitch_roll = self._rotation2d(state_vector[States.YAW][0]).transpose() @ np.array([[target_pitch][0], [target_roll][0]])
        
        target_roll = target_pitch_roll[0]
        target_pitch = target_pitch_roll[1]

         
        # Пример для контура управления угловым положением и угловой скоростью.
        target_roll_rate = self.roll_controller.update(state_vector[States.ROLL], -target_roll, dt)
        target_pitch_rate = self.pitch_controller.update(state_vector[States.PITCH], target_pitch, dt)
        target_yaw_rate = self.yaw_controller.update(state_vector[States.YAW], self.target_yaw, dt)

        cmd_roll = self.roll_rate_controller.update(state_vector[States.ROLL_RATE], target_roll_rate, dt)
        cmd_pitch = self.pitch_rate_controller.update(state_vector[States.PITCH_RATE], target_pitch_rate, dt)
        cmd_yaw = self.yaw_rate_controller.update(state_vector[States.YAW_RATE], target_yaw_rate, dt)

        u = self._mixer(cmd_trust, cmd_roll, cmd_pitch, cmd_yaw)
        return u

    def _mixer(self, cmd_trust, cmd_roll, cmd_pitch, cmd_yaw) -> np.ndarray:
        u_1 = cmd_trust + cmd_roll - cmd_yaw #Реализуйте алгоритм смешивания команд
        u_2 = cmd_trust - cmd_pitch + cmd_yaw #
        u_3 = cmd_trust - cmd_roll - cmd_yaw #
        u_4 = cmd_trust + cmd_pitch + cmd_yaw #
        return np.array([u_1, u_2, u_3, u_4])

    def _rotation2d(self, theta):
        return np.array([[np.cos(theta), -np.sin(theta)], 
                         [np.sin(theta),  np.cos(theta)]])
    
