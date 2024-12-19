import multicopter_model.constants as cs
from multicopter_model.controller import QuadCopterController
from multicopter_model.model import QuadCopterModel
from multicopter_model.simulator import Simulator

controller = QuadCopterController()

controller.position_controller_x.set_pid_gains(0.2, 0, 0) #(0.2, 0, 0)
controller.position_controller_x.set_saturation_limit(-3, 3)
controller.position_controller_x.integral_limit = 0.0

controller.position_controller_y.set_pid_gains(0.2, 0, 0) #(0.2, 0, 0)
controller.position_controller_y.set_saturation_limit(-3, 3)
controller.position_controller_y.integral_limit = 0.0

controller.position_controller_z.set_pid_gains(5, 0, 0.1) #(5, 0, 0)
controller.position_controller_z.set_saturation_limit(-3, 3)
controller.position_controller_z.integral_limit = 0.0


controller.velocity_controller_x.set_pid_gains(0.07, 0, 0) #(0.05, 0, 0)
controller.velocity_controller_x.set_saturation_limit(-0.5, 0.5)
controller.velocity_controller_x.integral_limit = 1.0

controller.velocity_controller_y.set_pid_gains(0.07, 0, 0) #(0.05, 0, 0)
controller.velocity_controller_y.set_saturation_limit(-0.5, 0.5)
controller.velocity_controller_y.integral_limit = 1.0

controller.velocity_controller_z.set_pid_gains(1, 0, 0.1) #(1, 0, 0)
controller.velocity_controller_z.set_saturation_limit(-10, 10)
controller.velocity_controller_z.integral_limit = 0.0


controller.pitch_controller.set_pid_gains(1, 0, 0) #(1, 0, 0)
controller.pitch_controller.set_saturation_limit(-5, 5)
controller.pitch_controller.integral_limit = 0.0

controller.roll_controller.set_pid_gains(1, 0, 0) #(1, 0, 0)
controller.roll_controller.set_saturation_limit(-5, 5)
controller.roll_controller.integral_limit = 0.0

controller.yaw_controller.set_pid_gains(0.1, 0, 0.001) #(0.1, 0, 0.01)
controller.yaw_controller.set_saturation_limit(-1, 1)
controller.yaw_controller.integral_limit = 0.0


controller.pitch_rate_controller.set_pid_gains(100, 0, 0) #(100, 0, 0)
controller.pitch_rate_controller.set_saturation_limit(-7, 7)
controller.pitch_rate_controller.integral_limit = 1.0

controller.roll_rate_controller.set_pid_gains(100, 0, 0) #(100, 0, 0)
controller.roll_rate_controller.set_saturation_limit(-7, 7)
controller.roll_rate_controller.integral_limit = 1.0

controller.yaw_rate_controller.set_pid_gains(500, 0.5, 1) #(50, 0, 1.5)
controller.yaw_rate_controller.set_saturation_limit(-0.5, 0.5)
controller.yaw_rate_controller.integral_limit = 0.0


model = QuadCopterModel(cs.quadcopter_inertia, cs.quadcopter_mass, cs.trust_coef, cs.drag_coef, cs.arm_length)

sim = Simulator(controller, model, 0.05)

sim.run()


