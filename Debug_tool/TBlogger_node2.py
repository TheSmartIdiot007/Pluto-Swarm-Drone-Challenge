from torch.utils.tensorboard import SummaryWriter
from time import sleep
import threading
# loss = 0.5
# with writer.as_default():
#     self.writer.add_scalar('loss', loss, global_step=0)

class TBlogger:
    def __init__(self, controller, args):
        self.controller = controller
        self.args = args
        self.writer = SummaryWriter(log_dir=self.args.logdir)
        self.step = 0
        self.log_thread = threading.Thread(target=self._log)
        self.log_thread.start()
        print('TBLogger init')
        
        


    def _log(self):
        while True:
            if self.controller.log:
                #Controller
                if self.controller.error_ready:
                    # self.writer.add_scalar('Controller/roll_error', self.controller.angle_error[0], global_step=self.step)
                    # self.writer.add_scalar('Controller/pitch_error', self.controller.angle_error[1], global_step=self.step)
                    # self.writer.add_scalar('Controller/yaw_error', self.controller.angle_error[2], global_step=self.step)

                    self.writer.add_scalar('Controller/X_error', self.controller.x_error, global_step=self.step)
                    self.writer.add_scalar('Controller/Y_error', self.controller.y_error, global_step=self.step)
                    self.writer.add_scalar('Controller/Z_error', self.controller.z_error, global_step=self.step)

                    self.writer.add_scalar('Controller/x', self.controller.pos[0], global_step=self.step)
                    self.writer.add_scalar('Controller/y', self.controller.pos[1], global_step=self.step)
                    self.writer.add_scalar('Controller/z', self.controller.pos[2], global_step=self.step)
                    self.writer.add_scalar('Controller/deviation', self.controller.deviation, global_step=self.step)

                    # self.writer.add_scalar('Controller/roll', self.controller.orientation[0], global_step=self.step)
                    # self.writer.add_scalar('Controller/pitch', self.controller.orientation[1], global_step=self.step)
                    # self.writer.add_scalar('Controller/yaw', self.controller.orientation[2], global_step=self.step)

                    # self.writer.add_scalar('Controller/p_wtZ', self.controller.log_data_z[0], global_step=self.step)
                    # self.writer.add_scalar('Controller/i_wtZ', self.controller.log_data_z[1], global_step=self.step)
                    # self.writer.add_scalar('Controller/d_wtZ', self.controller.log_data_z[2], global_step=self.step)

                    # self.writer.add_scalars('Controller/Z/Position', {'Kp':self.controller.position_controller.Kp[2],
                    #                                                   'Kd':self.controller.position_controller.Kd[2],
                    #                                                   'Ki':self.controller.position_controller.Ki[2]}, global_step= self.step)
                    
                    # self.writer.add_scalars('Controller/Z/PID_Output', {'Kp output':self.controller.position_controller.Kp_out[2],
                    #                                                     'Kd output':self.controller.position_controller.Kd_out[2],
                    #                                                     'Ki output':self.controller.position_controller.Ki_out[2]}, global_step= self.step)
                    # self.writer.add_scalar('Controller/position_Z/Kd', self.controller.position_controller.Kd[2], step = self.step)
                    # self.writer.add_scalar('Controller/position_Z/Ki', self.controller.position_controller.Ki[2], step = self.step)

                # CV Data
                self.writer.add_scalar('CV/X', self.controller.vision.X[self.args.id_main][0], global_step=self.step)
                self.writer.add_scalar('CV/Y', self.controller.vision.Y[self.args.id_main][0], global_step=self.step)
                self.writer.add_scalar('CV/Z', self.controller.vision.Z[self.args.id_main][0], global_step=self.step)

                self.writer.add_scalar('CV/Vx', self.controller.vision.X[self.args.id_main][1], global_step=self.step)
                self.writer.add_scalar('CV/Vy', self.controller.vision.Y[self.args.id_main][1], global_step=self.step)
                self.writer.add_scalar('CV/Vz', self.controller.vision.Z[self.args.id_main][1], global_step=self.step)

                # self.writer.add_scalar('CV/yaw', self.controller.vision.yaw[self.args.id_main][0][0], global_step=self.step)
                # self.writer.add_scalar('CV/d_yaw', self.controller.vision.yaw[self.args.id_main][1][0], global_step=self.step)

                # Pluto data
                self.writer.add_scalar('Pluto/roll', self.controller.bridge.pro.roll, global_step=self.step)
                self.writer.add_scalar('Pluto/pitch', self.controller.bridge.pro.pitch, global_step=self.step)
                self.writer.add_scalar('Pluto/yaw', self.controller.bridge.pro.yaw, global_step=self.step)

                # self.writer.add_scalar('Pluto/roll_KF', self.controller.bridge.pro.gyro_KF.x[0][0], global_step=self.step)
                # self.writer.add_scalar('Pluto/pitch_KF', self.controller.bridge.pro.gyro_KF.y[0][0], global_step=self.step)
                # self.writer.add_scalar('Pluto/yaw_KF', self.controller.bridge.pro.gyro_KF.z[0][0], global_step=self.step)

                # self.writer.add_scalar('Pluto/roll_rate', self.controller.bridge.pro.gyro_KF.x[1][0], global_step=self.step)
                # self.writer.add_scalar('Pluto/pitch_rate', self.controller.bridge.pro.gyro_KF.y[1][0], global_step=self.step)
                # self.writer.add_scalar('Pluto/yaw_rate', self.controller.bridge.pro.gyro_KF.z[1][0], global_step=self.step)

                # self.writer.add_scalar('Pluto/height', self.controller.bridge.pro.alt[0], global_step=self.step)

                #RC control data
                self.writer.add_scalar('RC/roll', self.controller.bridge.pro.rc.roll, global_step=self.step)
                self.writer.add_scalar('RC/pitch', self.controller.bridge.pro.rc.pitch, global_step=self.step)
                self.writer.add_scalar('RC/yaw', self.controller.bridge.pro.rc.yaw, global_step=self.step)
                self.writer.add_scalar('RC/throttle', self.controller.bridge.pro.rc.throttle, global_step=self.step)
                # self.writer.add_scalar('Pluto/KF_alt', self.controller.bridge.pro.alt_KF_out[0][0], global_step=self.step)


                self.step += 1
                sleep(self.args.log_interval)
