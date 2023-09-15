import argparse
import datetime
####3512
class Params:
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('--host', type=str, default='192.168.4.1')
        self.parser.add_argument('--port', type=int, default=23)
        self.parser.add_argument('--name', type=str, default='Drone1')

        # self.parser.add_argument('--ip', type=str, default='192.168.43.129')
        self.parser.add_argument('--ip', type=str, default='192.168.64.21')
        self.parser.add_argument('--task', type=int, default=3)
        self.parser.add_argument('--camera_port', type=int, default=9060)


        self.parser.add_argument('--zero_pitch_value', type= int, default=1500)
        
        self.parser.add_argument('--zero_roll_value', type= int, default=1500)
        self.parser.add_argument('--zero_throttle_value', type= int, default=1600)
        self.parser.add_argument('--throttleMax', type= int, default=1900)
        self.parser.add_argument('--throttleMin', type= int, default=1300)
        self.parser.add_argument('--yaw_ff', type=int, default=1492)


        self.parser.add_argument('--camera_width', type=int, default=1920)      #1280
        self.parser.add_argument('--camera_height', type=int, default=1080)

        self.parser.add_argument('--camera_fps', type=int, default=120)
        self.parser.add_argument('--arucosize', type=int, default=6)

        self.parser.add_argument('--calib_file', type=str, default='../../CV/calib_data/MultiMatrix.npz')
        self.parser.add_argument('--video_src', type=str, default='/dev/video2')

        self.parser.add_argument('--id_main', type=int, default=17)

        self.parser.add_argument('--logdir', type=str, default='logs/{}/{}/'.format('test', datetime.datetime.now().strftime("  %Y%m%d-  Hr%H   Min%M %S")))
        self.parser.add_argument('--log_interval', type=float, default=0.1)
        self.parser.add_argument('--log', type=bool, default=True)



        self.args = self.parser.parse_args()

args1 = Params().args