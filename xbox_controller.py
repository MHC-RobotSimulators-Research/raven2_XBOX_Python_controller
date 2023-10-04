import inputs
from inputs import get_gamepad
from inputs import devices
import math
import threading
import raven_ik as ik
import raven_fk as fk
import ambf_raven_def as ard
import pandas as pd

class xbox_controller:
    
    """
    Gets the state of xbox controller inputs using the inputs library. Modified from
    https://stackoverflow.com/questions/46506850/how-can-i-get-input-from-an-xbox-one-controller-in-python
    """
    MAX_TRIG_VAL = math.pow(2, 8)
    MAX_JOY_VAL = math.pow(2, 15)
    def __init__(self):
        self.gamepad = devices.gamepads[0]
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickY = 0
        self.RightJoystickX = 0
        self.LeftTrigger = 0
        self.RightTrigger = 0
        self.LeftBumper = 0
        self.RightBumper = 0
        self.A = 0
        self.X = 0
        self.Y = 0
        self.B = 0
        self.LeftThumb = 0
        self.RightThumb = 0
        self.Back = 0
        self.Start = 0
        self.LeftDPad = 0
        self.RightDPad = 0
        self.UpDPad = 0
        self.DownDPad = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        # load the csv file for csv_controller mode
        self.df = pd.read_csv('controller_csv/controller.csv')
        # initialize the index to read each row in csv file
        self.id = 0
    def read(self): # return the buttons/triggers that you care about in this methode
        lx = self.LeftJoystickX
        ly = self.LeftJoystickY
        lt = self.LeftTrigger
        lb = self.LeftBumper

        rx = self.RightJoystickX
        ry = self.RightJoystickY
        rt = self.RightTrigger
        rb = self.RightBumper
        return [[lx, ly, lt, lb], [rx, ry, rt, rb],
                [self.A, self.B, self.X, self.Y, self.Back, self.Start]]

    # def rumble(self, left, right, time):
    #     try:
    #         self.gamepad.set_vibration(left, right, time)
    #     except OSError:
    #         print("no space left on device")

    def csv_reader(self):
        self.id += 1
        if self.id <= self.df.shape[0]:
            row_array = self.df.iloc[self.id]
            cmd_row = [[row_array[0], row_array[1], row_array[2], row_array[3]], [row_array[4], row_array[5], row_array[6], row_array[7]],
                [row_array[8], row_array[9], row_array[10], row_array[11], row_array[12], row_array[13]]]
            print(cmd_row)
        else:
            print("Go to the end of the csv file")
            cmd_row = None
        return cmd_row
    
    def _monitor_controller(self):
        while True:
            try:
                events = get_gamepad()
            except inputs.UnknownEventCode:
                print("unknown event")

            for event in events:
                if event.code == 'ABS_Y':
                    self.LeftJoystickY = event.state / xbox_controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_X':
                    self.LeftJoystickX = event.state / xbox_controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RY':
                    self.RightJoystickY = event.state / xbox_controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_RX':
                    self.RightJoystickX = event.state / xbox_controller.MAX_JOY_VAL # normalize between -1 and 1
                elif event.code == 'ABS_Z':
                    self.LeftTrigger = event.state / xbox_controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'ABS_RZ':
                    self.RightTrigger = event.state / xbox_controller.MAX_TRIG_VAL # normalize between 0 and 1
                elif event.code == 'BTN_TL':
                    self.LeftBumper = event.state
                elif event.code == 'BTN_TR':
                    self.RightBumper = event.state
                elif event.code == 'BTN_SOUTH':
                    self.A = event.state
                elif event.code == 'BTN_NORTH':
                    self.Y = event.state #previously switched with X
                elif event.code == 'BTN_WEST':
                    self.X = event.state #previously switched with Y
                elif event.code == 'BTN_EAST':
                    self.B = event.state
                elif event.code == 'BTN_THUMBL':
                    self.LeftThumb = event.state
                elif event.code == 'BTN_THUMBR':
                    self.RightThumb = event.state
                elif event.code == 'BTN_SELECT':
                    self.Back = event.state
                elif event.code == 'BTN_START':
                    self.Start = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY1':
                    self.LeftDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY2':
                    self.RightDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY3':
                    self.UpDPad = event.state
                elif event.code == 'BTN_TRIGGER_HAPPY4':
                    self.DownDPad = event.state

# if __name__ == '__main__':
#     joy = xbox_controller()
#     # print(fk.fwd_kinematics_p5(0, ard.HOME_JOINTS))
#     # joy.rumble(1, 1, 100)

#     while True:
#         # for event in get_gamepad():

#         #     print(event.code)
#         print(joy.read())