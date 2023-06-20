import xbox_controller
import geometry_msgs.msg

class sr_command(self):
    def __init__(self):
        self.cmd = xbox_controller()
        self.m = geometry_msgs.msg.TransformStamped()
        return None
    
    def update_x(self):
        x = self.cmd.get_lj_x()
        if (x >0 and x<=1):
            m.transform.translation.x = x
        