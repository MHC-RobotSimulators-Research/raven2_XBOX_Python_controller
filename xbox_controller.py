import pygame

class xbox_controller:
    """
    Creates a controller object using the pygame joystick module and contains 
    the values of the axes and buttons, developed for use with the Raven II robot

    Author: Natalie 
    """

    def __init__(self):

        # Initialize joystick
        pygame.joystick.init()

        if pygame.joystick.get_count() > 0:
            # Create joystick object
            pygame.joystick.Joystick(0).init()
        else:
            print("No controller found")

        self.joy = pygame.joystick.Joystick(0)
        # specify controller dead zone for the sticks
        self.deadzone = 0.08

    def get_axes(self):
        """
        Method to get all axes values as a list of floats
        Returns:
        A list containing the value of each axes as a float between -1 and 1
        0 = lj_x
        1 = lj_y
        2 = lt
        3 = rj_x
        4 = rj_y
        5 = rt
        """

        axes = []
        for i in range(self.joy.get_numaxes()):
            axes.append(self.joy.get_axis(i))

        return axes

    def get_lj_x(self):
        """
        Method to return the left joystick's x value as a float, -1 to 1.
        """
        if abs(self.joy.get_axis(0)) > self.deadzone:
            return self.joy.get_axis(0)
        else:
            return 0

    def get_lj_y(self):
        """
        Method to return the left joystick's y value as a float, -1 to 1.
        """
        if abs(self.joy.get_axis(1)) > self.deadzone:
            return self.joy.get_axis(1)
        else:
            return 0

    def get_lt(self):
        """
        Method to return the left trigger's value as a float, -1 to 1.
        """
        return self.joy.get_axis(2)

    def get_rj_x(self):
        """
        Method to return the right joystick's x value as a float, -1 to 1.
        """
        if abs(self.joy.get_axis(3)) > self.deadzone:
            return self.joy.get_axis(3)
        else:
            return 0

    def get_rj_y(self):
        """
        Method to return the right joystick's y value as a float, -1 to 1.
        """
        if abs(self.joy.get_axis(4)) > self.deadzone:
            return self.joy.get_axis(4)
        else:
            return 0

    def get_rt(self):
        """
        Method to return the right trigger's value as a float, -1 to 1.
        """
        return self.joy.get_axis(5)

    def get_buttons_bool(self):
        """
        Method to get the state of all the controller buttons

        Returns:
            A list containing the state af all buttons as booleans, True for pressed and false otherwise
            0 = A
            1 = B
            2 = X
            3 = Y
            4 = LB
            5 = RB
            6 = Back
            7 = Start
            8 = ???
            9 = LS
            10 = RS
        """
        buttons = []
        for i in range(self.joy.get_numbuttons()):
            if self.joy.get_button(i) == 1:
                buttons.append(True)
            else:
                buttons.append(False)

        return buttons

    def get_buttons(self):
        """
        Method to get the state of all the controller buttons

        Returns:
            A list containing the state af all buttons as integers, 1 for pressed and 0 otherwise
            0 = A
            1 = B
            2 = X
            3 = Y
            4 = LB
            5 = RB
            6 = Back
            7 = Start
            8 = ???
            9 = LS
            10 = RS
        """
        buttons = []
        for i in range(self.joy.get_numbuttons()):
            buttons.append(self.joy.get_button(i))

        return buttons

    def get_dpad(self):
        """
        Gets the state of the dpad buttons
        Returns:
            A list of ints, when
            [0] = -1 -> left
            [0] = 1 -> right
            [1] = -1 -> down
            [1] = 1 -> up
        """
        return self.joy.get_hat(0)

    def rumble(self):
        pygame.event.get()

        print(self.joy.rumble(0.5, 0.5, 0))
        
# Stuff for testing
def main():

    # Initialize the pygame joystick module followed by the joystick with the id 1
    controller = xbox_controller()
    clock = pygame.time.Clock()


    print("num axes ", pygame.joystick.Joystick(0).get_numaxes())
    print("num hats ", pygame.joystick.Joystick(0).get_numhats())
    print("num buttons ", pygame.joystick.Joystick(0).get_numbuttons())

    running = True

    while running:

        print(controller.get_buttons())
        clock.tick(1)


if __name__ == "__main__":
    main()

