#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

class PippinoCommander(Node):
    """
    Create an PippinoCommander class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('pippino_commander')

        nav = BasicNavigator()

        nav.waitUntilNav2Active()

        # Subscribe to battery state topic and check if battery is too low. If too low, go to the dock.

        




    def go_near_dock_and_autodock():
        pass


def main(args=None):
    rclpy.init(args=args)

    pippino_commander = PippinoCommander()



    rclpy.spin(pippino_commander)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
