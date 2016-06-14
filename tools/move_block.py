#!/usr/bin/python
import roslib; roslib.load_manifest('fast_simulator')
import rospy

from fast_simulator import client
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Block(object):
    def __init__(self, id, W, x=0.0, y=0.0, z=0.0, th=0.0):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self._delta = 0.025
        self._turn_delta = 3.14159265/2.0
        self._block = W.get_object(self.id)

    def updatePosition(self):
        self._block.set_position(self.x, self.y, self.z, 0.0, 0.0, self.th )

    def move(self, key):
        dx = 0
        dy = 0
        dz = 0
        dth = 0

        if key == '8':
            dx = 0
            dy = 1
        elif key == '2':
            dx = 0
            dy = -1
        elif key == '4':
            dx = -1
            dy = 0
        elif key == '6':
            dx = 1
            dy = 0
        elif key == '9':
            dx = 1
            dy = 1
        elif key == '3':
            dx = 1
            dy = -1
        elif key == '1':
            dx = -1
            dy = -1
        elif key == '7':
            dx = -1
            dy = 1
        elif key == '+':
            dz = 1
        elif key == '-':
            dz = -1
        elif key == '/':
            dth = 1
        elif key == '*':
            dth = -1
        else:
            print "Don't do that"

        self.x += dx * self._delta
        self.y += dy * self._delta
        self.z += dz * self._delta
        self.th += dth * self._turn_delta
        self.updatePosition()


if __name__ == "__main__":
    rospy.init_node('move_block')

    settings = termios.tcgetattr(sys.stdin)

    W = client.SimWorld()

    block = Block(id="block-1", W=W)

    print "Move the simulated block"
    print "Usage: press 5 to print the current coordinates"
    print "       use the numpad keys to move the thing around (+/- : up/down)"

    while not rospy.is_shutdown():

        key = getKey()
        if key == '5':
            print "Current position: [%f, %f, %f, %f]" % (block.x, block.y, block.z, block.th)
        elif key in ['1','2','3','4','6','7','8','9','+','-','/','*']:
            block.move(key)
        else:
            break
