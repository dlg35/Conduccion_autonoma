from __future__ import print_function

import rospy
import smach_ros
from smach import State,StateMachine
from time import sleep

class Uno(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])
    
    def execute(self, userdata):
        print('uno')
        sleep(1)
        return 'success'

class Dos(State):
    def __init__(self):
        State.__init__(self, outcomes=['success','end'])
        self.veces = 0
    
    def execute(self, userdata):
        print('dos')
        sleep(1)
        self.veces+=1
        print("veces estado DOS", self.veces)
        if (self.veces>=3):
            return 'end'
        else:
            return 'success'

if __name__ == '__main__':
    rospy.init_node("test_smach")
    sm = StateMachine(outcomes=['stop'])
    with sm:
        StateMachine.add('UNO', Uno(), transitions={'success':'DOS'})
        StateMachine.add('DOS', Dos(), transitions={'success':'UNO', 'end':'stop'})
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()