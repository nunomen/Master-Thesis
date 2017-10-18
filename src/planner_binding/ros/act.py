#!/usr/bin/env python
import rospy
import time
import subprocess
import actionlib
from mbot_action_msgs.msg import MoveBaseSafeAction, MoveBaseSafeGoal


class RobotAction(object):

    def __init__(self, action):
        self._action = action
        self.voice_message = ''

    @property
    def voice_message(self):
        return self.__voice_message

    @voice_message.setter
    def voice_message(self, voice_message):
        self.__voice_message = voice_message

    def start(self):
        rospy.loginfo('Starting Execution.')
        self.speak()

    def finish(self):
        rospy.loginfo('Finish Execution.')

    def speak(self):
        subprocess.call(['espeak', '-s 140 -v en-us ', self.voice_message])

    def run(self):
        rospy.loginfo('[CURRENT ACTION]: {}'.format(self._action))


class WaitAction(RobotAction):

    def __init__(self):
        super(WaitAction, self).__init__('Wait')
        self.voice_message = 'I am waiting'

    def start(self):
        super(WaitAction, self).start()

        start_time = time.time()

        while time.time() - start_time < 10:
            continue

        super(WaitAction, self).finish()

    def run(self):
        super(WaitAction, self).run()
        self.start()


class NavigateAction(RobotAction):

    def __init__(self):
        super(NavigateAction, self).__init__('Navigate')

    @property
    def region(self):
        return self.__region

    @region.setter
    def region(self, region):
        self.__region = region

    def start(self):
        super(NavigateAction, self).start()

        # Declare the actionlib client coupled with the destination server and respective action message
        client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)

        client.wait_for_server()
        rospy.loginfo('Action server is up.')

        # Instantiate action message
        goal = MoveBaseSafeGoal()
        goal.arm_safe_position = 'folded'
        goal.source_location = 'start'
        goal.destination_location = self.region
        goal.destination_orientation = ''
        goal.use_destination_pose = False
        timeout = 60.0

        # Procedure that sends a goal to the action server
        rospy.loginfo('Sending action lib goal to move_base_safe_server, destination : ' + goal.destination_location)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        if client.get_result():
            rospy.loginfo("Server responded with success")
        else:
            rospy.loginfo("Server responded with error")

        super(NavigateAction, self).finish()

    def run(self, region):
        super(NavigateAction, self).run()
        self.region = region
        self.voice_message = 'I am moving to the {}'.format(self.region)
        rospy.loginfo('--- Destination: {}\n'.format(self.region))
        self.start()


class RespondAction(RobotAction):

    def __init__(self, intention):
        super(RespondAction, self).__init__('Respond')
        self._intention = intention

    @property
    def whom(self):
        return self.__whom

    @whom.setter
    def whom(self, whom):
        self.__whom = whom

    def run(self, whom):
        super(RespondAction, self).run()
        self.whom = whom
        rospy.loginfo('--- To: {}'.format(self.whom))
        rospy.loginfo('--- Intention: {}'.format(self._intention))


class InitiateConversationAction(RespondAction):

    def __init__(self):
        super(InitiateConversationAction, self).__init__('Initiate Conversation')

    def start(self):
        super(InitiateConversationAction, self).start()
        super(InitiateConversationAction, self).finish()

    def run(self, whom):
        super(InitiateConversationAction, self).run(whom)
        self.voice_message = 'Yes {}, did you call me?'.format(self.whom)
        self.start()


class ConfirmMissionAction(RespondAction):

    def __init__(self):
        super(ConfirmMissionAction, self).__init__('Confirm Mission')

    def start(self):
        super(ConfirmMissionAction, self).start()
        super(ConfirmMissionAction, self).finish()

    def run(self, whom):
        super(ConfirmMissionAction, self).run(whom)
        self.voice_message = 'Do you really want the coke, {}?'.format(whom)
        self.start()


class HelpRequestAction(RespondAction):

    def __init__(self):
        super(HelpRequestAction, self).__init__('Request Help')

    def start(self):
        super(HelpRequestAction, self).start()
        super(HelpRequestAction, self).finish()

    def run(self, whom):
        super(HelpRequestAction, self).run(whom)
        self.voice_message = '{}, I need your help to get the coke.'.format(whom)
        self.start()


class ReceiveObjectAction(RobotAction):

    def __init__(self):
        super(ReceiveObjectAction, self).__init__('Receive Object')

    @property
    def whom(self):
        return self.__whom

    @whom.setter
    def whom(self, whom):
        self.__whom = whom

    def start(self):
        super(ReceiveObjectAction, self).start()
        super(ReceiveObjectAction, self).finish()

    def run(self, whom):
        super(ReceiveObjectAction, self).run()
        self.whom = whom
        self.voice_message = '{}, give me back the coke, please.'.format(self.whom)
        self.start()


class GrabAction(RobotAction):

    def __init__(self):
        super(GrabAction, self).__init__('Grasping Object')

    def start(self):
        super(GrabAction, self).start()
        super(GrabAction, self).finish()

    def run(self):
        super(GrabAction, self).run()
        self.voice_message = 'I am picking up the coke.'
        self.start()


class DeliverAction(RobotAction):

    def __init__(self):
        super(DeliverAction, self).__init__('Delivering Object')

    @property
    def whom(self):
        return self.__whom

    @whom.setter
    def whom(self, whom):
        self.__whom = whom

    def start(self):
        super(DeliverAction, self).start()
        super(DeliverAction, self).finish()

    def run(self, whom):
        super(DeliverAction, self).run()
        self.whom = whom
        self.voice_message = 'I am delivering the coke to {}.'.format(self.whom)
        self.start()


if __name__ == '__main__':
    try:
        rospy.init_node('talker', anonymous=True)
        w = GrabAction()
        w.run()
    except rospy.ROSInterruptException:
        pass
