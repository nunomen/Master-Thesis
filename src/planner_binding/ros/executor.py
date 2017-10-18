#!/usr/bin/env python
import rospy
from planner_binding.msg import PlannerMessage
import act as actions


class ActionExecutor(object):

    def __init__(self):
        self.subscriber = rospy.Subscriber("pipeline_actions", PlannerMessage, self.run)

    def run(self, request):
        action = request.action
        arg = request.arg

        if action == 'wait':
            a = actions.WaitAction()
            a.run()

        elif action == 'navigate':
            a = actions.NavigateAction()
            a.run(arg.upper())

        elif action == 'ready_to_listen':
            a = actions.InitiateConversationAction()
            a.run(arg)

        elif action == 'confirm':
            a = actions.ConfirmMissionAction()
            a.run(arg)

        elif action == 'help_request':
            a = actions.HelpRequestAction()
            a.run(arg)

        elif action == 'receive':
            a = actions.ReceiveObjectAction()
            a.run(arg)

        elif action == 'grasp':
            a = actions.GrabAction()
            a.run()

        elif action == 'deliver':
            a = actions.DeliverAction()
            a.run(arg)

        else:
            rospy.loginfo('No valid action received.')

if __name__ == "__main__":
    try:
        rospy.init_node('pipeline_action_executor')
        w = ActionExecutor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
