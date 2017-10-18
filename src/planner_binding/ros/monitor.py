#!/usr/bin/env python
import ast
import re
import time
import rospy
from std_msgs.msg import String
from planner_binding.msg import PlannerMessage
import copy
import binding


class StateMonitor(object):

    def __init__(self, timeout, goal='none'):
        self._status = ('idle', None)
        self._waiting_for_help = (None, None)
        self._have_goal = goal
        self._conversation_timestamp = None
        self._conversation_timeout = timeout
        self._old_observations = None
        self._sub = rospy.Subscriber("qrcode", String, self.waiting_for_observations)
        self._pub = rospy.Publisher('pipeline_actions', PlannerMessage, queue_size=10)

    def waiting_for_observations(self, external_observations):
        """ Callback function when received an external observation.

        """
        # Convert string of observations to a dictionary.
        external_obs = ast.literal_eval(external_observations.data)

        # Create object for the observation list.
        observation_list = binding.Observations()

        # Add every observation to the observation list.
        for obs in external_obs:
            observation_list.append_observation(obs, external_obs.get(obs))

        if self._old_observations != observation_list:

            self._old_observations = observation_list

            current_obs = copy.deepcopy(observation_list)

            current_obs.append_observation('have_goal', self._have_goal)

            if self._status[0] is 'conversation' and \
                                    time.time() - self._conversation_timestamp > self._conversation_timeout:
                self._status = ('idle', None)

            if self._status[0] is 'idle':
                current_obs.append_observation('status', 'idle')
            else:
                current_obs.append_observation('status', 'conversation({})'.format(self._status[1]))

            if self._waiting_for_help != (None, None) and current_obs.observation('holding(coke)') == 'mbot':
                self._waiting_for_help = (None, None)

            if self._waiting_for_help == (None, None):
                current_obs.append_observation('waiting_for_help', 'none')
            else:
                object = self._waiting_for_help[0]
                who = self._waiting_for_help[1]
                current_obs.append_observation('waiting_for_help', '({}, {})'.format(object, who))

            rospy.loginfo('Got observations: \n {}'.format(current_obs))

            action = self.run_planner(current_obs)

            if action is 'null':
                self._have_goal = 'none'

            elif action is not None:
                execution_message = self.action_caller(action)

                rate = rospy.Rate(1)
                sent = False

                while not rospy.is_shutdown() and sent is False:
                    self._pub.publish(execution_message)
                    sent = True
                    rate.sleep()

        else:
            rospy.loginfo('Got Repeated External observations. Will wait for further observations.')

    def run_planner(self, observations):
        """ Planner pipeline procedures.

        """

        # Clean files from the previous instance.
        binding.Binding.clean_instance()

        # Instantiate a planning problem by creating an object of the following type.
        planning_problem = binding.Binding(obs=observations)

        # Select problem domain from the internal observation of the agent.
        if self._have_goal is 'none':
            planning_problem.set_domain('communication')
            rospy.loginfo('Compiling a wandering domain problem.')
        else:
            planning_problem.set_domain('pickup')
            observations.remove_observation('status')
            observations.remove_observation('have_goal')
            planning_problem.set_observations(observations)
            rospy.loginfo('Compiling a mission domain problem.')

        # Create problem instance file.
        planning_problem.compile_mdp()

        # Solve the problem instance file by running hype prolog solver.
        rospy.loginfo('Solving planning problem.')
        binding.Binding.solve_problem()

        # Get the solution from file.
        action, reward = binding.Binding.action_reward_from_file()

        rospy.loginfo('Hype solver finished execution: action -> {} reward -> {}'.format(action, reward))

        return action

    def action_caller(self, action_result):
        """ Action execution pipeline.

        """
        p = PlannerMessage()

        # Regular expression for action parsing
        match = re.search(r'action[(](\w+)', action_result)
        action = match.group(1)

        if action == 'navigate':
            match = re.search(r'[(]\w+[(](\w+)', action_result)
            where = match.group(1)
            p.action = action
            p.arg = where

        elif action == 'respond':
            match = re.search(r'[(]\w+[(](\w+)[\,\s]+(\w+)', action_result)
            who = match.group(1)
            msg_type = match.group(2)

            if msg_type == 'ready_to_help':
                self._conversation_timestamp = time.time()
                self._status = ('conversation', who)
                p.action = 'ready_to_listen'
                p.arg = who

            else:
                match = re.search(r'[(]\w+[(](\w+)[\,\s]+(\w+)[,\s]+(\w+[\(\s]+[\w\s\,\)]+)[)][)]', action_result)
                who = match.group(1)
                msg_type = match.group(2)
                what = match.group(3)
                p.action = 'confirm'
                p.arg = who
                self._have_goal = what if msg_type == 'confirm_mission' else self._have_goal

        elif action == 'grasp':
            match = re.search(r'[(]\w+[(](\w+)', action_result)
            what = match.group(1)
            p.action = 'grasp'
            p.arg = what

        elif action == 'deliver':
            match = re.search(r'[(]\w+[(](\w+)[\,\s]+(\w+)', action_result)
            object = match.group(1)
            who = match.group(2)
            p.action = 'deliver'
            p.arg = who

        elif action == 'receive':
            match = re.search(r'[(]\w+[(](\w+)[\,\s]+(\w+)', action_result)
            object = match.group(1)
            who = match.group(2)
            p.action = 'receive'
            p.arg = who

        elif action == 'ask_help':
            match = re.search(r'[(]\w+[(](\w+)[\,\s]+(\w+)', action_result)
            object = match.group(1)
            who = match.group(2)
            p.action = 'help_request'
            p.arg = who
            self._waiting_for_help = (object, who)

        elif action == 'wait':
            p.action = action

        else:
            rospy.logerr('No valid action received')

        return p


if __name__ == '__main__':
    try:
        rospy.init_node('sense_and_plan')
        s = StateMonitor(200, goal='want(robert,coke)')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
