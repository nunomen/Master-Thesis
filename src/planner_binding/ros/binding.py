#!/usr/bin/env python
import os
import pyqrcode
import copy


class Observations(object):

    def __init__(self):
        self._observations = {}

    def append_observation(self, observation, value):
        """ Adds a pair observation-value to the observation dictionary.

        """
        self._observations[observation] = value

    def observation(self, obs):
        return self._observations.get(obs)

    def remove_observation(self, type):
        del self._observations[type]

    def generate_qrcode(self, filename):
        """ Creates a QR code in the png format on the tests folder.

        """
        qr_text = str(self._observations)

        qr = pyqrcode.create(qr_text)

        file_path = '../instances/{}.png'.format(filename)

        qr.png(file_path, scale=6)

    def __copy__(self):
        cls = self.__class__
        result = cls.__new__(cls)
        result.__dict__.update(self.__dict__)
        return result

    def __deepcopy__(self, memo):
        cls = self.__class__
        result = cls.__new__(cls)
        memo[id(self)] = result
        for k, v in self.__dict__.items():
            setattr(result, k, copy.deepcopy(v, memo))
        return result

    def __eq__(self, other):
        """Override the default Equals behavior"""
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__
        return NotImplemented

    def __ne__(self, other):
        """Define a non-equality test"""
        if isinstance(other, self.__class__):
            return not self.__eq__(other)
        return NotImplemented

    def __hash__(self):
        """Override the default hash behavior (that returns the id or the object)"""
        return hash(tuple(sorted(self.__dict__.items())))

    def __repr__(self):
        r = '['
        for obs in self._observations:
            r += 'observation({}) ~= {}, '.format(obs, self._observations.get(obs))
        return r[:-2] + ']'


class Binding(object):

    def __init__(self, domain='', obs=None, depth=10, particles=200):
        self._observations = obs
        self._depth = depth
        self._particles = particles
        self._domain = domain

    def set_domain(self, dom):
        self._domain = dom

    def set_observations(self, observations):
        """ Adds the world domain observations to this object

        Example: {near: robert, have(codsworth): coke}
        """
        self._observations = observations

    def set_depth(self, depth):
        """ Modifies the finite horizon of the planner.

        """
        self._depth = depth

    def set_particles(self, number):
        """ Sets the number of episodes that the planner will run.

        """
        self._particles = number

    def compile_mdp(self):
        """ Outputs the planning problem to a file named 'planning.prob'

        """
        r = ':- use_module(\'../domain/{}_mdp.pl\').\n\n'.format(self._domain)
        r += 'run_planner :- '
        r += 'executedplan_start, '
        r += 'executedplan_step(BA, false,'
        r += ' {}, {}, {}, TotalR, T, {}, STOP), '.format(self._observations, self._particles, self._depth, self._depth)
        r += 'open(\'planner_binding/prolog/instance/planning.sol\', \'write\', S), '
        r += 'write(S,\'action \'), write(S, BA), nl(S), write(S,\'reward \'), write(S, TotalR).\n\n'
        r += ':- initialization(run_planner).'

        with open('planner_binding/prolog/instance/problem.pl', 'w') as outfile:
            outfile.write(r)

    @staticmethod
    def clean_instance():
        """ Removes instance problem file and its respective solution from their directories.

        """
        try:
            os.remove('planner_binding/prolog/instance/problem.pl')
            try:
                os.remove('planner_binding/prolog/instance/planning.sol')
            except OSError:
                print('Solution instance file does not exist.')
        except OSError:
            print('Problem instance file does not exist.')

    @staticmethod
    def action_reward_from_file():
        """ Read solution file and return action, reward pair

        """
        action = None
        reward = None

        import pathlib

        problem_file = pathlib.Path('planner_binding/prolog/instance/planning.sol')

        if problem_file.is_file():
            with open('planner_binding/prolog/instance/planning.sol', 'r') as sol_file:
                for line in sol_file:
                    words = line.split()
                    if len(words) == 2 and words[0] == 'action':
                        action = words[1]
                    elif len(words) == 2 and words[0] == 'reward':
                        reward = words[1]

        return action, reward

    @staticmethod
    def solve_problem():
        """ Runs the prolog solver by running the generated MDP.

        """

        FNULL = open(os.devnull, 'w')

        # Try to check if the file exists
        import pathlib

        problem_file = pathlib.Path('planner_binding/prolog/instance/problem.pl')

        if problem_file.is_file():
            import subprocess

            # Run command to solve MDP
            # subprocess.call(['yap', '-L', 'planner_binding/prolog/instance/problem.pl'], stdout=FNULL, stderr=subprocess.STDOUT)
            subprocess.call(['yap', '-L', 'planner_binding/prolog/instance/problem.pl'], stderr=subprocess.STDOUT)

if __name__ == '__main__':
    Binding.solve_problem()
