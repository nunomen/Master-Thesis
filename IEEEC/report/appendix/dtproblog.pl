%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                Home Environment Domain with possible           %%
%%                   Symbiotic Relationship example               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This is the final example illustrating the workflow of a DT-ProbLog
% program, continuing the example left from ProLog and ProbLog.
% Some rules are added to state the successful execution of an action.
success(action(To), Object) :- ask_help(Human, To, Object), action(To).
success(action(grasp), Object) :- action(grasp), reachable(Object),
                                  want_grasp(Object).

% It is now defined the decision facts that are available to be made.
% In this example the mobile robot agent can only choose one action
% from three: ask to find, ask to grasp or grasp the object himself.
? :: action(grasp); ? :: action(to_find); ? :: action(to_grasp).

% Followed next are the utilities for the defined actions and for the
% sucess state (same reward for the goal state)
utility(action(to_grasp),-2).
utility(action(to_find),-3).
utility(action(grasp),-1).
utility(success(action(to_grasp), car_keys),5).
utility(success(action(to_find),car_keys),5).
utility(success(action(grasp),car_keys),5).

% Here, it is made an inference based on what decision should be made
% to maximize the total expected utility, so the result for running
% this program would be:
---- action(grasp) || d = 0
---- action(to_find) || d = 1
---- action(to_grasp) || d = 0
Score = -0.0318
