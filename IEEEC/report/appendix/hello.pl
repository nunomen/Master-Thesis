%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                Home Environment Domain with possible           %%
%%                   Symbiotic Relationship example               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This simple example describes a situation where exists a
% mobile robot agent that has to complete tasks in a domestic
% environment. If he can not complete a task in this domain
% alone, he must ask for help to a nearby human. It is this last
% detail that will be explored to learn prolog.

% Followed next are the rules of this small example, along with
% descriptions about the reasoning behind them. Most of them
% are self explanatory, as this is an advantage of a
% relational programming language.

% In the first rule, an assumption is made, by saying
% that a human will always help the robot if he is not
% working, which in most of the cases is not true, but
% is a good starting point for an educational example.
available(Human) :- \+ working(Human).

% An object will be reachable to the mobile robot agent if
% he can see it and the simulation of a grasp turned out to
% be successful.
reachable(Object) :- visible(Object), grasp_simulation(Object).

% This is a classical rule, stating that the mobile robot is near
% to someone if he is in the same house division.
near(Human) :- robot_at_loc(X), located(Human, X).

% If the mobile robot agent can not see an object and wants to grasp
% it, he will need help from a human to find the object.
need_help(to_find, Object) :- \+ visible(Object), want_grasp(Object).

% If the mobile robot agent wants to grasp some object
% and he can not perform the action grasp. He will need help to perform
% this action as he would never be able to complete it by himself.
need_help(to_grasp, Object) :- \+ reachable(Object), want_grasp(Object),
                               \+ need_help(to_find, Object).

% The next rule states when the symbiotic relationship will start.
% The ask for help action should be triggered if the robot agent needs help
% to complete a directive and he is close to an available human agent.
ask_help(Human, To, Object) :- near(Human), available(Human),
                               need_help(To, Object).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% In this section, it is disclosed the grounded expressions for a domain
% like the one described above.
% The mobile robot agent is in the kitchen along with two human agents, Robert
% and Melanie. Thus, the first is working and will not be available
% to help the robot. So if an 'ask_help' action is triggered, the human that
% will help the robot is Melanie.
% In this problem the robot wants to grasp three objects: a Cup, a TV and the
% car keys. The robot failed the simulation to grasp the TV, as he can not
% lift an heavy object.
% The car keys are the only type of object that is not visible to the
% robot (typical right?).

robot_at_loc(kitchen).

working(robert).
located(robert,kitchen).

located(melanie,kitchen).

grasp_simulation(cup).
visible(cup).
want_grasp(cup).

visible(tv).
want_grasp(tv).

want_grasp(car_keys).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Finally, here it is shown some useful queries that can be
% made to the knowledge base. While making these, the relevant
% decision that should be made is given by the result of each query.
% By making these queries, ProLog tries to prove its veracity by using
% the unification principle with the information available in the
% knowledge base.

?- ask_help(Human, To, cup).
false

?- ask_help(Human, To, tv).
H = melanie,
T = to_grasp
false

?- ask_help(Human, To, car_keys).
H = melanie,
T = to_find
false
