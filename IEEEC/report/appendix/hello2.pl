%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%                Home Environment Domain with possible           %%
%%                   Symbiotic Relationship example               %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% By using the same rules defined in the previous example in ProLog,
% we have that in ProbLog, facts can now have probabilities
% associated with them. The grounded expressions will be:

robot_at_loc(kitchen).

0.6 :: working(robert).
0.2 :: located(robert, kitchen).

0.3 :: working(melanie).
0.9 :: located(melanie,kitchen).

0.1 :: visible(car_keys).
want_grasp(car_keys).

grasp_simulation(cup).
0.9 :: visible(cup).
want_grasp(cup).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Probabilistic inference can now be made to knowledge base.
% To illustrate this example, it will be queried the probability that
% the action ask_help will be successful for the objects 'Car_keys' and 'Cup'.
% It is important to note that the second argument predicate ask_help
% can unify with two types of needs: to_find and to_grasp.
% There are also two people that could help the mobile robot agent, as in the
% previous example.

% Example 1
query(ask_help(H,T,car_keys)).
---- ask_help(melanie,to_find,car_keys) || Prob = 0.567
---- ask_help(melanie,to_grasp,car_keys) || Prob = 0.063
---- ask_help(robert,to_find,car_keys) || Prob =  0.072
---- ask_help(robert,to_grasp,car_keys) || Prob = 0.008

% Example 2
query(ask_help(H,T,cup)).
---- ask_help(melanie,to_find,cup) || Prob = 0.063
---- ask_help(robert,to_find,cup) || Prob =  0.008

% For each example the action taken by the mobile robot would
% be the one with the highest probability of success. Being
% this the case, on the first example, it would ask Melanie
% to find the Car_keys for him. In the second example,
% the robot would ask Melanie to find the cup for him.
