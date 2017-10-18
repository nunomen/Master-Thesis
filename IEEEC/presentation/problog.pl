available(Human) :- \+ working(Human).

reachable(Object) :- visible(Object), grasp_simulation(Object).

near(Human) :- robot_at_loc(X), located(Human, X).

need_help(to_find, Object) :- \+ visible(Object).

need_help(to_grasp, Object) :- \+ reachable(Object), \+ need_help(to_find, Object).

ask_help(Human, To, Object) :- near(Human), available(Human), need_help(To, Object).

robot_at_loc(kitchen).

0.6 :: working(robert).
0.2 :: located(robert,kitchen).

0.3 :: working(melanie).
0.9 :: located(melanie,kitchen).

0.1 :: visible(car_keys).

grasp_simulation(cup).
0.9 :: visible(cup).

query(ask_help(Human, To, car_keys)).

query(ask_help(Human, To, cup)).
