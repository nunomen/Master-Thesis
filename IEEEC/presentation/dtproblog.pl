? :: action(grasp); ? :: action(to_find); ? :: action(to_grasp).

available(Human) :- \+ working(Human).

near(Human) :- robot_at_loc(X), located(Human, X).

reachable(Object) :- visible(Object), grasp_simulation(Object).

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

success(action(To), Object) :- ask_help(Human, To, Object), action(To).
success(action(grasp), Object) :- action(grasp), reachable(Object).


utility(action(to_grasp),-2).
utility(action(to_find),-3).
utility(action(grasp),-1).
utility(success(action(to_grasp),car_keys),5).
utility(success(action(to_find),car_keys),5).
utility(success(action(grasp),car_keys),5).
