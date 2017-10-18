available(H) :- \+ working(H).

reachable(O) :- visible(O), grasp_simulation(O).

near(H) :- robot_at_loc(X), located(H,X).

need_help(to_find,O) :- \+ visible(O).

need_help(to_grasp,O) :- \+ reachable(O), \+ need_help(to_find,O).

ask_help(H,T,O) :- near(H), available(H), need_help(T,O).

robot_at_loc(kitchen).
working(robert).
located(robert,kitchen).
located(melanie,kitchen).
grasp_simulation(cup).
visible(cup).
visible(tv).
