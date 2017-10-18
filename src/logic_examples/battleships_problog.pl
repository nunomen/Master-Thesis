bomb(a,0).

% Define every region of the grid:
region(a).
region(b).
region(c).


% This facts should represent the initial beliefs of the POMDP:
% Problem: initial belief probabilities are doing nothing.... I change them and they give the same result.
1/3::state(ship_on_square_A, 0); 1/3::state(ship_on_square_B, 0); 1/3::state(ship_on_square_C, 0); 0::state(no_ship, 0).


% The rules described below represent the transition model of the POMDP:

0.9::hit_target(ship_on_square_A, T); 0.1::failed_target(ship_on_square_A, T) :- state(ship_on_square_A, T), bomb(a, T).

0.5::hit_target(ship_on_square_B, T); 0.5::missed_target(ship_on_square_B, T) :- state(ship_on_square_B, T), bomb(b, T).

0.5::mistargeted(ship_on_square_A, T); 0.5::mistargeted(ship_on_square_C, T) :- missed_target(ship_on_square_B, T).

hit_target(X, T) :- mistargeted(X, T), state(X, T), X \= no_ship.

hit_target(ship_on_square_C, T) :- state(ship_on_square_C, T), bomb(c, T).

state(X, 1) :- failed_target(X, 0).

state(no_ship, 1) :- hit_target(X, 0).

%state(X, T) :- failed_target(X, P), P is T-1.

%state(no_ship, T) :- hit_target(X, P), P is T-1.


% Observation model of the POMDP:

%0.5::destroyed(T); 0.5::survived(T) :- hit_target(X, T).
%0.9::destroyed(T); 0.1::survived(T) :- failed_target(X, T).


% Evidence given to the inference engine:
%evidence(destroyed(0),true).


% Queries made to the interpreter:

% time = 0
%query(state(no_ship,0)).
%query(state(ship_on_square_A,0)).
%query(state(ship_on_square_B,0)).
%query(state(ship_on_square_C,0)).

% time = 1
query(state(no_ship,1)).
query(state(ship_on_square_A,1)).
query(state(ship_on_square_B,1)).
query(state(ship_on_square_C,1)).

% other type
query(hit_target(X,T)).
query(failed_target(X,T)).
