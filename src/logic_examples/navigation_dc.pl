%%% -*- Mode: Prolog; -*-

:- use_module('~/Git/DC/distributionalclause.pl').
:- use_module('~/Git/DC/random/sampling.pl').

:- set_options(default), set_inference(backward(lw)).


% Regions defined in the program:

region(living_room).

region(outside).

region(kitchen).

region(dining_room).

region(patio).

region(bathroom).

region(outside_hallway).

region(inside_hallway).

region(charging_station).

% Actors defined in the program:

robot(mbot).

person(lynda).

% General status information.

% The voltage of the batteries should be decaying over time.
battery(electronics, 12.1).
battery(motors, 12.3).

motors(on).

location(mbot, living_room).

location(lynda, living_room).

mood(lynda) ~ finite([0.01:angered, 0.01:contempt, 0.01:disgusted, 0.01:fear, 0.93:happy, 0.01:neutral, 0.01:sad, 0.01:surprised]).

navigate(outside) ~ finite([0.8:success, 0.2:failure]).

% There are no goals in POMDP's and MDP's (99 % of the cases).
% Missions should have rewards on a crescent time decaying ratio.
% Should take out mission predicate probably.
mission(location(mbot, patio)).

% Action rules:

navigate(X) :- mission(location(mbot, X)), battery(electronics, Y), Y > 12, battery(motors, Z), Z > 12, \+(navigate(X) ~= failure).

navigate(charging_station) :- charge.

request_help(X, Y) :- available(X), near(X), Y ~= failure.

% Logical rules of the domain:

actor(X) :- person(X); robot(X).

location(X, Y) :- actor(X), region(Y).

location(X, Y) :- location(X, Z), navigate(Y).

available(X) ~ poisson(3) := person(X), mood(X) ~= happy.

near(X) :- location(mbot, Y), location(X, Y).

charge :- battery(electronics, X), X < 12; battery(motors, Y), Y < 12.

% Goal of the domain:

% mission(location(mbot, outside)).

action(N) :-
    init,
    query([], [], mission(location) ~=  , N, P1, _, _),
    write(), writeln(P1).

:- initialization(time(action(1000))).
