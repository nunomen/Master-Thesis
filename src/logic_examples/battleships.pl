%%% -*- Mode: Prolog; -*-

:- use_module('../distributionalclause.pl'). % load distributional clauses library
:- use_module('../random/sampling.pl'). % load sampling library
:- use_module(library(lists)). % load list library (prolog)
   
:- set_options(default),set_inference(backward(lw)).
%:- set_debug(true).
 
:- initialization(init). % initialize DC

% Action performed on this timestep.
% Change it to perform another action.
bomb(region(a)).

% Possible regions of the problem.
region(a).
region(b).
region(c).

% Belief distribution for the ship position in each square
state ~ finite([0.333333:ship_on_square_A, 0.333333:ship_on_square_B, 0.333333:ship_on_square_C, 0:no_ship]).

% Transition model of the POMDP
outcome ~ finite([0.9:hit_target(a), 0.1:failed_target(a)]) := state~=ship_on_square_A, bomb(region(a)).
outcome ~ finite([0.5:hit_target(b), 0.5:missed_target(b)]) := state~=ship_on_square_B, bomb(region(b)).
outcome ~ finite([0.5:hit_target(a), 0.5:hit_target(c)]) := missed_target(b).

outcome ~= hit_target(c) :=  state~=ship_on_square_C, bomb(region(c)).

no_ship := hit_target(X).

% Observation model of the POMDP
opponent_said ~ uniform([destroyed, survived]) := hit_target(X).
opponent_said ~ finite([0.9:destroyed, 0.1:survived]) := failed_target(X).

% call for example test_battleships(100). to use 100 samples
test_battleships(N) :-
    init, % initialize DC
    % syntax query: positive evidence,negative evidence, query, number of samples, result (probability of the query)
    query([opponent_said ~= destroyed],[],state ~= ship_on_square_A,N,P), % compute probability color=black, use name_var~=value to compare the value of random variable with a value
    write('probability state = ship_on_square_A: '),writeln(P). % output the result                      
