%%% -*- Mode: Prolog; -*-
:- use_module('~/Git/DC/distributionalclause.pl').
:- use_module('~/Git/DC/random/sampling.pl').

:- set_options(default), set_inference(backward(lw)).

% Integrate this predicates into DC.
builtin(object(_)).

sodacan(coke).
cereal_box(cereal).

volume(coke, 6.60).
volume(cereal, 16.60).

mobility(cereal,1).
mobility(coke, 1).

movable_object(X) :- cereal_box(X).
movable_object(X) :- sodacan(X).
object(X) :- movable_object(X).

immovable_object :- table; counter; bench.

occlusion(X, Y) :- volume(X, VX), volume(Y, VY), VX > VY.

is_in(X) ~ uniform([counter, table, bench]) := object(X).

%seen(X, Y, T1, T2) ~ finite([0.7:true,0.3:false]) := is_in(X, Y), mobility(X, M), P is 0.6+0.1*exp(-(T1-T2)/10M).
%seen(X, Y, T1, T2) ~ finite([P:true,R:false]) := \+is_in(X, Y), mobility(X, M), P is 0.3+0.1*exp(-(T1-T2)/10M), R is 1-P.

test_cereal(N) :-
    % Initialize the inference system
    init,
    % How likely is the event in which the cereal is in the table.
    query([],[],is_in(cereal) ~= table, N, P1, _, _),
    write('The cereal is in the table with probability: '),writeln(P1).


test_coke(N) :-
    % Initialize the inference system
    init,
    % How likely is the event in which the coke is in the counter.
    query([],[],is_in(coke) ~= counter, N, P2, _, _),
    write('The coke is in the counter with probability: '),writeln(P2).

% automatically execute the provided query.
:- initialization(time(test_coke(1000))).
