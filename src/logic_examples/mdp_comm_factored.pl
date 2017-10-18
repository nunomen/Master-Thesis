%%% -*- Mode: Prolog; -*-

:- use_module('../planner/planning.pl').

:- use_module(library(lists)).

:- use_module(library(system)).

:- set_options(default).
:- set_current2nextcopy(false).

% General Status Information.

maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.

builtin(person(_)).

% This is the same thing but with factored state representation

person(lynda).
person(robert).
person(melanie).

waiting:0 ~ val(true) :=
    observation(heard) ~= none.

conversation(Name):0 ~ val(true) :=
    observation(heard) ~= Name,
    person(Name).

waiting:t+1 ~ finite([0.2:true, 0.8:false]) :=
    waiting:t ~= true,
    action(navigate(Name)).

conversation(Name):t+1 ~ finite([0.8:true, 0.2:false]) :=
    waiting:t ~= true,
    action(navigate(Name)).

waiting:t+1 ~ val(true) :=
    state:t ~= waiting,
    action(wait).

waiting:t+1 ~ val(true) :=
    conversation(Name):t ~= true,
    action(navigate(Name)).

waiting:t+1 ~ val(0.2:true, 0.8:false) :=
    conversation(Name):t ~= true,
    action(wait).

conversation(Name):t+1 ~ finite([0.8:true, 0.2:false]) :=
    conversation(Name):t ~= true,
    action(wait).

observation(heard):t+1 ~ finite([1:_]) := true.

waiting:t+1 ~  val(true) :=
	observation(heard) ~= none.

conversation(Name):t+1 ~  val(true) :=
	observation(heard) ~= Name,
    person(Name).

% Action Model
adm(action(navigate(Name))):t :=
    person(Name).

adm(action(wait)):t := true.

reward:t ~ val(3) :=
    conversation(Name):t ~= true,
    person(Name).

reward:t ~ val(0) :=
    action(navigate(Name)).

reward:t ~ val(0) :=
    action(wait).

reward:t ~ val(-1) :=
    waiting:t ~= true.

:- initialization(executedplan_start).

test_call :-
    %executedplan_start,
    % executedplan_step(BAction,Abstract,Init,N,MaxD,TotalR,T,MaxDSearch,STOP)
    % BAction=best action,Abstract = use abstraction,Init = list of
    % observations, N = num samples,MaxD =max number of steps,TotalR,T,
    % MaxDSearch=max depth for planning,STOP=if goal is reached
    executedplan_step(BA,false,[observation(heard) ~= none],
    100,10,TotalR,T,10,STOP),
    executedplan_step(BA2,false,[observation(heard) ~= robert],
    100,10,TotalR2,T2,10,STOP2),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR),
    write('Action chosen: '), writeln(BA2),
    write('Timestep: '), writeln(T2),
    write('Reward: '), writeln(TotalR2).

testone :-
    executedplan_start,
    executedplan_step(BA,false,[observation(heard) ~= robert],
    100,10,TotalR,T,10,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

testtwo :-
    executedplan_start,
    executedplan_step(BA,false,[observation(heard) ~= none],
    100,10,TotalR,T,10,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

testthree :-
    executedplan_start,
    executedplan_step(BA,false,[observation(heard) ~= melanie],
    100,10,TotalR,T,10,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

test_step :-
    executedplan_step(BA,false,[observation(heard) ~= robert],
    100,10,TotalR,T,10,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Automatic tests
%:- initialization(time(testp)).

abolish_all_tables :-!.

getparam(com1) :-
	bb_put(user:spant,0),
	setparam(
        % enable abstraction (true/false) you can try this
        false,
        % the ratio of samples reserved for the first action (e.g. 3 means that
        % the first action will get 3 samples more than the remaining actions).
        % the first action should get more samples because it search the entire
        % policy, the remaining steps are just and improving the found policy.
        50,
        % use correct formula for the proposal (leave true)
        true,
        % strategy used to store the V function. Use 'max' that generally works
        % better
        max,
        % ExecAction
        best,
        % Domain
        propfalse,
        % Discount
        0.8,
        % probability to explore in the beginning (first sample)
        0.3,
        % probability to explore in the end (last sample)
        0.4,
        % how many previous samples you want to use to estimate the Q, bigger
        % gives better performance but it is slower, around 100 is generally ok
        100,
        % Max horizon span
        0,
        % Lambda Init
        1,
        % Lambda Final
        1,
        % UCBV
        false,
        % Decay
        0.015,
        % Action selection
        egreedy,
        % higher is better but it is slower
        110,
        % WHeuInit
        -0.1,
        % WHeuFinal
        -0.1)
        ,!.

getparam2(com1,N) :-
	par(com1,N,_,_).
score(_,Avg,Avg).

plotepisode(E,N) :-!.
plotV(MinE,E,DD) :-!.
