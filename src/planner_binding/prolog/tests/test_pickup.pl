%%% -*- Mode: Prolog; -*-

:- use_module('../planning/planning.pl').

:- use_module(library(lists)).

:- use_module(library(system)).

:- set_options(default).
:- set_current2nextcopy(false).

:- include('../domain/regions.pl').
:- include('../domain/people.pl').
:- include('../domain/pickup_mdp.pl').

% -------------- % -------------- % -------------- % --------------

% Expected action: navigate(robert)
testone(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= coke,
    observation(holding(robert)) ~= none,
    observation(holding(codsworth)) ~= coke,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).


% Expected action: navigate(coke)
testtwo(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= robert,
    observation(holding(robert)) ~= none,
    observation(holding(codsworth)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Expected action: respond(robert,completed_mission,want(robert,coke))
testthree(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= robert,
    observation(holding(robert)) ~= coke,
    observation(holding(codsworth)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Expected action: deliver(coke,robert)
testfour(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= robert,
    observation(holding(codsworth)) ~= coke,
    observation(holding(robert)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

testfive(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= coke,
    observation(holding(codsworth)) ~= none,
    observation(holding(robert)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= success
    ],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

testsix(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= coke,
    observation(holding(codsworth)) ~= none,
    observation(holding(robert)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure
    ],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

testseven(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(recognized) ~= lynda,
    observation(holding(codsworth)) ~= none,
    observation(holding(robert)) ~= none,
    observation(have_goal) ~= want(robert,coke),
    observation(grasp_planning(coke)) ~= failure
    ],
    400,16,TotalR,T,16,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% -------------- % -------------- % -------------- % --------------

% General Status Information.

maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.


testall :-
    testone(X1,Y1), testtwo(X2,Y2), testthree(X3,Y3), testfour(X4,Y4),
    testfive(X5,Y5),testsix(X6,Y6), testseven(X7,Y7),
    writeln(X1), writeln(X2), writeln(X3), writeln(X4), writeln(X5),
    writeln(X6), writeln(X7).


getparam(pick1) :-
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
        0.9,
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

% Same thing...
getparam2(pick1,N) :-
	par(pick1,N,_,_).
score(_,Avg,Avg).

% take this out
%plotepisode(E,N) :-!.

search_query(I,Q) :-
	distributionalclause:proof_query_backward(I,Q).

plotepisode(E,N) :-
	open('pickup_episode.txt','write',S),
	NN is N+1,
	term_to_atom(p(NN,E),I),
	(
        search_query(I,next(near) ~= A),
        search_query(I,next(have(Name)) ~= Some),
		write(S,near),write(S,' '),
		write(S,A),write(S,' '),
		nl(S),
		fail;
		true
	),
	forall(	between(0,N,T),
	(
		D is N-T,
		term_to_atom(p(D,E),Key),
		search_query(Key,next(near) ~= A),
        search_query(I,next(have(Name)) ~= Some),
		search_query(Key,next(greedy(GR))),
        write(S,near),write(S,' '),
		write(S,A),write(S,' '),
        write(S,GR),nl(S)
		;true
	)
	),
	nl(S),
	close(S),!.

% Exclamation mark is a cut.
plotV(MinE,E,DD) :-!.
