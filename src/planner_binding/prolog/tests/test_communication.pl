%%% -*- Mode: Prolog; -*-

:- use_module('../planning/planning.pl').

:- use_module(library(lists)).

:- use_module(library(system)).

:- set_options(default).
:- set_current2nextcopy(false).

:- include('../domain/regions.pl').
:- include('../domain/people.pl').
:- include('../domain/communication_mdp.pl').

% -------------- % -------------- % -------------- % --------------

% Wait
testone(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= none,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= inside_hallway,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= false,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= waiting,
    observation(recv) ~= msg(none, none),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Wait
testtwo(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= none,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= true,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= conversation(robert),
    observation(recv) ~= msg(none, none),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Navigate(inside_hallway)
testthree(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= melanie,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= false,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= waiting,
    observation(recv) ~= msg(none, none),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Wait
testfour(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= melanie,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= true,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= conversation(robert),
    observation(recv) ~= msg(none, none),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Respond(robert, ready_to_help)
testfive(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= robert,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= true,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= waiting,
    observation(recv) ~= msg(none, none),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Respond(robert, confirm_mission_request, want(robert,coke))
testsix(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= none,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= true,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= conversation(robert),
    observation(recv) ~= msg(robert, want(robert, coke)),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% Respond(robert, accept_mission, want(robert,coke))
testseven(BA,TotalR) :-
    executedplan_start,
    executedplan_step(BA,false,
    [observation(heard) ~= none,
    observation(camera_tracker(coke)) ~= kitchen_table,
    observation(camera_tracker(mbot)) ~= bed,
    observation(camera_tracker(robert)) ~= bed,
    observation(camera_tracker(lynda)) ~= sofa,
    observation(camera_tracker(melanie)) ~= inside_hallway,
    observation(recognized(coke)) ~= false,
    observation(recognized(robert)) ~= true,
    observation(recognized(lynda)) ~= false,
    observation(recognized(melanie)) ~= false,
    observation(status) ~= conversation(robert),
    observation(recv) ~= msg(robert, confirm(want(robert, coke))),
    observation(have_goal) ~= none],
    300,7,TotalR,T,7,STOP),
    write('Action chosen: '), writeln(BA),
    write('Timestep: '), writeln(T),
    write('Reward: '), writeln(TotalR).

% -------------- % -------------- % -------------- % --------------


% General Status Information.

maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.


% -------------- % -------------- % -------------- % --------------

testall :-
    testone(X1,Y1), testtwo(X2,Y2), testthree(X3,Y3), testfour(X4,Y4),
    testfive(X5,Y5), testsix(X6,Y6), testseven(X7,Y7),
    writeln(X1), writeln(X2), writeln(X3), writeln(X4), writeln(X5), writeln(X6),
    writeln(X7).


getparam(com1) :-
	bb_put(user:spant,0),
	setparam(
        % enable abstraction (true/false) you can try this
        false,
        % the ratio of samples reserved for the first action (e.g. 3 means that
        % the first action will get 3 samples more than the remaining actions).
        % the first action should get more samples because it search the entire
        % policy, the remaining steps are just and improving the found policy.
        100,
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
        0.6,
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
getparam2(com1,N) :-
	par(com1,N,_,_).
score(_,Avg,Avg).

% take this out
%plotepisode(E,N) :-!.

search_query(I,Q) :-
	distributionalclause:proof_query_backward(I,Q).

plotepisode(E,N) :-
	open('communication_episode.txt','write',S),
	NN is N+1,
	term_to_atom(p(NN,E),I),
	(
    search_query(I,next(near(coke)) ~= A1),
    search_query(I,next(near(melanie)) ~= A2),
    search_query(I,next(near(robert)) ~= A3),
    search_query(I,next(near(lynda)) ~= A4),
    search_query(I,next(located(lynda)) ~= UU1),
    search_query(I,next(located(robert)) ~= UU2),
    search_query(I,next(located(melanie)) ~= UU3),
    search_query(I,next(located(coke)) ~= UU4),
    search_query(I,next(located(mbot)) ~= UU5),
		write(S,near(coke)),write(S,' '),
		write(S,A1),write(S,' '),
    write(S,near(melanie)),write(S,' '),
    write(S,A2),write(S,' '),
    write(S,near(robert)),write(S,' '),
		write(S,A3),write(S,' '),
    write(S,near(lynda)),write(S,' '),
		write(S,A4),write(S,' '),
    write(S,located(lynda)),write(S,' '),
		write(S,UU1),write(S,' '),
    write(S,located(robert)),write(S,' '),
		write(S,UU2),write(S,' '),
    write(S,located(melanie)),write(S,' '),
		write(S,UU3),write(S,' '),
    write(S,located(coke)),write(S,' '),
		write(S,UU4),write(S,' '),
    write(S,located(mbot)),write(S,' '),
		write(S,UU5),write(S,' '),
		nl(S),
		fail;
		true
	),
	forall(	between(0,N,T),
	(
		D is N-T,
		term_to_atom(p(D,E),Key),
    search_query(Key,next(near(coke)) ~= A1),
    search_query(Key,next(near(melanie)) ~= A2),
    search_query(Key,next(near(robert)) ~= A3),
    search_query(Key,next(near(lynda)) ~= A4),
    search_query(Key,next(located(lynda)) ~= UU1),
    search_query(Key,next(located(robert)) ~= UU2),
    search_query(Key,next(located(melanie)) ~= UU3),
    search_query(Key,next(located(coke)) ~= UU4),
    search_query(Key,next(located(mbot)) ~= UU5),
    write(S,near(coke)),write(S,' '),
		write(S,A1),write(S,' '),
    write(S,near(melanie)),write(S,' '),
    write(S,A2),write(S,' '),
    write(S,near(robert)),write(S,' '),
		write(S,A3),write(S,' '),
    write(S,near(lynda)),write(S,' '),
		write(S,A4),write(S,' '),
    write(S,located(lynda)),write(S,' '),
		write(S,UU1),write(S,' '),
    write(S,located(robert)),write(S,' '),
		write(S,UU2),write(S,' '),
    write(S,located(melanie)),write(S,' '),
		write(S,UU3),write(S,' '),
    write(S,located(coke)),write(S,' '),
		write(S,UU4),write(S,' '),
    write(S,located(mbot)),write(S,' '),
		write(S,UU5),write(S,' '),
    nl(S)
		;true
	)
	),
	nl(S),
	close(S),!.

% Exclamation mark is a cut.
plotV(MinE,E,DD) :-!.
