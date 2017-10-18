%%% -*- Mode: Prolog; -*-


:- use_module('../planning/planning.pl').
:- use_module('../domain/regions.pl').
:- use_module('../domain/agents.pl').
:- use_module(library(lists)).
:- use_module(library(system)).

:- set_options(default).

% What is this?
:- set_current2nextcopy(false).

% -------------- % -------------- % -------------- % --------------

have(Type):0 ~ val(Name) :=
    observation(holding(Type)) ~= Name,
    object(Type), movable_agent(Name).

have(Type):0 ~ val(none) :=
    observation(holding(Type)) ~= none,
    object(Type).

located(Name):0 ~ val(Place) :=
    observation(camera_tracker(Name)) ~= Place,
    region(Place),
    agent(Name).

near(Name):0 ~ val(Val) :=
    other_agent(Name),
    observation(recognized(Name)) ~= Val.

asked:0 ~ val((Type, Other)) :=
    object(Type), person(Other),
    observation(waiting_for_help) ~= (Type, Other).

asked:0 ~ val((none, none)) :=
    observation(waiting_for_help) ~= none.


observation(camera_tracker(Name)):t+1 ~ finite([1:_]) := true.

observation(holding(Type)):t+1 ~ finite([1:_]) := true.

observation(recognized(Name)):t+1 ~ finite([1:_]) := true.

observation(waiting_for_help):t+1 ~ finite([1:_]) := true.

have(Type):t+1 ~ val(Name) :=
    observation(holding(Type)) ~= Name,
    object(Type), movable_agent(Name).

have(Type):t+1 ~ val(none) :=
    observation(holding(Type)) ~= none,
    object(Type).

located(Name):t+1 ~ val(Place) :=
    observation(camera_tracker(Name)) ~= Place,
    region(Place),
    agent(Name).

near(Name):t+1 ~ val(Val) :=
    other_agent(Name),
    observation(recognized(Name)) ~= Val.

asked:t+1 ~ val((Type, Other)) :=
    object(Type), person(Other),
    observation(waiting_for_help) ~= (Type, Other).

asked:t+1 ~ val((none, none)) :=
    observation(waiting_for_help) ~= none.

% -------------- % -------------- % -------------- % --------------


% Navigate predicate
% Robot
effect_navigate(Name):t ~ finite([0.85:NewPlace, 0.15:none]) :=
    robot(Name),
    action(navigate(NewPlace)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(wait).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(deliver(Type, Other)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(ask_help(Type, Other)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(receive(Type, Other)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(grasp(Type)).

% Person
effect_navigate(Other):t ~ val(Place1) :=
    person(Other), region(Place1), region(Place2), object(Type),
    asked:t ~= (Type, Other),
    have(Type):t ~= none,
    located(Type):t ~= Place1,
    located(Other):t ~= Place2,
    \+(Place1 = Place2).

effect_navigate(Other):t ~ val(none) :=
    person(Other), region(Place), object(Type),
    asked:t ~= (Type, Other),
    have(Type):t ~= none,
    located(Type):t ~= Place,
    located(Other):t ~= Place.

effect_navigate(Other):t ~ val(none) :=
    person(Other), object(Type), robot(Name),
    asked:t ~= (Type, Other),
    have(Type):t ~= Name.

effect_navigate(Other1):t ~ val(Place1) :=
    person(Other1), person(Other2), object(Type), region(Place1), region(Place2),
    asked:t ~= (Type, Other1),
    have(Type):t ~= Other2,
    located(Other1):t ~= Place2,
    located(Other2):t ~= Place1,
    \+(Place1 = Place2),
    \+(Other1 = Other2).

effect_navigate(Other1):t ~ val(none) :=
    person(Other1), person(Other2), object(Type), region(Place),
    asked:t ~= (Type, Other1),
    have(Type):t ~= Other2,
    located(Other1):t ~= Place,
    located(Other2):t ~= Place,
    \+(Other1 = Other2).

effect_navigate(Other):t ~ val(none) :=
    person(Other), region(Place), object(Type), robot(Name),
    located(Name):t ~= Place,
    located(Other):t ~= Place,
    have(Type):t ~= Other,
    asked:t ~= (Type, Other).

effect_navigate(Other):t ~ val(Place1) :=
    person(Other), region(Place1), region(Place2), object(Type), robot(Name),
    located(Name):t ~= Place1,
    located(Other):t ~= Place2,
    have(Type):t ~= Other,
    asked:t ~= (Type, Other),
    \+(Place1 = Place2).

effect_navigate(Other1):t ~ val(none) :=
    person(Other1), person(Other2), object(Type),
    asked:t ~= (Type, Other2),
    \+(Other1 = Other2).

effect_navigate(Other):t ~ val(none) :=
    person(Other), object(Type),
    asked:t ~= (none, none).


% Robot and Person Location
located(Name):t+1 ~ val(NewPlace) :=
    effect_navigate(Name):t ~= NewPlace,
    region(NewPlace),
    movable_agent(Name).

located(Name):t+1 ~ val(OldPlace) :=
    effect_navigate(Name):t ~= none,
    located(Name):t ~= OldPlace,
    region(OldPlace), movable_agent(Name).


% Object location
located(Type):t+1 ~ val(NewPlace) :=
    effect_navigate(Name):t ~= NewPlace,
    have(Type):t ~= Name,
    object(Type), region(NewPlace), movable_agent(Name).

located(Type):t+1 ~ val(OldPlace) :=
    effect_navigate(Name):t ~= none,
    have(Type):t ~= Name,
    located(Type):t ~= OldPlace,
    object(Type), region(OldPlace), movable_agent(Name).

located(Type):t+1 ~ val(OldPlace) :=
    have(Type):t ~= none,
    located(Type):t ~= OldPlace,
    object(Type), region(OldPlace).

% The robot near predicate when he is in the same location of another agent.
near(Other):t ~ finite([0.7:true, 0.3:false]) :=
    located(Name):t ~= Place,
    located(Other):t ~= Place,
    other_agent(Other), robot(Name),
    region(Place).

% The robot near predicate when he is not in the same location of another agent.
near(Other):t ~ val(false) :=
    other_agent(Other), robot(Name),
    located(Name):t ~= Place1,
    located(Other):t ~= Place2,
    \+(Place1 = Place2).


% HAVE OBJECT BEHAVIOR
% When executing this actions, all remains the same.
% The Robot has the object
have(Type):t+1 ~ val(Name) :=
    robot(Name), object(Type),
    have(Type):t ~= Name,
    action(wait).

have(Type):t+1 ~ val(Name) :=
    robot(Name), object(Type), region(SomePlace),
    have(Type):t ~= Name,
    action(navigate(SomePlace)).

have(Type):t+1 ~ val(Name) :=
    robot(Name), object(Type), person(Other),
    have(Type):t ~= Name,
    action(ask_help(_, Other)).

have(Type):t+1 ~ val(none) :=
    robot(Name), object(Type),
    have(Type):t ~= Name,
    action(grasp(_)).

have(Type):t+1 ~ val(none) :=
    robot(Name), object(Type),
    have(Type):t ~= Name,
    action(receive(Type, Other)).

have(Type):t+1 ~ finite([0.9:Other, 0.05:Name, 0.05:none]) :=
    robot(Name), object(Type), person(Other),
    have(Type):t ~= Name,
    near(Other):t ~= true,
    action(deliver(Type, Other)).

have(Type):t+1 ~ val(none) :=
    robot(Name), object(Type), person(Other),
    have(Type):t ~= Name,
    near(Other):t ~= false,
    action(deliver(_, Other)).


% Person has the object
have(Type):t+1 ~ val(Other) :=
    person(Other), object(Type),
    have(Type):t ~= Other,
    action(wait).

have(Type):t+1 ~ val(Other) :=
    person(Other), object(Type), region(SomePlace),
    have(Type):t ~= Other,
    action(navigate(SomePlace)).

have(Type):t+1 ~ val(Other) :=
    person(Other), object(Type),
    have(Type):t ~= Other,
    action(ask_help(_, _)).

have(Type):t+1 ~ val(Other) :=
    person(Other), object(Type),
    have(Type):t ~= Other,
    action(grasp(_)).

have(Type):t+1 ~ finite([0.9:Name, 0.05:Other, 0.05:none]) :=
    robot(Name), object(Type), person(Other),
    have(Type):t ~= Other,
    near(Other):t ~= true,
    action(receive(Type, Other)).

have(Type):t+1 ~ val(Other) :=
    object(Type), person(Other),
    have(Type):t ~= Other,
    near(Other):t ~= false,
    action(receive(Type, Other)).

have(Type):t+1 ~ val(Other) :=
    object(Type), person(Other),
    have(Type):t ~= Other,
    action(deliver(Type, _)).

% Nobody has the object and help requested.
have(Type):t+1 ~ val(Other) :=
    object(Type), person(Other), region(Place),
    have(Type):t ~= none,
    asked:t ~= (Type, Other),
    located(Other):t ~= Place,
    located(Type):t ~= Place.

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other), region(Place1), region(Place2),
    have(Type):t ~= none,
    asked:t ~= (Type, Other),
    located(Other):t ~= Place1,
    located(Type):t ~= Place2,
    \+(Place1 = Place2).

% Nobody has the object and help not requested.
have(Type):t+1 ~ val(none) :=
    object(Type), person(Other),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    action(wait).

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other), region(Place),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    action(navigate(Place)).

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    action(ask_help(_, Other)).

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    action(deliver(_, Other)).

have(Type):t+1 ~ finite([0.7:Name, 0.3:none]) :=
    object(Type), person(Other), robot(Name),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    near(Type):t ~= true,
    action(grasp(Type)).

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other), robot(Name),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    near(Type):t ~= false,
    action(grasp(Type)).

have(Type):t+1 ~ val(none) :=
    object(Type), person(Other), robot(Name),
    have(Type):t ~= none,
    asked:t ~= (none, none),
    action(receive(Type, _)).

% When asking for help
asked:t+1 ~ val((Type, Other)) :=
    object(Type), person(Other),
    near(Other):t ~= true,
    action(ask_help(Type, Other)).

asked:t+1 ~ val((Type2, Other2)) :=
    object(Type), person(Other),
    action(ask_help(Type, Other)),
    near(Other):t ~= false,
    asked:t ~= (Type2, Other2).

asked:t+1 ~ val((none, none)) :=
    object(Type), person(Other), robot(Name),
    asked:t ~= (Type, Other),
    have(Type):t ~= Name.

asked:t+1 ~ val((Type, Other)) :=
    object(Type), person(Other), person(Other2),
    asked:t ~= (Type, Other),
    have(Type):t ~= Other2.

asked:t+1 ~ val((Type, Other)) :=
    object(Type), person(Other),
    asked:t ~= (Type, Other),
    have(Type):t ~= none.

asked:t+1 ~ val((none, none)) :=
    asked:t ~= (none, none),
    \+action(ask_help(Type, Other)).

% Possible actions.
adm(action(navigate(Name))):t := region(Name).

adm(action(wait)):t := true.

adm(action(grasp(Type))):t :=
    object(Type),
    have(Type):t ~= none.

adm(action(deliver(Type, Other))):t :=
    object(Type), person(Other),
    have(Type):t ~= mbot,
    near(Other):t ~= true.

adm(action(receive(Type, Other))):t :=
    object(Type), person(Other),
    have(Type):t ~= Other,
    near(Other):t ~= true.

adm(action(ask_help(Type, Other))):t :=
    object(Type), person(Other),
    have(Type):t ~= none,
    near(Other):t ~= true.


stop:t := asked:t ~= (none, none), have(coke):t ~= robert.

reward:t ~ val(100) := stop:t.

reward:t ~ val(-7) :=
    action(navigate(Somewhere)).

reward:t ~ val(-5) :=
    action(wait).

reward:t ~ val(-25) :=
    action(grasp(Object)).

reward:t ~ val(-6) :=
    action(deliver(Object, Name)).

reward:t ~ val(-6) :=
    action(ask_help(Type, Other)),
    \+(Other = robert).

reward:t ~ val(-20) :=
    action(ask_help(Type, robert)).

reward:t ~ val(-6) :=
    action(receive(Type, Other)).

reward:t ~ val(-50) := true.


maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.

getparam(pick) :-
	bb_put(user:spant,0),
	setparam(
        % enable abstraction (true/false) you can try this
        false,
        % the ratio of samples reserved for the first action (e.g. 3 means that
        % the first action will get 3 samples more than the remaining actions).
        % the first action should get more samples because it search the entire
        % policy, the remaining steps are just and improving the found policy.
        60,
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
        0.9,
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
% Only need this for fullplan
% getparam2(pick1,N) :- par(pick1,N,_,_).
% score(_,Avg,Avg).

plotepisode(E,N) :-!.
% Exclamation mark is a cut.
plotV(MinE,E,DD) :-!.
