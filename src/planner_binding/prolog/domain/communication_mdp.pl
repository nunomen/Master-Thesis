%%% -*- Mode: Prolog; -*-


:- use_module('../planning/planning.pl').
:- use_module('/prolog/domain/regions.pl').
:- use_module('/prolog/domain/agents.pl').
:- use_module(library(lists)).
:- use_module(library(system)).

:- set_options(default).

% What is this?
:- set_current2nextcopy(false).
% Put this on other script.

builtin(want(_, _)).
builtin(goal(_)).

goal(want(Other, Type)) :- person(Other), object(Type).

% -------------- % -------------- % -------------- % --------------

% Setter for the initial state of the robot.
dynamic_state:0 ~ val(idle) :=
	observation(status) ~= idle.

dynamic_state:0 ~ val(conversation(Name)) :=
    observation(status) ~= conversation(Name),
    person(Name).

called:0 ~ val(Someone) :=
    observation(heard) ~= Someone,
    person(Someone).

called:0 ~ val(none) :=
    observation(heard) ~= none.

located(Name):0 ~ val(Place) :=
    observation(camera_tracker(Name)) ~= Place,
    region(Place),
    agent(Name).

near(Name):0 ~ val(Val) :=
    person(Name),
    observation(recognized(Name)) ~= Val.

listened:0 ~ val((Name, Message)) :=
		person(Name), goal(Message),
    observation(recv) ~= (Name, Message).

listened:0 ~ val((none, none)) :=
    observation(recv) ~= (none, none).

mission:0 ~ val(none) :=
    observation(have_goal) ~= none.

mission:0 ~ val(Something) :=
    goal(Something),
    observation(have_goal) ~= Something.

observation(status):t+1 ~ finite([1:_]) := true.

observation(heard):t+1 ~ finite([1:_]) := true.

observation(camera_tracker(Name)):t+1 ~ finite([1:_]) := true.

observation(recognized(Name)):t+1 ~ finite([1:_]) := true.

observation(recv):t+1 ~ finite([1:_]) := true.

observation(have_goal):t+1 ~ finite([1:_]) := true.

dynamic_state:t+1 ~ val(idle) :=
	observation(status) ~= idle.

dynamic_state:t+1 ~ val(conversation(Name)) :=
    observation(status) ~= conversation(Name),
    person(Name).

called:t+1 ~ val(Someone) :=
    observation(heard) ~= Someone,
    person(Someone).

called:t+1 ~ val(none) :=
    observation(heard) ~= none.

located(Name):t+1 ~ val(Place) :=
    observation(camera_tracker(Name)) ~= Place,
    region(Place),
    agent(Name).

near(Name):t+1 ~ val(Val) :=
    person(Name),
    observation(recognized(Name)) ~= Val.

listened:t+1 ~ val((Name, Message)) :=
		person(Name), goal(Message),
    observation(recv) ~= (Name, Message).

listened:t+1 ~ val((none, none)) :=
    observation(recv) ~= (none, none).

mission:t+1 ~ val(none) :=
    observation(have_goal) ~= none.

mission:t+1 ~ val(Something) :=
		goal(Something),
    observation(have_goal) ~= Something.

% -------------- % -------------- % -------------- % --------------

% Transition Model of the World Model

% Navigate predicate
% Robot
effect_navigate(Name):t ~ finite([0.85:NewPlace, 0.15:none]) :=
    robot(Name),
    action(navigate(NewPlace)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name),
    action(wait).

effect_navigate(Name):t ~ val(none) :=
    robot(Name), person(Other),
    action(respond(Other, ready_to_help)).

effect_navigate(Name):t ~ val(none) :=
    robot(Name), person(Other), goal(Message),
    action(respond(Other, confirm_mission, Message)).

% Human
effect_navigate(Other):t ~ val(none) :=
		person(Other).

% Robot and Person Location
located(Name):t+1 ~ val(NewPlace) :=
		movable_agent(Name), region(NewPlace),
    effect_navigate(Name):t ~= NewPlace.

located(Name):t+1 ~ val(OldPlace) :=
		region(OldPlace), movable_agent(Name),
    effect_navigate(Name):t ~= none,
    located(Name):t ~= OldPlace.

% The robot near predicate when he is in the same location of another agent.
near(Other):t ~ finite([0.7:true, 0.3:false]) :=
		person(Other), robot(Name), region(Place),
    located(Name):t ~= Place,
    located(Other):t ~= Place.

% The robot near predicate when he is not in the same location of another agent.
near(Other):t ~ val(false) :=
    person(Other), robot(Name),
    located(Name):t ~= Place1,
    located(Other):t ~= Place2,
    \+(Place1 = Place2).

% While idle
dynamic_state:t+1 ~ val(idle) :=
    dynamic_state:t ~= idle,
    action(wait).

dynamic_state:t+1 ~ val(idle) :=
		region(Place),
    dynamic_state:t ~= idle,
    action(navigate(Place)).

dynamic_state:t+1 ~ finite([0.9:conversation(Other), 0.1:idle]) :=
		person(Other),
    dynamic_state:t ~= idle,
		near(Other):t ~= true,
		called:t ~= Other,
    action(respond(Other, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other),
    dynamic_state:t ~= idle,
		near(Other):t ~= false,
    action(respond(Other, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other),
    dynamic_state:t ~= idle,
		called:t ~= none,
    action(respond(Other, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other1), person(Other2),
    dynamic_state:t ~= idle,
		called:t ~= Other2,
    action(respond(Other1, ready_to_help)),
		\+(Other1 = Other2).

dynamic_state:t+1 ~ val(idle) :=
		person(Other),
    dynamic_state:t ~= idle,
    action(respond(Other, confirm_mission,_)).

% While in conversation
dynamic_state:t+1 ~ finite([0.7:conversation(Other), 0.3:idle]) :=
		person(Other),
    dynamic_state:t ~= conversation(Other),
    action(wait).

dynamic_state:t+1 ~ val(idle) :=
		person(Other), region(Place),
    dynamic_state:t ~= conversation(Other),
    action(navigate(Place)).

dynamic_state:t+1 ~ finite([0.9:conversation(Other1), 0.1:idle]) :=
		person(Other1), person(Other2),
    dynamic_state:t ~= conversation(Other2),
		near(Other1):t ~= true,
		called:t ~= Other1,
    action(respond(Other1, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other1), person(Other2),
    dynamic_state:t ~= conversation(Other2),
		near(Other1):t ~= false,
    action(respond(Other1, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other1), person(Other2),
    dynamic_state:t ~= conversation(Other2),
		called:t ~= none,
    action(respond(Other1, ready_to_help)).

dynamic_state:t+1 ~ val(idle) :=
		person(Other1), person(Other2), person(Other3),
    dynamic_state:t ~= conversation(Other3),
		called:t ~= Other2,
    action(respond(Other1, ready_to_help)),
		\+(Other1 = Other2).

dynamic_state:t+1 ~ val(idle) :=
		person(Other), goal(Purpose),
    dynamic_state:t ~= conversation(Other),
    action(respond(Other, confirm_mission, Purpose)).

dynamic_state:t+1 ~ finite([0.3:conversation(Other1), 0.7:idle]) :=
		person(Other1), person(Other2), goal(Purpose),
    dynamic_state:t ~= conversation(Other1),
    action(respond(Other2, confirm_mission, Purpose)),
		\+(Other1 = Other2).

% This should be made as list. FIX ME
people_draft:t ~ uniform([robert,lynda,melanie]) := true.

% Call behavior
called:t+1 ~ finite([0.9:Other, 0.1:none]) :=
		person(Other),
		called:t ~= Other,
		\+action(respond(Other, ready_to_help)).

called:t+1 ~ val(none) :=
		person(Other),
		called:t ~= Other,
		near(Other):t ~= true,
		action(respond(Other, ready_to_help)).

called:t+1 ~ val(none) :=
		person(Other),
		called:t ~= Other,
		near(Other):t ~= false,
		action(respond(Other, ready_to_help)).

called:t+1 ~ finite([0.1:Other, 0.9:none]) :=
		called:t ~= none,
		person(Other),
    people_draft:t ~= Other.

% Listened Predicate
listened:t ~ val((none,none)) :=
		dynamic_state:t ~= idle.

listened:t ~ finite([0.9:(Other, want(Other, Type)), 0.1:(none,none)]) :=
		person(Other), object(Type),
		dynamic_state:t ~= conversation(Other).

% Mission Predicates
mission:t+1 ~ val(Purpose) :=
		person(Other), goal(Purpose),
		listened:t ~= (Other, Purpose),
    action(respond(Other, confirm_mission, Purpose)).

mission:t+1 ~ val(Purpose3) :=
		person(Other), goal(Purpose1), goal(Purpose2),
		mission:t ~= Purpose3,
		listened:t ~= (Other, Purpose1),
    action(respond(_, confirm_mission, Purpose2)),
		\+(Purpose1 = Purpose2).

mission:t+1 ~ val(Purpose2) :=
		person(Other1), person(Other2), goal(Purpose),
		mission:t ~= Purpose2,
		listened:t ~= (Other1, Purpose),
    action(respond(Other2, confirm_mission, _)),
		\+(Other1 = Other2).

mission:t+1 ~ val(Purpose) :=
		mission:t ~= Purpose,
		action(respond(_, confirm_mission, _)),
		listened:t ~= (none, none).

mission:t+1 ~ val(Purpose) :=
		mission:t ~= Purpose,
    \+action(respond(_, confirm_mission, _)).

% -------------- % -------------- % -------------- % --------------

% Action Model
adm(action(wait)):t := true.

adm(action(navigate(Place))):t := region(Place).

adm(action(respond(Other, ready_to_help))):t :=
		near(Other):t ~= true,
		person(Other).

adm(action(respond(Other, confirm_mission, want(Other,Type)))):t :=
		near(Other):t ~= true,
		person(Other), object(Type).

% -------------- % -------------- % -------------- % --------------

stop:t :=
    mission:t ~= Purpose,
    goal(Purpose).

% Reward Model

reward:t ~ val(100) :=
	stop:t.

reward:t ~ val(-7) :=
    action(respond(Other, ready_to_help)).

reward:t ~ val(-7) :=
    action(respond(Other, confirm_mission, _)).

reward:t ~ val(-8) :=
    action(navigate(Place)).

reward:t ~ val(-5) :=
    action(wait).

reward:t ~ val(-50) := true.


maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.

getparam(wander) :-
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
