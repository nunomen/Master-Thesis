%%% -*- Mode: Prolog; -*-

% Use ':=' when talking about distributional clauses and
% ':-' when using normal prolog.

% use_module: All exported predicates from the loaded files
% are imported into the module from which this predicate is
% called. The imported predicates act as weak symbols in
% the module into which they are imported. This implies that
% a local definition of a predicate overrides (clobbers)
% the imported definition.

% Import Nitti's planner
:- use_module('../planner/planning').

% Import lists. This library provides commonly accepted
% basic predicates for list manipulation in the Prolog community.
% This includes predicates such as 'member', 'append', 'prefix',
% 'select', 'delete', ...
:- use_module(library(lists)).

% Import system. This package contains utilities for invoking
% services from the operating system that does not fit elsewhere.
% Predicates such as 'now', 'datime', ...
:- use_module(library(system)).

% Set
:- set_options(default).
:- set_current2nextcopy(false).

% builtin predicate to integrate normal prolog predicates into DC.
builtin(prob(P)).
builtin(dim(_,_)).
builtin(color(_,_)).
builtin(type(_,_)).
builtin(deltaT(_)).
builtin(varQ(_)).
builtin(cov(_,_,_)).

% This is the timestep.
deltaT(1).

% Variance of the Q-function.
varQ(0.0004).

% Max of value function.
maxV(100).
maxV(D,V) :- V is 100-1.
maxV(D,100):t := true.

% Cov is equal to VarQ * 1^2 = VarQ
cov(1,[Cov],VarQ) :-
	deltaT(DeltaT),
	Cov is VarQ*DeltaT^2.

% Predicate that returns True with probability P
% Binds a new random float in the open interval (0.0,1.0).
prob(P) :- X is random, X<P.

% Dimensions of the objects in the world.
dim(table,(11,11,2)).
dim(1,(0.02,0.02,0.02)).
dim(2,(0.20,0.255,0.155)).
dim(3,(0.05,0.09,0.05)).
dim(4,(0.055,0.19,0.145)).
dim(5,(0.08,0.08,0.05)).
dim(6,(0.03,0.03,0.03)).
dim(7,(1.1,0.55,0.4)).

% Cube identified as 1 which is green.
type(1,cube).
color(1,green).
rgbcolor(1,(0.0,1.0,0.0)).

% Box identified as 2 which is brown.
color(2,brown).
rgbcolor(2,(0.8,0.5,0.1)).
type(2,box).

% Cube identified as 3 which is blue.
color(3,blue).
rgbcolor(3,(0.0,0.0,1.0)).
type(3,cube).

% Box identified as 4 which is fuschia
color(4,fucsia).
rgbcolor(4,(1.0,0.0,1.0)).
type(4,box).

% Cylinder identified as 5 which is grey
color(5,grey).
rgbcolor(5,(0.3,0.3,0.3)).
type(5,cylinder).

% Cube identified as 6 which is white
color(6,white).
rgbcolor(6,(1.0,1.0,1.0)).
type(6,cube).

% Table identified as 7 which is white
color(7,white).
rgbcolor(7,(1.0,1.0,1.0)).
type(7,table).

% If goal reached, get 100 as reward on timestep t
reward:t ~ val(100) :=
	stop:t.

% If goal state not reached and we have an object in a cilinder region around (0.5,0.8), get -10 as reward on timestep t
reward:t ~ val(-10) :=
    \+stop:t,
	object(ID):t ~= (X,Y,Z),
	sqrt((X-0.5)^2+(Y-0.8)^2)<0.2.

% If goal state not reached and we have an object outside the region defined before, get -1 as reward on timestep t
reward:t ~ val(-1) :=
    \+stop:t,
	object(ID):t ~= (X,Y,Z),
	sqrt((X-0.5)^2+(Y-0.8)^2)>=0.2.

% If the object is within the cylinder around (0.6,1) the goal has been reached.
stop:t :=
	object(ID):t ~= (X,Y,Z),
	sqrt((X-0.6)^2+(Y-1)^2)<0.1.

% If an object is observed initially at 0, set its position to that observation.
object(ID):0 ~  val(P) :=
	observation(object(ID)) ~= P.

object(ID):t+1 ~  val(P) :=
	observation(object(ID)) ~= P.

% If the object was at a specified position, and some action move with
% some displacement (DX,DY) is executed, its new position will be X+DX and Y+DY.
% What does the \+observation mean?
object(ID):t+1 ~ indepGaussians([ ([NX],Cov), ([NY],Cov), ([0],Cov) ]) :=
	object(ID):t ~= (X,Y,Z),
	\+observation(object(ID)) ~=_,
	action(move(DX,DY)),
	NX is X+DX,
	NY is Y+DY,
	varQ(VarQ),
	cov(1,Cov,VarQ),
	deltaT(DeltaT).

% If no action is performed, the object should around the same place.
object(ID):t+1 ~ indepGaussians([ ([X],Cov), ([Y],Cov), ([Z],Cov) ]) :=
	object(ID):t ~= (X,Y,Z),
	\+observation(object(ID)) ~=_,
	\+action(move(ID,_,_)),
	varQ(VarQ),
	cov(1,Cov,VarQ),
	deltaT(DeltaT).

% If there is not an observation at t, generate an observation?
observation(object(ID)):t+1 ~ finite([1:_]) :=
	true.

% admissible actions
% This seems very limited, can only move objects 0.1?
% True if Elem is a member of List.
adm(action(move(A,B))):t :=
	member((X,Y),[(-1,0),(1,0),(0,-1),(0,1)]),A is X/10,B is Y/10.

% between(L,H,val): when val is bounded between L and H, the predicate is True.
m(X,Y):t ~ contUniform(A,B,C,D) :=
	between(1,3,X),
	between(1,3,Y),
	\+ (X=2,Y=2),
	A is (-1+X*2/3-2/3)/10,
	B is (-1+X*2/3)/10,
	C is (-1+Y*2/3-2/3)/10,
	D is (-1+Y*2/3)/10.

% The signature of this predicate is fullplan(File,Init,AVG,Instance,D,Times1,Notes)
% This is very important, must know what the planner does.

testp :-
    executedplan_start,
    executedplan_step(BA,false,[observation(object(1)) ~= (0.3,0.4,0)],100,20,TotalR,T,20,STOP),
    % you should execute the best action BA and get the new observations to pass to the new step
    executedplan_step(BA2,false,[observation(object(1)) ~= (0.1,0.4,0)],100,20,TotalR2,T2,20,STOP2),
    writeln('SOLUTION: '),writeln((BA,T,TotalR)),writeln((BA2,T2,TotalR2)).

% Parameters of the planner
% What is the End constant.
% N is the max number of samples of the solver.
% UsedD is the horizon of the solver
par(test1,N,UsedD,End) :-
	End=1,
	N=15,
	UsedD=10,
	getparam(test1).

abolish_all_tables :-!.

/**
* setparam has the following arguments:
*
* Abstract: *
*   - false; true
* Ratio: *
*   - 0-100
* Useproposal: *
*   - true;false.
* Storing: *
*   - max
* ExecAction: best action should be used
*   - best
* Domain: the name of this domain instance.
*   - predicate describing the name of the domain.
    * Discount: the gamma discount of the bell equation.
*   - 0-1
* [Epsilon_min, Epsilon_max]:
*   - current: 0, 0.4
* Limit_previous_episodes:
*   - current: 100
* Max_horizon_span:
*   - current: 0
* [Lambda_Init, Lambda_Final]:
*   - current: 1,1
* Ucbv:
*   - current: false
* Decay:
*   - current: 0.015
* Strategy:
*   - current: egreedy
* Prunning:
*   - current: 110
* [WHeuInit, WHeuFinal]:
*   - current: -0.1,-0.1
**/

getparam(test1) :-
	bb_put(user:spant,0),
	setparam(false,50,true,max,best,propfalse,1,0,0.4,100,0,1,1,false,0.015,egreedy,110,-0.1,-0.1),!.

% Same thing...
getparam2(test1,N) :-
	par(test1,N,_,_).
score(_,Avg,Avg).

plotepisode(E,N) :-!.
% Exclamation mark is a cut.
plotV(MinE,E,DD) :-!.
