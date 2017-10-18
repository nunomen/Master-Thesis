%%% -*- Mode: Prolog; -*-


/*
    Knowledge base of possible agents in the domain.

*/
builtin(person(_)).
builtin(object(_)).
builtin(robot(_)).
builtin(agent(_)).
builtin(movable_agent(_)).
builtin(other_agent(_)).

person(lynda).
person(robert).
person(melanie).

object(coke).

robot(mbot).

agent(Name) :- object(Name); robot(Name); person(Name).

movable_agent(Name) :- person(Name); robot(Name).

other_agent(Name) :- person(Name); object(Name).
