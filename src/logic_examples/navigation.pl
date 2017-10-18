%%% -*- Mode: Prolog; -*-

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

battery(electronics, 12.1).

battery(motors, 12.3).

motors(on).

location(mbot, living_room).

location(lynda, living_room).

mood(lynda, happy).

failed(navigate(outside)).

mission(location(mbot, patio)).

% Action rules:

navigate(X) :- mission(location(mbot, X)), battery(electronics, Y), Y > 12, battery(motors, Z), Z > 12, \+failed(navigate(X)).

charge :- battery(electronics, X), X < 12; battery(motors, Y), Y < 12.

navigate(charging_station) :- charge.

request_help(X, Y) :- available(X), failed(Y).

% Logical rules of the domain:

actor(X) :- person(X); robot(X).

location(X, Y) :- actor(X), region(Y).

location(X, Y) :- location(X, Z), navigate(Y).

available(X) :- person(X), mood(X, happy).

% Goal of the domain:

% mission(location(mbot, outside)).
