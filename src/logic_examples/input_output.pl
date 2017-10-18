%%% -*- Mode: Prolog; -*-

% type 'go' to start the program.
go :- repeat, % repeat if the following predicates fail.
    write('Guess my number between 1 and 10!'), nl, % write to current output with a newline
    read(X), nl, % read the next prolog term from the current input stream and unify it with X.
    (X==7 -> write('Well done!')), nl. % It represents a 'condition,action,else' control procedure
