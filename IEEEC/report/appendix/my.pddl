(define (domain DOMAIN_NAME)
    (:requirements [:req_1] ... [:req_N])
    (:types [TYPE_1] ... [TYPE_N])
	(:constants [CTE_1] ... [CTE_N])
	(:predicates (PREDICATE_1 ?A_1 ? ... ?A_N)
					      	...
        	     (PREDICATE_N ?A_1  ... ?A_N))

    (:action ACTION_1
    	[:parameters (?P_1 ... ?P_N)]
    	[:precondition PRECOND_FORMULA]
    	[:effect EFFECT_FORMULA]
    )
		   ...
  	(:action ACTION_N
        [:parameters (?P_1 ... ?P_N)]
    	[:precondition PRECOND_FORMULA]
    	[:effect EFFECT_FORMULA]
    )
)
