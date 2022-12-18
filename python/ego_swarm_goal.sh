#!/bin/bash
cnt=10
a=0
while(( $a< cnt))

do
    python ego_swarm_goal.py typhoon_h480 0 -2 -32 4.0&
    python ego_swarm_goal.py typhoon_h480 1 88 20 4.0&
    python ego_swarm_goal.py typhoon_h480 2 17 -17 1.5&
    python ego_swarm_goal.py typhoon_h480 3 15 -4 4.0&
    let "a++"
done



