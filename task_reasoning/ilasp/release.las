%problem entities
color(red).
color(green).
color(blue).
color(yellow).
color(white1).
color(white2).
color(white3).
color(white4).
robot(psm1).
robot(psm2).



%0{release(A)}1 :- in_hand(A, ring, _).







#modeha(release(var(robot))).

#modeb(1,in_hand(var(robot),ring,var(color))).
#modeb(1,at(var(robot),ring,var(color))).
#modeb(1,at(var(robot),peg,var(color))).
#modeb(1,reachable(var(robot),ring,var(color))).
#modeb(1,reachable(var(robot),peg,var(color))).
#modeb(1,placed(ring,var(color),peg,var(color))).
#modeb(1,at(var(robot),center)).
#modeb(1,release(var(robot))).
#modeb(1,closed_gripper(var(robot))).

#maxv(2).
#maxhl(1).
#max_penalty(100).




























































































































































