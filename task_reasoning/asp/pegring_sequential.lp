% #include <incmode>.






#program base.

% variables and constants
color(red;green;blue;yellow;white1;white2;white3;white4).
robot(psm1;psm2).
object(ring;peg).


% data from sensors
#external reachable(Robot, Object, Color) : object(Object), color(Color), robot(Robot).
#external at(Robot, Object, Color) : object(Object), color(Color), robot(Robot).
#external at(Robot, center) : robot(Robot).
#external placed(ring, Color, peg, OtherColor) : color(Color), color(OtherColor).
#external closed_gripper(Robot) : robot(Robot).

% initial conditions
at(Robot, Object, Color, 1) :- at(Robot, Object, Color).
at(Robot, center, 1) :- at(Robot, center).
placed(ring, Color, peg, OtherColor, 1) :- placed(ring, Color, peg, OtherColor).
closed_gripper(Robot, 1) :- closed_gripper(Robot).




% closed_gripper(psm1).
% closed_gripper(psm2).
% reachable(psm1,peg,red).
% reachable(psm2,peg,green).
% reachable(psm1,peg,blue).
% reachable(psm2,peg,yellow).
% reachable(psm2,peg,white1).
% reachable(psm2,peg,white2).
% reachable(psm1,peg,white3).
% reachable(psm1,peg,white4).
% placed(ring,red,peg,white1).
% placed(ring,green,peg,white4).
% reachable(psm2,ring,red).
% reachable(psm1,ring,green).











#program step(t).

% action preconditions
0{move(Robot, ring, Color, t) : reachable(Robot, ring, Color) ; %distance(Robot, ring, Color, X), best(X, t)
grasp(Robot, ring, Color, t) : at(Robot, ring, Color, t); 
release(Robot, t) : closed_gripper(Robot, t) ; 
move(Robot, peg, Color, t) : reachable(Robot, peg, Color);
extract(Robot, ring, Color, t) : in_hand(Robot, ring, Color, t);
move(Robot, center, Color, t) : in_hand(Robot, ring, Color, t)}1. % :- robot(Robot).

% constraints
% at transfer, if both arms are holding the ring, no one can move
:- move(Robot, center, _, t), in_hand(Robot, ring, Color, t), in_hand(OtherRobot, ring, Color, t), Robot != OtherRobot.
:- move(Robot, Object, _, t), in_hand(Robot, ring, Color, t), in_hand(OtherRobot, ring, Color, t), Robot != OtherRobot.
% a ring cannot be released on other place than a peg
:- release(Robot, t-1), in_hand(Robot, ring, Color, t-1), not in_hand(_, ring, Color, t), not placed(ring, Color, peg, _, t).
% cannot move to a ring with closed gripper
:- closed_gripper(Robot, t), move(Robot, ring, _, t).
:- closed_gripper(Robot, t), move(OtherRobot, center, _, t), Robot != OtherRobot.
% need extraction if the grasped ring is placed on a peg
:- move(Robot, center, Color, t), in_hand(Robot, ring, Color, t), placed(ring, Color, peg, _, t).
:- move(Robot, peg, Color, t), in_hand(Robot, ring, Color1, t), placed(ring, Color1, peg, _, t).
% cannot move to an occupied peg (WHICH RULE?)

% effects of actions (state transitions)
closed_gripper(Robot, t) :- grasp(Robot, ring, _, t-1).
closed_gripper(Robot, t) :- closed_gripper(Robot, t-1), not release(Robot, t-1).
in_hand(Robot, ring, Color, t) :- at(Robot, ring, Color, t), closed_gripper(Robot, t).
in_hand(Robot, ring, Color, t) :- grasp(Robot, ring, Color, t-1).
in_hand(Robot, ring, Color, t) :- in_hand(Robot, ring, Color, t-1), not release(Robot, t-1).
placed(ring, Color, peg, OtherColor, t) :- at(Robot, peg, OtherColor, t-1), release(Robot, t-1), in_hand(Robot, ring, Color, t-1). 
placed(ring, Color, peg, OtherColor, t) :- placed(ring, Color, peg, OtherColor, t-1), not extract(_, ring, Color, t-1).
at(Robot, ring, Color, t) :- move(Robot, ring, Color, t-1). 
at(Robot, ring, Color, t) :- move(OtherRobot, center, Color, t-1), Robot!=OtherRobot, robot(Robot), in_hand(OtherRobot, ring, Color, t-1). 
at(Robot, peg, Color, t) :- move(Robot, peg, Color, t-1).
at(Robot, center, t) :- move(Robot, center, Color, t-1).



#program check(t).

% task goal
#external query(t). % for python interface
:- reachable(_,ring,Color), reachable(_,peg,Color), not placed(ring,Color,peg,Color,t), query(t).











% output actions
#show move/4.
#show grasp/4.
#show release/2.
#show extract/4.
