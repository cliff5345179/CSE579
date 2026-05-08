% init objects
node(N,X,Y) :- init(object(node,N), value(at,pair(X,Y))).
highway(N,X,Y) :- init(object(highway,N), value(at,pair(X,Y))).
pickingStation(P,X,Y) :- init(object(pickingStation,P), value(at,pair(X,Y))).
object(robot,R) :- init(object(robot,R), value(at,pair(X,Y))).
object(shelf,S) :- init(object(shelf,S), value(at,pair(X,Y))).
orderTo(O,P) :- init(object(order,O),value(pickingStation,P)).
orderContent(O,I,C) :- init(object(order,O),value(line,pair(I,C))).

%fluent
loc(robot,R,X,Y,0) :- init(object(robot,R), value(at,pair(X,Y))).
loc(shelf,S,X,Y,0) :- init(object(shelf,S), value(at,pair(X,Y))).
productLoc(I,S,C,0) :- init(object(product,I), value(on,pair(S,C))).
orderComplete(O,I,C,0,0) :- init(object(order,O),value(line,pair(I,C))).
on(0,S,0) :- init(object(shelf,S), value(at,pair(X,Y))).

% uniqueness and existence of value constraints.
:- not {loc(robot,R,X,Y,T):node(N,X,Y)}=1, object(robot,R), T=1..m.
:- not {loc(shelf,S,X,Y,T):node(N,X,Y)}=1, object(shelf,S), T=1..m.
:- not {productLoc(I,S,C,T)}=1, productLoc(I,S,C,T), T=1..m.
:- not {orderComplete(O,I,C,U,T)}=1, orderContent(O,I,C), T = 1..m.
:- not {on(_,S,T)}=1,object(shelf,S), T = 1..m.

% commonsense law of inertia
{loc(robot,R,X,Y,T+1)} :- loc(robot,R,X,Y,T), T=0..m-1.
{loc(shelf,S,X,Y,T+1)} :- loc(shelf,S,X,Y,T), T=0..m-1.
{productLoc(I,S,C,T+1)} :- productLoc(I,S,C,T), T=0..m-1.
{orderComplete(O,I,C,U,T+1)} :- orderComplete(O,I,C,U,T), T=0..m-1.
{on(R,S,T+1)} :- on(R,S,T),T=0..m-1.

% constraints
% 1 action at a time
:- 2{occurs(object(robot,R),X ,T)}, object(robot,R), T = 1..m.
% two shelves cannot collide 
:- loc(shelf,S,X,Y,T), loc(shelf,S1,X,Y,T), S!=S1.
% avoid collision between robots
:- loc(robot,R,X,Y,T), loc(robot,R1,X,Y,T), R!=R1.
% force product loc exist
:- not {productLoc(I,S,_,T)}=1, productLoc(I,S,_,T-1), T=1..m.

%move
move(1,0; -1,0; 0,1; 0,-1).

%%%%%%%%%%%%%%%%
%Robot movement
%%%%%%%%%%%%%%%%
% robot movement is exogenous
{occurs(object(robot,R), move(X1,Y1),T)} :- object(robot,R), move(X1,Y1), T=1..m.
% direct effect.
loc(robot,R,X+X1,Y+Y1,T) :- occurs(object(robot,R), move(X1,Y1), T), loc(robot,R,X,Y,T-1), T=1..m.
%bots are not allowed to switch places.
:- loc(robot,R,X1,Y1,T), loc(robot,R1,X2,Y2,T), loc(robot,R,X2,Y2,T+1), loc(robot,R1,X1,Y1,T+1), R!=R1.
%robot cannot step outside of warehouse.
:- not node(_,X,Y), loc(robot,R,X,Y,T), T=1..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot pickup shelf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pickup is exogenous
{pickup(R,S,T): object(shelf,S)}1 :- object(robot,R), T=1..m.
occurs(object(robot,R), pickup,T) :- pickup(R,S,T).
% concept of on
on(R,S,T) :- pickup(R,S,T).
% direct effect
loc(shelf,S,X,Y,T) :- on(R,S,T), loc(robot,R,X,Y,T).
% can only pickup at same location
:- pickup(R,S,T), loc(robot,R,X,Y,T-1), not loc(shelf,S,X,Y,T-1).

% cannot pickup two shelves
:- pickup(R,S,T), on(R,S1,T-1), T = 0..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Robot putdown shelf
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% putdown is exogenous
{occurs(object(robot,R), putdown,T)}1 :- object(robot,R), T=1..m.
%direct effect
on(0,S,T) :- occurs(object(robot,R), putdown,T), on(R,S,T-1), T=1..m.
% cannot do putdown if is not lift.
:- not on(R,_,T-1), occurs(object(robot,R), putdown,T), T=1..m.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%shelf cannot be on highway
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
:- highway(N,X,Y), loc(shelf,S,X,Y,T), loc(robot,R,X,Y,T), not on(R,S,T).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%product and deliver
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% product can't < 0
:- productLoc(P,S,C,T), C < 0, T = 1..m.
% only deliver exact order count
:- orderComplete(O,I,C,U,T), C < U, T = 1..m.

%deliver is exogenous
%{occurs(object(robot,R),deliver(O,I,U),T)}1 :- object(robot,R), on(R,S,T-1), productLoc(I,S,U,T-1), orderComplete(O,I,C,_,T), T=1..m.
{deliver(R,O,I,U,T)}1 :- object(robot,R), on(R,S,T-1), productLoc(I,S,_,T-1), orderComplete(O,I,C,U1,T-1), U=C-U1, T=1..m.
occurs(object(robot,R),deliver(O,I,U),T) :- deliver(R,O,I,U,T).
% picking station
:- deliver(R,O,I,U,T), loc(robot,R,X,Y,T-1), orderTo(O,P), not pickingStation(P,X,Y).
% shelf needs to be on
:- deliver(R,O,_,_,T), not on(R,_,T-1).
% need to carry valid product
:- deliver(R,O,I,_,T), object(robot,R), on(R,S,T-1), not productLoc(I,S,_,T-1).

%direct effect
orderComplete(O,I,C,U+U1,T) :- deliver(R,O,I,U,T), orderComplete(O,I,C,U1,T-1).
productLoc(I,S,C-U,T) :- deliver(R,O,I,U,T), on(R,S,T-1), productLoc(I,S,C,T-1).

%goal
%:- not loc(robot,1,3,3,m).
%:- not on(1,1,3).
%:- not occurs(object(robot,1),pickup,2).
%:- not loc(shelf,6,1,3,m).
%:- not occurs(object(robot,2),deliver(1,3,4),4).
%:- not occurs(object(robot,2),putdown,5).
%:- not orderComplete(1,3,4,4,m).
%:- not orderComplete(1,1,1,1,m).
%:- not orderComplete(2,2,1,1,m).
%:- not orderComplete(3,4,1,1,m).
:- orderComplete(O,I,C,U,m), C!=U.
#minimize{1,T: occurs(object(robot,_),_,T)}.

#show occurs/3.
%#show loc/5.
%#show on/3.
%#show productLoc/4.
%#show orderComplete/5.
%#show deliver/5.