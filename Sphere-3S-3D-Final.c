//Declare any variables shared between functions here
/*
3-D game, Version 2.3
With opponent sphere also operational.
Highest score posted with opponent Item-bot : 46
Highest score posted with opponent Item-Thief : 15
Uses setTargetPosition to move sphere to specified coord.
Changed parameters of PD controller from default for
greater speed, comes at the expense of more fuel consumption

The New Strategy:
1. Drop first SPS at initial position.
2. Go and pick up large item closest to SPHERE
3. With the item, go and drop the 2nd and then the third SPS
4. Go to zone and drop large item there.

version 2.3
1. Sometimes the opponent snuck in and picked up the item we were
   targetting. So collapsed the stems choosing and picking up the
   the item into 1 step. Also, added an additional check to see if
   the opponent has our item.
2. Increased the angle error to 0.10. Decreased the zoneErrorTolerance
   to 0.02, this was because sometimes even after dropping the item in
   the zone, it wasn't accurate enough to accrue points.
3. For the position PD values, as fuel dropped increased D from 3 to 5.
4. In getClosestAvailableItem() function also update array itemDist[],
   which stores the distances from the sphere to all items searched for.
5. Code size is 89% of max allowable.

FIXED ISSUES
============
1. Code size is 111% (not eligible for submisson). Fix by using mathVec operations in
   computeDistance function. Code size with change is now:85%

2. Modify dockItem() call to include item_id, as in docItem(item_id). This should
   fix the issue when 2 items are close to the sphere and we incur docking penalties
   because the sphere is trying to dock with wrong item.

3. DEBUG print messages under control of VERBOSE define

4. Reduced size of itemApproachOffset array from 6 to 3, since
   items of the same size will have the same offset

5. Added dockVelocityOk() and dockOrientOk() methods to help make sure
   dockItem() call will not result in penalty

6. Modified getClosestAvailableItem(), adding an argument that specifies
   the greatest item index it will look for.
         For example, to restrict search to large items, call with LARGE_2.
         To search for large and medium items, call MEDIUM_2
         To search for all, call SMALL_2.

7. Updated getItemApproachInfo(). Calculates which face of the item cube
   is closest to the SPHERE and sets that as the target position.

8. Added function pickUpItem(), that checks velocity, position and orientation
   before calling dockItem.

9. Approach to assembly zone uses vector math to calculate a direct
   route and orientation. See lines 601-607

10. Code starting at line 646, waits at landing zone, waiting for 2nd large item
    to be available.

PERFORMANCE
===========
Uses same strategy, picks up sphere then drops SPS.
Experiments with different strategies of dropping SPS before picking up
large item, didn't show any advantage. On the contrary more matches were lost.

Against original code wins majority of matches. (note not all)

Still runs out of fuel.

Check there is no out of bounds penalty.

Overall should be more robust to dock and not get caught in
loop incurring repeated incorrect docking penalties as the old
code sometime did.

Codesize usage : 85% (has to be less than 100% for valid submission)
*/

//Uncomment following line for detailed debug messages
//#define VERBOSE 1

#define NUMBER_OF_ITEMS 6
#define NUMBER_OF_ITEM_TYPES 3
#define NUMBER_OF_SPS   3
#define OPPONENT_ID     2

#define X_COORD 0
#define Y_COORD 1
#define Z_COORD 2
#define X_VELOCITY 3
#define Y_VELOCITY 4
#define Z_VELOCITY 5
#define X_ORIENT 6
#define Y_ORIENT 7
#define Z_ORIENT 8

//Item IDs
#define LARGE_1  0
#define LARGE_2  1
#define MEDIUM_1 2
#define MEDIUM_2 3
#define SMALL_1  4
#define SMALL_2  5

//Option passed to functions
//indicating if they can increment
//the step counter
#define STEP_INC    1
#define STEP_NO_INC 0

#define MAX_DOCK_VELOCITY  0.01
#define MAX_DOCK_ANGLE     0.10

//Target positions to drop off the SPS
float spsPosn[NUMBER_OF_SPS][3];
int counter;

//Coordinates of the center of each item
float itemPosn[NUMBER_OF_ITEMS][3];

//In order to pick up the item, the target
//coordinates of the sphere must be offset
//from the center of the item
float itemApproachOffset[NUMBER_OF_ITEM_TYPES];

//Initialized to all 1s, set to 0, when
//corresponding item no longer available
//already picked up you or opponent.
int itemAvailable[NUMBER_OF_ITEMS] ;

float diffVec[3];

//Sphere orientations
float faceDown[3];
float faceUp[3];
float faceLeft[3];
float faceRight[3];
float faceZDown[3];
float faceZUp[3];
float largeitem[3];
float zoneInfo[4];
float myState[12];
float otherState[12];
float zonePosn[3];
float zoneCenterOffset ;
float vectorBetween[3];
int step ;

float tolerance ;
float spsTolerance ;
float zonePosnTolerance ;
float pickUpTolerance ;

float posn[3];
float orient[3];
int item_id;
float err;
float dist ;
float mydistance;
float opponentdistance;
float itemDist[6];
int item_large_id ;
int item_any_id;

void init(){
  //This function is called once when your code is first loaded.
  //IMPORTANT: make sure to set any variables that need an initial value.
  //Do not assume variables will be set to 0 automatically!
  step = 1;

  tolerance = 0.02;
  zoneCenterOffset = 0.15;
  zonePosnTolerance = 0.02;
  pickUpTolerance = 0.02;

  counter = 0;

  //offset differ only by the type of items
  //These defines are part of the game api
  itemApproachOffset[ITEM_TYPE_LARGE]=0.16;
  itemApproachOffset[ITEM_TYPE_MEDIUM]=0.15;
  itemApproachOffset[ITEM_TYPE_SMALL]=0.14;

  api.getMyZRState(myState);

  //We are the blue sphere
  if(myState[1] >= 0.140){
    spsPosn[0][0] =  0.30;
    spsPosn[0][1] =  0.50;
    spsPosn[0][2] =  0.30;

    spsPosn[1][0] = -0.45;
    spsPosn[1][1] =  0.30;
    spsPosn[1][2] =  0.30;

    spsPosn[2][0] = -0.50;
    spsPosn[2][1] = -0.35;
    spsPosn[2][2] =  0.0;

    largeitem[0] = -0.23;
    largeitem[1] = -0.23;
    largeitem[2] = -0.23;
  }
  //We are the red sphere
  else {
    spsPosn[0][0] =  -0.40;
    spsPosn[0][1] = -0.30;
    spsPosn[0][2] =  -0.30;

    spsPosn[1][0] =  0.55;
    spsPosn[1][1] =  -0.30;
    spsPosn[1][2] =  -0.30;

    spsPosn[2][0] = -0.50;
    spsPosn[2][1] = -0.35;
    spsPosn[2][2] =  0.0;

    largeitem[0] = 0.23;
    largeitem[1] = 0.23;
    largeitem[2] = 0.23;
  }

  //Larger values will get sphere to drop SPS
  //before reaching target position
  spsTolerance = 0.30 ;

  //Initialize itemPosn array
  for(int i=0;i<NUMBER_OF_ITEMS;i++) {
    game.getItemLoc( itemPosn[i] , i);
  }
  faceZDown[0] = 0.0;
  faceZDown[1] = 0.0;
  faceZDown[2] = -1.0;
  faceZUp[0] = 0.0;
  faceZUp[1] = 0.0;
  faceZUp[2] = 1.0;
  faceDown[0] = 0.0;
  faceDown[1] = -1.0;
  faceDown[2] = 0.0;
  faceUp[0] = 0.0;
  faceUp[1] = 1.0;
  faceUp[2] = 0.0;
  faceLeft[0] = -1.0;
  faceLeft[1] =  0.0;
  faceLeft[2] = 0.0;
  faceRight[0] = 1.0;
  faceRight[1] = 0.0;
  faceRight[2] = 0.0;

  zonePosn[0]=0.0;
  zonePosn[1]=0.0;
  zonePosn[2]=0.0;

  //Initially all items are available
  for (int i=0;i<NUMBER_OF_ITEMS;i++)
    itemAvailable[i] = 1;
}

//Compute distance and orientation between 2 coordinates
//Returns distance and updates orient[] array
//Use mathVec library functions to reduce size
//of code, was 111% of max before this change,
//now is 89%
float computeDistance( float a[], float b[]) {
  float c[3];
  //vector c = vector a - vector b
  mathVecSubtract(c,a,b,3);
  return mathVecNormalize(c,3);
}

//Function to check my sphere's velocity below threshold to dock
//since negative points if you call dock
//when you are too fast
//Returns 1 when less 0 otherwise
int dockVelocityOk() {
  float vel[3];
  float vmag ;

  //Velocity is in indices 3,4,5
  vel[0] = myState[3];
  vel[1] = myState[4];
  vel[2] = myState[5];

  vmag = mathVecMagnitude(vel,3);

    #ifdef VERBOSE
  DEBUG(("SPHERE:Velocity = %f\n",vmag));
    #endif

  return (vmag < MAX_DOCK_VELOCITY) ;
}

//Function to check if dock orientation ok
//returns 1 if ok, 0 otherwise
int dockOrientOk( float targetorient[] ) {
  float myorient[3];
  float angle ;

  //Orient in indices 6,7,8
  myorient[0] = myState[6];
  myorient[1] = myState[7];
  myorient[2] = myState[8];

  //these should already be normalized ?
  mathVecNormalize(myorient,3);
  mathVecNormalize(targetorient,3);

  //Angle between 2 vectors that are normalized is
  //the acos of the inner(dot) product
  angle = acosf(mathVecInner( myorient, targetorient,3 ));

    #ifdef VERBOSE
  DEBUG(("SPHERE:Angle = %f\n",angle));
    #endif

  return (angle < MAX_DOCK_ANGLE);
}

//The opponent could have moved taken items
//including those in your assembly zone
//so update item position before calling
//routine to find closest available item
//To account for the fact that an
//item that was previously placed in your
//assembly zone could have been moved out
//calculate the item position distance from
//the assembly zone, if > sps error, mark
//that item as available.
void updateItemPositions() {
  for (int i=0;i<NUMBER_OF_ITEMS;i++) {
    game.getItemLoc( itemPosn[i] , i);
    //zoneInfo[4] has the spsTolerance
    if ( computeDistance( itemPosn[i] , zoneInfo) > 0.3 )
      itemAvailable[i] = 1;
  }
}

//Based on current position of sphere, return ID of
//closest available item
//Argument: limitTypes if set to 1, will look for
//closest large items.
int getClosestAvailableItem(int limitTypes) {
  float minDist = 10.0;//Initialize to large value
  int minDistItem = NUMBER_OF_ITEMS + 1 ; //Initialize to ID out of bounds
  for(int i=0;i<=limitTypes;i++) {
    itemDist[i]=computeDistance(myState,itemPosn[i]);
    if ( game.hasItem(i) == OPPONENT_ID ) {
      DEBUG(("Opponent has item %d",i));
    } else
      if ( itemAvailable[i] ) {
	if ( minDist > itemDist[i]) {
	  minDist = itemDist[i] ;
	  minDistItem = i;
	}
      }
  }//for
  return minDistItem ;
}

int getLargeItem() {
  if(itemAvailable[0] == 1)
    {
      if(game.hasItem(0) == OPPONENT_ID)
        {
	  DEBUG(("Opponent has item 0 "));
        }
      else {
	return 0;
      }
    }
  if(itemAvailable[1] == 1)
    {
      if(game.hasItem(1) == OPPONENT_ID)
        {
	  DEBUG(("Opponent has item 1 "));
        }
      else {
	return 1;
      }
    }
  return 99;
}

void setFloatArray(float target[], float source[], int size) {
  for(int i=0;i<size;i++){
    target[i] = source[i];
  }
}

//No longer use simplistic face up/face down , now that we
//are in 3D.
//Compute vector from current position to target.
//Orientation will be the vector above normalized.
//Compute distance to all 6 faces of item and choose the shortest
void getItemApproachInfo( int itemId, float posn[], float orient[] ) {
  float dist ;
  float tpos[3];
  float offset ;
  float minDist = 10;
  int minDim = 4;
  int minDir = 0;
  float minOffset = 0;

  orient[0]=0;
  orient[1]=0;
  orient[2]=0;

  offset = itemApproachOffset[ game.getItemType(itemId) ];

  setFloatArray(posn, itemPosn[itemId],3);

  api.getMyZRState(myState);

  //Which direction do we approach the item cube from ?
  //Cube has 6 faces, take points offset from the cube face
  //in the X,Y and Z direction. Choose the point closest to
  //the sphere.
  for(int dim=0;dim<3;dim++) {
    tpos[0] = posn[0];
    tpos[1] = posn[1];
    tpos[2] = posn[2];
    for(int i=-1; i < 2; i+=2 ) {
      tpos[dim] = tpos[dim] + offset * i ;
      dist = computeDistance( tpos ,myState ) ;
      if ( dist < minDist ){
	minDim = dim ;
	minDist = dist ;
	minOffset = offset * i;
	minDir = i;
      }
    }
  }
  posn[minDim] = posn[minDim] + minOffset ;
  orient[minDim] = -1*minDir;

  //Should work since position is in first 3 indices of myState
  //even though myState itself is a higher dimensional array
  //dist = computeDistanceAndOrient( posn ,myState, orient) ;

  //shorten the distance
  //dist = dist - itemApproachOffset[ game.getItemType(itemId) ];

  //Compute the new diffVec
  //for (int i=0; i<3;i++ )
  //diffVec[i] = orient[i] * dist;

  //Target position is my position + diffVec
  //mathVecAdd( posn, myState, diffVec, 3);
}

//Sum of absolute error differences between 2 vectors
float absError( float a[], float b[], int n){
  float err=0;
  for(int i=0;i<n;i++)
    err += fabsf(a[i]-b[i]);
  return err;
}

//Seems like the absolute error works better
float goToPosition( float posn[] , float tolerance , int inc ) {
  float err;

  /*
    float vectorBetween[3];
    mathVecSubtract(vectorBetween,posn,myState,3);
    err = mathVecMagnitude(vectorBetween,3);
  */
  err = absError(myState,posn,3);

  if (err > tolerance)
    api.setPositionTarget(posn);
  else if(inc)
    step++;
    #ifdef VERBOSE
  DEBUG(("SPHERE:Position Error = %f\n",err));
    #endif

  return err ;
}

//Pick up item while checking docking criteria are met
void pickUpItem(int next_step) {
  err =  goToPosition(posn,pickUpTolerance,STEP_NO_INC);

    #ifdef VERBOSE
  DEBUG(("SPHERE:Posn   %f, %f, %f\n",posn[0],posn[1],posn[2]));
  DEBUG(("SPHERE:Orient %f, %f, %f\n",orient[0],orient[1],orient[2]));
    #endif

  api.setAttitudeTarget(orient);

  //Check speed, orientation and position are ok
  //important to avoid penalties
  //Call dockItem with specific id, this takes care of the case
  //when 2 items are close to each other, so that the system
  //knows exactly which item you want to dock with.
  if ( dockVelocityOk() && dockOrientOk(orient) && err < pickUpTolerance ) {

    if ( game.dockItem(item_id) ) {
      DEBUG(("SPHERE:Picked up item %d",item_id));
      step = next_step ;
    }
  }
  else {
    ;
                                                #ifdef VERBOSE
    DEBUG(("Item-Pickup:Incorrect orientation"));
                                                #endif
  }
}

float getOpponentDistToMyZone() {
  return computeDistance( otherState, zoneInfo ) ;
}

//Main program Loop
void loop(){
  //This function is called once per second.  Use it to control the satellite.
  api.getMyZRState(myState);
  api.getOtherZRState(otherState);
  //By default Position controller used PD and
  //Attitude controller uses PID
  //not changing it right now
  //api.setControlMode(CTRL_PD,CTRL_PID);
  //float err = fabsf(myState[0]-posn[0]) + fabsf(myState[1]-posn[1]) + fabsf(myState[2] - posn[2]) ;
  api.setControlMode(CTRL_PD,CTRL_PID);
  if (game.getFuelRemaining() < 13)
    {
      api.setPosGains(0.45,0.01,5.0);
    }
  else if (game.getFuelRemaining() < 50)
    {
      api.setPosGains(0.45,0.01,4.0);
    }
  else {
    api.setPosGains(0.45,0.01,3.0);
  }

  //P, I and D values for position controller
  //High P value will increase speed of movement
  //resulting in overshoot, counteract with a higher
  //D value.
  //Experimenting with a few values, came up with this
  //combination, that posted a score of upto 40 in some
  //games.
  // api.setPosGains(0.45,0.01,3.0);

  switch(step) {

  case 1:
    //api.setControlMode(CTRL_PD,CTRL_PID);
    //api.setPosGains(0.40,0.01,3.0);
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    game.dropSPS();
    step++;
    break ;

  case 2:
    updateItemPositions();
    item_id = getClosestAvailableItem(LARGE_2) ;
            #ifdef VERBOSE
    DEBUG(("Closest item is: %d",item_id));
            #endif
    getItemApproachInfo( item_id, posn, orient ) ;

    //Make sure item_id is valid and opponent doesn't have the item we
    //are targetting.
    if ( item_id < NUMBER_OF_ITEMS &&  game.hasItem(item_id) != OPPONENT_ID ) {
      pickUpItem(3);
    }
    break ;

  case 3:
    // api.setControlMode(CTRL_PD,CTRL_PID);
    //api.setPosGains(0.50,0.01,3.0);
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    //api.setControlMode(CTRL_PD,CTRL_PID);
    //api.setPosGains(0.6,0.01,3.0);
    goToPosition(spsPosn[0],spsTolerance, STEP_INC);
    break ;

  case 4:
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    game.dropSPS();
    step++;
    break;

  case 5:
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    goToPosition(spsPosn[1],spsTolerance, STEP_INC);
    break ;

  case 6:
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    game.dropSPS();
    if ( game.getZone(zoneInfo) ) {
      DEBUG(("ZoneInfo: %f,%f,%f,%f",zoneInfo[0],zoneInfo[1],zoneInfo[2],zoneInfo[3]));
      step++;
    }
    break ;

  case 7:
    //api.setControlMode(CTRL_PD,CTRL_PID);
    //api.setPosGains(0.5,0.01,3.0);
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    DEBUG(("item %d picked up by player %d",item_id,game.hasItem(item_id)));

    //Now that in 3D, use vector math to compute optimal position and orientation to
    //drop item in assembly zone.
    mathVecSubtract(orient, zoneInfo, myState,3);
    dist = mathVecNormalize( orient , 3);
    dist = dist - zoneCenterOffset ;
    for(int i=0;i<3;i++)
      diffVec[i] = orient[i] * dist ;
    mathVecAdd( zonePosn, myState, diffVec,3 );

    /*
            zonePosn[0] = zoneInfo[0]  ;
            zonePosn[2] = zoneInfo[2];
            if (zoneInfo[1] < 0) {
                setFloatArray(orient,faceDown,3);
                zonePosn[1] = zoneInfo[1] + zoneCenterOffset;
            }
            else {
                zonePosn[1] = zoneInfo[1] - zoneCenterOffset;
                setFloatArray(orient,faceUp,3);
            }
    */
    step++;
    break;

  case 8:
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    api.setAttitudeTarget(orient);
    goToPosition(zonePosn,zonePosnTolerance,STEP_INC);
    break;

  case 9:
            #ifdef VERBOSE
    DEBUG(("step %d",step));
            #endif
    game.dropItem();
    DEBUG(("Dropped item %d",item_id));
    //Indicate this item is no longer available
    itemAvailable[item_id] = 0;
    step++;//loop back to find next closest item
    break;

  case 10:
    // api.setPosGains(0.5,0.01,3.0);
    updateItemPositions();

    //Item indexes are from 0(LARGE_1) to 5(SMALL_2)
    //argument to getClosestAvailableItem specifies
    //to which item the search must be limited to.
    //SMALL_2 (the last index),
    //implies we are searching for all items.
    item_large_id = getClosestAvailableItem(LARGE_2);
    item_any_id   = getClosestAvailableItem(MEDIUM_2);

    //After dropping my large item can I go to a smaller item and back
    //or should I go after the other large item, which most likely
    //has been placed in the opponent's zone.
    if ( 3*itemDist[item_any_id] < itemDist[item_large_id] ) {
      item_id = item_any_id;
    }
    else
      item_id = item_large_id;

    //Is item_id valid ?
    if (item_id < NUMBER_OF_ITEMS) {

                #ifdef VERBOSE
      DEBUG(("Closest item is: %d",item_id));
                #endif

      getItemApproachInfo( item_id, posn, orient ) ;

      //Attempt to pick up, in the same step
      //if item ends up being picked up by opponent by
      //the time we get to it, we will move on to the next
      //Handle case where opponent sneaks in picks up item
      //before we do.
      if ( game.hasItem(item_id) != OPPONENT_ID ) {
	pickUpItem(7);
      }
      //step++;
    }
    break;

  }//switch
}
//End page main
