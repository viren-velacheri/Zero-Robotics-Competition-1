


// 2016.12.07 code size experiment and clean up (more)
//  (1) Removed OutOfBound check (less likely to happen due to better triangulation method)
//  (2) Removed SUPPORT_GUARD and otState
//  (3) Removed ORTHO, PROJECT, SAFENORM, CROSSN (ROTATE is kept but not called anymore)
//  (4) Improved simple priority function
//  (5) Removed a few unnecessary computations (RobotTrend, etc)
//  (6) Some bug fixes (in priority function, 1st item)

// 2016.12.07 code size experiment and clean up
//  (1) Separated pieces relating to function groups: CONE, ITEM1ST, DELIVERY_RETRY
//  (2) Submit first version with CONE, no ITEM1ST
//  (3) Implemented simpler priority function

// 2016.12.05 Triangle/Initial selection changed
//  (1) ApproachFast to compensate side move early and hard (due to rival interference)
//  (2) Move method is unified to one now, slightly conservative on turning points.
//  (3) Priority function adjusted for first pick: prefer first medium item, then steal.
//  (4) New stable yet simple triangulation (34-39s triangle, 75-80% fuel left)

// 2016.12.04 Unified move methods in one code base
//  (1) Cone and Orbit are in one code base now. Cone method is boiled down to 
//      more conservative turning points (with potentially couple more steps to dock)
//      enable macro CONE to switch to Cone method.
//  (2) Disabled spinning slowdown as it burns fuel.
//  (3) Turned on Guard/defense

// 2016.12.03 change
//  (1) Major enhancement in ApproachFast and delivery also use this method!!!
//  (2) Discourage first item on rival side due to robot interference and changing mind may cause bad triangle.
//  (3) Improved ApproachFast to suppress overspin.
//  (4) Fixed condition for using back turning point. Also better back turning point is used.
//  (5) Improved benchmark of docking time (for a set of tests) while the docking is more reliable.

#define CONE				// Support approaching from side or back (comment this out to only support direct docking)
#define ENABLE_ITEM1ST		// Support dual triangulation (triangle first and pickup first)
//#define DELIVERY_RETRY	// Defer delivery for 1 sec if next position is better
#define SIMPLE_PRIORITY		// Use simplified priority function (code size kills)

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) < (b) ? (b) : (a))
#define ASSIGN(a, b) memcpy(a, b, 12)
#define INIT3(v, c0, c1, c2) { v[0] = c0; v[1] = c1; v[2] = c2; }
#define MAG(a) mathVecMagnitude(a, 3)
#define NORM(p) mathVecNormalize(p, 3)
#define ADD(p, a, b) mathVecAdd(p, a, b, 3)
#define SUB(p, a, b) mathVecSubtract(p, a, b, 3)
#define CROSS(p, a, b) mathVecCross(p, a, b)
#define DOT(a, b) mathVecInner(a, b, 3)
#define DEBUG1(x) DEBUG(x)
#define DEBUG2(x) //DEBUG(x)
#define DEBUG3(x) //DEBUG(x)

// These are intentionally defined as functions not macros, to save space of code built

// get the normal direction from point B to A, returning the distance between A and B.
// out = Normalize(a - b)
float DIR(float *out, float *a, float *b) {
	SUB(out, a, b);
	return NORM(out);
}

// return distance between position A and B
float DIST(float *a, float *b) {
	float c[3];
	return DIR(c, a, b);
}

// scale vector out = in * #f
void SCALE(float *out, float *in, float f) {
	for (int i = 0; i < 3; i ++) out[i] = in[i] * f;
}

// add scaled vector to another: p = a + b*#c
void OFS(float *p, float *a, float *b, float c) {
	float d[3];
	SCALE(d, b, c);
	ADD(p, a, d);
}

// Rotate v along z of angle a in radians
// Changed last parameter from cosine angle to angle: so we can rotate a negative angle.
// This means caller most likely needs to call acosf, but it did not increase code size.
// Condition: v is perpendicular to z, v and z are unit vectors
// vp = v * cos(a) + (z x v) * sin(a)
void ROTATE(float *out, float *z, float *v, float a) {
	float v1[3];
	CROSS(v1, z, v);
	SCALE(v1, v1, sinf(a));
	OFS(out, v1, v, cosf(a));
}

// Declare any variables shared between functions here
// Alert! Global variable consumes more code space in both storage and use
short phase;			// 0..4 for SPS, 10, 30, 40, 50, 60 are working iterations
short choice;			// current choice item
short action;			// +1 move, +2 turn
short spsTime;			// next SPS drop time (left)
bool SPSMode;			// false when all 3 SPS dropped
#ifdef DELIVERY_RETRY
bool deliverTry;		// true when defer dropoff
#endif
#ifdef ENABLE_ITEM1ST
bool item1st;			// true to dock item before triangle
#endif
float moveTo[3];		// move control: target position of robot
float pointTo[3];		// move control: target direction of robot
float SPSLoc[3];		// Last dropped SPS location
float triangleL;		// how far the triangle to travel (set by ComputeTrianglePoint)
float dropError;		// error allowed to drop item in zone (will decrease on failed placement)

#define DEFEND_ZONE 0.4f	// if we are in defence zone, defend/fight
#define COEF_DOCKING 22.0f	// time scale for approaching item (c * sqrt(d))
#define COEF_DELIVER 19.0f	// time scale for delivering item (c * sqrt(d))
#define CONE_EFFORT 15.0f	// # extra seconds to work around the hardest case (robot is on the back of item)
#define DOCK_THRES	0.0109f	// half of the docking range (which is 0.022m for all item size)
#define DOCK_THRES2	0.0218f	// the docking range (which is 0.022m for all item size)
#define ITEM_SPEED	0.003f	// if item moves faster than this speed, we ignore it
#define DOCK_ANGLE	0.24f	// ideal 0.35 = cos(asin(0.151/0.162)), use 0.24 = cos(asin(0.1574/0.162))
#define FRONT_ANGLE 0.971f	// =0.1574/0.162 (cosine compliment of dock angle)
//#define DOCK_ANGLE 0.29f	// ideal 0.35 = cos(asin(0.151/0.162)), use 0.29 = cos(asin(0.155/0.162))
//#define FRONT_ANGLE 0.957f	// =0.155/0.162 (cosine compliment of dock angle)
#define HASITEM_US	1		// returned value of game.hasItem: we have it on hand
#define HASITEM_OP	2		// returned value of game.hasItem: opponent has it on hand

// phase
#define cPHSETITEM	10		// To select strategy and item while approaching
#define cPHGETITEM	20		// To lock in approaching and picking up item
#define cPHDELIVER	40		// To push item toward zone
#define cPHGUARD	60		// To guard and protect zone (currently does nothing)

// action
#define cMOVE		1		// position target
#define cTURN		2		// attitude target
#define cMOVENTURN	3		// MOVE+TURN
#define cSLOWDOWN	35		// MOVE+TURN+32

struct RobotTrend {
	float gap;		// distance from item to target (gap, itemTo)
	float dist;		// distance from robot to target (dist, distTo)
	float ofs;		// distance from robot to item
	float cosAtt;	// cosine angle between robot attitude and item pos
	float cosAtAtt;	// cosine angle between robot and item attitudes
	float speed;	// robot speed
};

// Compute robot approaching parameters
//   robot: May be my robot, rival robot
//   item: May be my robot, rival robot, item, or item docking point
//   target: May be zone or item docking point
//   state: returned robot trend data
//   distTo: returned direction from robot to target (dist, distTo)
//   itemTo: returned direction from item to target (gap, itemTo)
void AnalyzeRobotTrend(float robot[12], float item[12], float *target, RobotTrend& state, float *distTo, float *itemTo) {
	state.ofs = DIR(itemTo, item, robot);
	state.cosAtt = DOT(itemTo, robot + 6);
	state.cosAtAtt = -DOT(item + 6, robot + 6);
	state.gap = DIR(itemTo, target, item);
	state.dist = DIR(distTo, target, robot);
	state.speed = MAG(robot + 3);
}

// Get item ZR state as well as computing the docking hotspot and minimum docking distance.
// If choice < 0 this function does nothing -- caller should not use the item state.
void GetItemStateAndDockSpot(float itState[12], float *dockPoint, float& dockMin, int choice) {
	unsigned char PickupMin[9] = {151, 151, 138, 138, 124, 124, 124, 0, 0};  // save code space?
	game.getItemZRState(itState, choice);
	// compute the docking hotspot and minimum docking range
	dockMin = PickupMin[choice] / 999.5f;   // we intentionally make this slightly bigger
	OFS(dockPoint, itState, itState + 6, dockMin + DOCK_THRES);
}

// Check if point p is out of bounds. gap = safe margin 
bool OutOfBound(float *p, float gap) {
	unsigned char bounds[3] = {64, 80, 64}; // in centimeters to save code space
	for (int i = 0; i < 3; i ++)
		if (fabsf(p[i]) > bounds[i] / 100.0f - gap) return true;
	return false;
}

// Compute next triangulation point, set target point and length.
// Requires the distance between sphere and dockPoint to be >0.45m.
// GLOBAL set: moveTo, triangleL
void ComputeTrianglePoint(float *myState, float *itemAtt, float *dockPoint) {
	float dockToMe[3], target[3], v1[3];
	float base = DIR(dockToMe, myState, dockPoint);
	CROSS(target, dockToMe, itemAtt);
	NORM(target);	// this is the rotation axis
	CROSS(v1, target, dockToMe);
	SCALE(v1, v1, 0.966f);          // sin(75)
	OFS(target, v1, dockToMe, 0.259f);  // cos(75)
	triangleL = (item1st ? 0.28 : 0.24f) / base;	// target area is 0.12 or SPS error = 0.08
	OFS(moveTo, dockPoint, target, 0.8f);
	DEBUG1(("Triangle: it=%d d=%.3f l=%.3f dir(%.1f,%.1f,%.1f)", choice, base, triangleL, target[0], target[1], target[2]));
}

// This function drops last SPS, get Zone data dn ready for item delivery.
void dropLastSPS(float myState[12], float zone[4])
{
	game.dropSPS();
	SPSMode = false;
#ifdef ENABLE_ITEM1ST
	item1st = false;
#endif
	game.getZone(zone);
}

// 90.0, 1.3x, 0.15 is faster but consumes more fuel
#define BRAKE_COEF  100.0f		// equals mass / 2 / force (bigger means smaller force needed)
#define ACCEL_COEF  0.25f		// increase target distance to make robot move quicker
#define ACCEL_XTRA  0.1f		// extra distance in case it is close to target

// Will check braking distance to determine the target position.
// If the distance is longer than braking distance, move at full speed.
// Otherwise slow down so that we can achieve target speed at target position.
// Speed target is given as target kinematic energy = BRAKE_COEF * V^2 (codesave=16)
// The goal is to get the target speed when the sphere gets to target position.
// GLOBAL writes: moveTo, pointTo, action	
void ApproachFast(float myState[12], float *targetPos, float slowRange, float targetKe)
{
	float dirTo[3], *velo = myState + 3;
	float dist = DIR(dirTo, targetPos, myState);
	float vAtt = DOT(velo, dirTo);
	float brakeDist = BRAKE_COEF * vAtt * vAtt - targetKe;
	if (action == cSLOWDOWN) brakeDist += brakeDist + slowRange; // discourage switching back to accel mode
	bool accel = (dist > slowRange && brakeDist < dist);  // true for fast move otherwise slowdown
	OFS(moveTo, targetPos, dirTo, accel ? dist * ACCEL_COEF + ACCEL_XTRA : targetKe);
	action = accel ? cMOVENTURN : cSLOWDOWN;
	// compensate for side speed
	if (dist < 0.2f) {
		OFS(dirTo, velo, dirTo, -vAtt);
		OFS(moveTo, moveTo, dirTo, -10.0f);
	}
}

// This function is called once when your code is first loaded.
void init() {
	dropError = 0.075f;
	phase = 0;
	SPSMode = true;
#ifdef ENABLE_ITEM1ST
	item1st = false;
#endif
	spsTime = 0;
}

//This function is called once per second.
void loop() {
	// Many highly used variables are still kept local for smaller code size
	RobotTrend tr;			// this is the robot target data
	int timeLeft = 180 - api.getTime();
	float myState[12];		// my robot state
	float itState[12];		// item states
	float zone[4];			// zone location and error
	float itemTo[3];		// this is the target rotation speed
	float dockPoint[3];		// item dock hotspot
	float dockMin = 0.0f;	// item pickup min-distance
	float *att = itState + 6;
	api.getMyZRState(myState);
	
	// Phase=0..4 are for a fixed triangulation. It could be disturbed by rival robot.
	if (timeLeft == 180) { // drop 1st SPS and set target for 2nd.
		DEBUG(("--Alliance SANTA (1.4.0)--"));
		game.dropSPS();
		ASSIGN(SPSLoc, myState);
		action = cMOVE;
	}
	
	game.getZone(zone);

	if (timeLeft == 180 || phase == cPHSETITEM) {
		// Priority computation on startup may suggest to pick up item first (item1st=true).
		ChooseStrategyAndItem(myState, zone, timeLeft);
#ifdef ENABLE_ITEM1ST
		if (phase == 0) phase = item1st ? cPHGETITEM : 1;
#else
		if (phase == 0) phase = 1;
#endif
	}
	// get the item state (if choice is none, item state and dockPoint are unassigned)
	GetItemStateAndDockSpot(itState, dockPoint, dockMin, choice);

	if (phase == 1) { // set target position for 2nd SPS
		ComputeTrianglePoint(myState, att, dockPoint);
		phase = 2;
	}
	
#ifdef ENABLE_ITEM1ST
	if (item1st) {
		// Item has been picked up. Now we need to place 2nd and 3rd SPS.
		// 2nd SPS will be extending current direction until L > 0.5m
		// Last SPS will be based on # moves as the target position has made it predictable.
		if (phase == 2 && DIST(myState, SPSLoc) > 0.5f) {
		    SCALE(att, SPSLoc, -6.667f);
			ComputeTrianglePoint(myState, att, SPSLoc);
			action = cMOVE;
			spsTime = timeLeft - 2; // Drop SPS in 2 seconds (to get bigger final SPS triangle)
			phase = 3;
		}
		if (phase == 4 && DIST(myState, SPSLoc) > triangleL) {
			dropLastSPS(myState, zone);
			phase = cPHDELIVER;
		}
	} else
#endif
	{
		// This is to draw triangle first.
		// 2nd SPS will be placed at pre-computed distance.
		// Last SPS will be placed when we pick up the item (or late -- we failed to get item)
		if (phase == 2 && DIST(myState, SPSLoc) > triangleL) {
			spsTime = timeLeft - 3; // Drop SPS in 3 seconds (to get bigger final SPS triangle)
			phase = cPHGETITEM;
		}
	}
	
	if (timeLeft == spsTime) {  // This is the deferred point
		game.dropSPS();
		ASSIGN(SPSLoc, myState);
		if (phase == 3) phase++;
	}
	
	if (phase == cPHSETITEM || phase == cPHGETITEM) { // Approach item
		float approach[3];	// vector from sphere to docking hotspot
		AnalyzeRobotTrend(myState, itState, dockPoint, tr, approach, itemTo);
		DEBUG2(("P%d..d=%.3f v=%.3f att=%.0f", phase, tr.ofs, tr.speed, acosf(tr.cosAtt) * 57.3f));
		DEBUG3(("item state: %d %.3f,%.3f,%.3f %.3f,%.3f,%.3f", choice, itState[0], itState[1], itState[2], itState[6], itState[7], itState[8]));
		if (PickupItem(tr, dockMin, itState)) {
#ifdef DELIVERY_RETRY
			deliverTry = false;
#endif
#ifdef ENABLE_ITEM1ST
			if (item1st) {  // also means SPS not done yet
				DIR(pointTo, myState, SPSLoc);
				OFS(moveTo, SPSLoc, pointTo, 0.85f);
				action = cMOVE;
				phase = 2;
			} else
#endif
			phase = cPHDELIVER;
		} else {
			SCALE(pointTo, att, -1.0f);
#ifdef CONE
			float r = dockMin + DOCK_THRES;
			DIR(itemTo, myState, itState);
			float cosA = DOT(att, itemTo);
			// See illustration and macros for detail.
			// Direct approaching will interfere with item: need to use turning point(s)
			// Added logic: if we already interfere with item we must be in assembly zone!
			// Just dock directly because interfering is OK in assembly zone.
			// Cone has same logic as Orbit method, but the turing points are more conservative.
			if (cosA < FRONT_ANGLE && DOT(approach, att) > DOCK_ANGLE && tr.ofs > 0.12f) {
				float yAxis[3];
				OFS(yAxis, itemTo, att, -cosA);
				NORM(yAxis);
				float x = tr.ofs * cosA;
				float y = tr.ofs * DOT(yAxis, itemTo);
				// If we are at the back of attitude, need to go to the back turning point first
				// otherwise go directly toward the front turning point
				bool backPt = y < r && x < 0.0f;
				OFS(itemTo, itState, yAxis, backPt ? r : min(r, y));
				OFS(itemTo, itemTo, att,  backPt ? max(-r, x) : r);
				// Back turning point target speed is 0.04 (Ke=0.16)
				// Front turning point target speed is 0.015 (Ke=0.0225)
				// Slow down range is 0.015
				ApproachFast(myState, itemTo, 0.015f, backPt ? 0.16f : 0.022f);
				DEBUG1(("Orbit%d %d: it=%d d=%.3f,%.3f v=%.3f vatt=%.3f att=%.0f atatt=%.0f rot=%.0f", backPt, action, choice, tr.ofs, DIST(myState, itemTo), tr.speed, DOT(myState + 3, att), acosf(tr.cosAtt) * 57.3f, acosf(tr.cosAtAtt) * 57.3f, MAG(myState+9) * 57.3f));
			} else
#endif
			{ // direct docking
				// docking point target speed is 0, means you will be less than 0.01 when entering docking zone
				// slow down range is 0.05 (increase this for lower risk of failed slowdown)
				ApproachFast(myState, dockPoint, 0.05f, 0.0f);
				DEBUG1(("Direct %d: it=%d d=%.3f,%.3f v=%.3f vatt=%.3f att=%.0f atatt=%.0f rot=%.0f", action, choice, tr.ofs, DIST(myState, dockPoint), tr.speed, DOT(myState + 3, att), acosf(tr.cosAtt) * 57.3f, acosf(tr.cosAtAtt) * 57.3f, MAG(myState+9) * 57.3f));
			}
		}
	}

#ifdef ENABLE_ITEM1ST
	if (!item1st)
#endif
		if ((timeLeft == 135 || phase == cPHDELIVER) && SPSMode)
			dropLastSPS(myState, zone);

	if (phase == cPHDELIVER) { // deliver item to zone
		if (game.getNumSPSHeld() > 0) dropLastSPS(myState, zone);   // in case last SPS failed to drop due to out of range
		AnalyzeRobotTrend(myState, itState, zone, tr, pointTo, itemTo);
		DEBUG1(("P%d %d gap=%.3f d=%.3f v=%.3f att=%.0f", phase, action, tr.gap, tr.dist, tr.speed, acosf(tr.cosAtt) * 57.3f));
		if (DropItem(tr, itState, myState, zone)) return;
		if (tr.dist > 0.05f) {
			OFS(dockPoint, zone, pointTo, -tr.ofs);
		} else {
			OFS(dockPoint, zone, itemTo, -tr.ofs);
			ASSIGN(pointTo, itemTo);
		}
		// Slow down range is 0.22, target speed is 0.03 (Ke=0.09)
		ApproachFast(myState, dockPoint, 0.22f, 0.09f);
	}

	if (phase != cPHGUARD) {
		if (action & cMOVE) api.setPositionTarget(moveTo);
		if (action & cTURN) api.setAttitudeTarget(pointTo);
	}
	DEBUG2(("act=%d moveTo = %.2f,%.2f,%.2f v=%.3f", action, moveTo[0], moveTo[1], moveTo[2], MAG(myState+3)));
	DEBUG3(("ph=%d it=%d act=%d tm=%d hasItem[%d %d %d %d %d %d]", phase, choice, action, timeLeft,
		game.itemInZone(0) ? -1 : game.hasItem(0),
		game.itemInZone(1) ? -1 : game.hasItem(1),
		game.itemInZone(2) ? -1 : game.hasItem(2), 
		game.itemInZone(3) ? -1 : game.hasItem(3), 
		game.itemInZone(4) ? -1 : game.hasItem(4), 
		game.itemInZone(5) ? -1 : game.hasItem(5)));
}

#define MIN_DOCKDIST	0.45f	// Begin: if distance is smaller than this we will dock item first
#define VAL_REJECT		-2.0f	// 
#define VAL_REJECTA		-20.0f

float ValueOfMove(int i, float *myState, float *zone, float *aItState, float *dockPnt, float value, short timeLeft) {
	// Calculate distances between us/item/target and opponent/item.
	float dirToDock[3], d1;
	d1 = DIR(dirToDock, dockPnt, myState);
	float cosAtt = DOT(dirToDock, aItState + 6);
	// Modified logic to support triangulation along with docking (phase=0)
	// Only pursue large and medium items sitting on our side or direct docking on rival side.
	// If item can be directly docked, will give 50+ as highest priority (item1st=true)
	// Otherwise will triangulate first and regular value calculation is used.
	// Triangulate-first item minimum distance is 0.45m (safe triangle will not go outbound).
	// Simplified: on average you will deliver first item in 45 seconds.
#ifdef CONE
	if (timeLeft == 180) {	// special for startup item: only big/medium item
		if (i > 3) return VAL_REJECTA;
#ifdef ENABLE_ITEM1ST
		if (cosAtt <= DOCK_ANGLE && d1 < 0.5f) // direct dockable nearby item high priority
			return 500.0f * value;  // make a value bigger than 50
#endif
		// too close, or more than 0.2 from Y=0 plane on the opposite side
		if (d1 < MIN_DOCKDIST || aItState[1] * myState[1] < -0.03f) return VAL_REJECTA;
	}
#else
	if (timeLeft == 180) {	// special for startup item: only big item on our side
		if (i > 3) return VAL_REJECTA;
#ifdef ENABLE_ITEM1ST
		if (cosAtt <= DOCK_ANGLE && d1 < 0.5f)
			return 500.0f * value;  // make a value bigger than 50
#endif
		// too close, or more than 0.2 from Y=0 plane on the opposite side
		if (d1 < MIN_DOCKDIST || aItState[1] * myState[1] < -0.03f) return VAL_REJECTA;
	}
	// cannot dock directly without CONE method
	if (cosAtt > DOCK_ANGLE) return VAL_REJECTA;
#endif
#ifdef SIMPLE_PRIORITY
	float fuel = game.getFuelRemaining();
	float fuelEst1 = 8.0f * d1;
	float pickupTime = 20.0f * sqrtf(d1);
	float eval = fuel > 2.0f * fuelEst1 ? (timeLeft - 2.0f * pickupTime) * value : 0.0f;    // estimated gain
	if (game.hasItemBeenPickedUp(i) && fuel > fuelEst1) eval += (timeLeft - pickupTime) * value; // add stealing benefit
	DEBUG1(("..ch: %d=%.2f, %d d=%.2f #mv=%.0f #fu=%.0f ev=%.2f", i, value, game.hasItemBeenPickedUp(i), d1, pickupTime*2.0f, fuelEst1*2.0f, eval));
#else
	float pickupTime = 3.0f + COEF_DOCKING * sqrtf(d1);
	if (cosAtt > DOCK_ANGLE)
		pickupTime += CONE_EFFORT * (1.0f - 0.76f * acosf(cosAtt));
	pickupTime -= 30.0f * DOT(myState + 3, dirToDock);
	float fuel = 1.0f + game.getFuelRemaining();
	float d2 = SPSMode ? 0.0f : DIST(dockPnt, zone);
	float totalTime = pickupTime + COEF_DELIVER * sqrtf(d2);
	float fuelEst1 = 0.02f * pickupTime * pickupTime;
	float fuelEst = fuelEst1 + 12.0f * d2;
	float eval = (fuel > fuelEst) ? (timeLeft - totalTime) * value : (fuel < fuelEst1 ? VAL_REJECT : 0.0f);
	if (game.hasItemBeenPickedUp(i)) {
		if (fuel > fuelEst1)eval += (timeLeft - pickupTime) * value;    // add stealing benefit
	}
	DEBUG1(("..ch: %d=%.2f, %d d=%.2f %.2f #mv=%.0f #fu=%.0f, ev=%.2f", i, value, game.hasItemBeenPickedUp(i), d1, d2, totalTime, fuelEst, eval));
#endif
	return eval;
}

// Determine the strategy & item based on the current condition: score, fuel, location, gain
// initially the strategy is to move or steal item
// later the strategy is to defend or guard
// JINBO GLOBAL WRITES: choice, item1st, phase=pGuard.
void ChooseStrategyAndItem(float *myState, float *zone, int timeLeft) {
	char ItemValues[7] = {4, 4, 3, 3, 2, 2, 2}; // save code space?
	float aItState[12], dockPoint[3], dockMin;
	float eval, best = -2.1f;
	choice = -1;
	for (int i = 0; i < 6; i++) {
		if (game.itemInZone(i)) continue;
		if (game.hasItem(i) == HASITEM_OP) continue;
		GetItemStateAndDockSpot(aItState, dockPoint, dockMin, i);
		if (MAG(aItState + 3) > ITEM_SPEED) continue;
		eval = ValueOfMove(i, myState, zone, aItState, dockPoint, 0.05f * ItemValues[i], timeLeft);
		if (eval > best) {
			best = eval;
			choice = i;
		}
	}
#ifdef ENABLE_ITEM1ST
	if (best > 50.0f) item1st = true;
#endif
	if (choice < 0) phase = cPHGUARD; // nothing to choose, guard in zone
}

// Pick up chosen item ONLY when the docking criteria is met (avoid penalty)
// Alert! Floating comparison is not accurate. Limits are tightened by 0.001.
// Return true if item is successfully picked up.
bool PickupItem(RobotTrend& tr, float dockMin, float *itState) {
	int hasItem = game.hasItem(choice);
	if (hasItem == HASITEM_US) return true;
	if (hasItem == HASITEM_OP) {
		phase = cPHSETITEM;
		return false;
	}
	// lock to this item when robot is speeding toward or is getting close to item
	if (tr.dist < 0.25f || tr.speed > 0.06f) phase = cPHGETITEM;
	// 0.985 is cos(10deg) need to include attitude error
	// 0.969 is cos(0.25rad) original docking condition
	if (tr.ofs > dockMin + DOCK_THRES2 || tr.ofs < dockMin || tr.cosAtt < 0.969f || tr.speed > 0.0099f) return false;
	return (game.isFacingCorrectItemSide(choice) && game.dockItem(choice));
}

// AachenerNerds: this logic can be simplified now, because there is only one try to drop item
// Singularity: actually this extra check proved to improve delivery accuracy due to zone error.
// Try to unload if it is in zone range
bool DropItem(RobotTrend& tr, float *itemPos, float *myState, float *zone) {
	if (tr.gap <= dropError) {
#ifdef DELIVERY_RETRY
		if (tr.gap > 0.055f && !deliverTry) {
			float v[3];
			ADD(v, itemPos, &myState[3]);
			// defer delivery if next improvement is at least 0.005f (there is risk due to turning)
			if (DIST(zone, v) < tr.gap - 0.005f) {
				DEBUG1(("**drop soon: %.3f >> %.3f", tr.gap, DIST(zone, v)));
				deliverTry = true;
				return false;
			}
			DEBUG1(("**drop now: %.3f >> %.3f", tr.gap, DIST(zone, v)));
		}
#endif
		game.dropItem();
		DEBUG(("Drop %d in zone (%d): gap = %.3f / %.3f", choice, game.itemInZone(choice), tr.gap, dropError));
		if (!game.itemInZone(choice)) dropError -= 0.02f;
		phase = cPHSETITEM; // try next item
		return true;
	}
	return false;
}
