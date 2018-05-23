/*
 * UavMobility.cc
 *
 *  Created on: 2018年5月14日
 *      Author: Jch
 */
#include "inet/mobility/uav/UavMobility.h"
#include "inet/common/INETMath.h"
#include "inet/applications/udpapp/uav/packet/UavPacket_m.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "inet/applications/udpapp/uav/UAVUdpSink.h"

namespace inet {

Define_Module(UavMobility);

UavMobility::UavMobility() :
    uavScript(nullptr),
    nextStatement(nullptr),
    speed(0),
    angle(0),
    borderPolicy(REFLECT),
    maxSpeed(0)
{
}

void UavMobility::initialize(int stage)
{
    LineSegmentsMobilityBase::initialize(stage);

    EV_TRACE << "initializing UavMobility stage " << stage << endl;
    leader = par("leader");
    Ref_x = par("Ref_x");
    Ref_y = par("Ref_y");

    if (stage == INITSTAGE_LOCAL) {
        WATCH(speed);
        WATCH(angle);
        WATCH(borderPolicy);
        computeMaxSpeed(par("uavScript"));
    }
}

void UavMobility::setInitialPosition()
{
    LineSegmentsMobilityBase::setInitialPosition();

    uavScript = par("uavScript");
    nextStatement = uavScript->getFirstChild();

    speed = 1;
    angle = 0;
    borderPolicy = REFLECT;

    // a dirty trick to extract starting position out of the script
    // (start doing it, but then rewind to the beginning)
    nextChange = simTime();
    resumeScript();
    targetPosition = lastPosition;
    nextChange = simTime();
    nextStatement = uavScript->getFirstChild();

    while (!loopVars.empty())
        loopVars.pop();
}

void UavMobility::setTargetPosition()
{
    resumeScript();
}

void UavMobility::move()
{
    EV_INFO << "leader: "<<leader<< endl;
    /*****************************修改*********************************/
        if(leader == 0){
            //集群坐标系到全局坐标系转化方法
            targetPosition.x = leaderPosition().x + Ref_x*cos(-M_PI*angle/180) - Ref_y*sin(-M_PI*angle/180);
            targetPosition.y = leaderPosition().y + Ref_x*sin(-M_PI*angle/180) - Ref_y*cos(-M_PI*angle/180);
            EV_INFO << "jinchenghao!!! "<< targetPosition<<endl;
    }
    /*****************************结束*********************************/

    LineSegmentsMobilityBase::move();

    /*****************************修改*********************************/
//    if(leader == 0){
//            targetPosition.x -= 10;
//            targetPosition.y -= 10;
//        }
    /*****************************结束*********************************/

    Coord dummy;
    handleIfOutside(borderPolicy, targetPosition, dummy, angle);
}

/**
 * Will set a new nextChange and targetPosition.
 */
void UavMobility::resumeScript()
{
    simtime_t now = simTime();

    while (nextChange == now) {
        if (nextStatement != nullptr) {
            executeStatement(nextStatement);
            gotoNextStatement();
        }
        else {
            nextChange = -1;
            stationary = true;
            targetPosition = lastPosition;
        }
    }
}

void UavMobility::executeStatement(cXMLElement *stmt)
{
    ASSERT(nextChange != -1);
    const char *tag = stmt->getTagName();

    EV_DEBUG << "doing <" << tag << ">\n";

    if (!strcmp(tag, "repeat")) {
        const char *nAttr = stmt->getAttribute("n");
        long n = -1;    // infinity -- that's the default

        if (nAttr) {
            n = (long)getValue(nAttr);

            if (n < 0)
                throw cRuntimeError("<repeat>: negative repeat count at %s", stmt->getSourceLocation());
        }

        loopVars.push(n);
    }
    else if (!strcmp(tag, "set")) {
        const char *speedAttr = stmt->getAttribute("speed");
        const char *angleAttr = stmt->getAttribute("angle");
        const char *xAttr = stmt->getAttribute("x");
        const char *yAttr = stmt->getAttribute("y");
        const char *bpAttr = stmt->getAttribute("borderPolicy");

        if (speedAttr)
            speed = getValue(speedAttr);

        if (angleAttr)
            angle = getValue(angleAttr);

        if (xAttr){
            targetPosition.x = lastPosition.x = getValue(xAttr);
            /*****************************修改*********************************/
//             if(leader == 0){
//                 targetPosition.x += Ref_x;
//                 lastPosition.x += Ref_x;
//              }
            /*****************************结束*********************************/
        }

        if (yAttr){
             targetPosition.y = lastPosition.y = getValue(yAttr);
//            /*****************************修改*********************************/
//             if(leader == 0){
//                  targetPosition.y += Ref_y;
//                  lastPosition.y += Ref_y;
//             }
            /*****************************结束*********************************/
        }


        if (speed <= 0)
            throw cRuntimeError("<set>: speed is negative or zero at %s", stmt->getSourceLocation());

        if (bpAttr) {
            if (!strcmp(bpAttr, "reflect"))
                borderPolicy = REFLECT;
            else if (!strcmp(bpAttr, "wrap"))
                borderPolicy = WRAP;
            else if (!strcmp(bpAttr, "placerandomly"))
                borderPolicy = PLACERANDOMLY;
            else if (!strcmp(bpAttr, "error"))
                borderPolicy = RAISEERROR;
            else
                throw cRuntimeError("<set>: value for attribute borderPolicy is invalid, should be "
                                    "'reflect', 'wrap', 'placerandomly' or 'error' at %s",
                        stmt->getSourceLocation());
        }
    }
    else if (!strcmp(tag, "forward")) {
        const char *dAttr = stmt->getAttribute("d");
        const char *tAttr = stmt->getAttribute("t");

        if (!dAttr && !tAttr)
            throw cRuntimeError("<forward>: must have at least attribute 't' or 'd' (or both) at %s", stmt->getSourceLocation());

        double d, t;

        if (tAttr && dAttr) {
            // cover distance d in time t (current speed is ignored)
            d = getValue(dAttr);
            t = getValue(tAttr);
        }
        else if (dAttr) {
            // travel distance d at current speed
            d = getValue(dAttr);
            t = d / speed;
        }
        else {    // tAttr only
                  // travel for time t at current speed
            t = getValue(tAttr);
            d = speed * t;
        }

        if (t < 0)
            throw cRuntimeError("<forward>: time (attribute t) is negative at %s", stmt->getSourceLocation());

        if (d < 0)
            throw cRuntimeError("<forward>: distance (attribute d) is negative at %s", stmt->getSourceLocation());

        // FIXME handle zeros properly...
        targetPosition.x += d * cos(M_PI * angle / 180);
        targetPosition.y += d * sin(M_PI * angle / 180);
        nextChange += t;
    }
    else if (!strcmp(tag, "turn")) {
        const char *angleAttr = stmt->getAttribute("angle");

        if (!angleAttr)
            throw cRuntimeError("<turn>: required attribute 'angle' missing at %s", stmt->getSourceLocation());

        angle += getValue(angleAttr);
    }
    else if (!strcmp(tag, "wait")) {
        const char *tAttr = stmt->getAttribute("t");

        if (!tAttr)
            throw cRuntimeError("<wait>: required attribute 't' missing at %s", stmt->getSourceLocation());

        double t = getValue(tAttr);

        if (t < 0)
            throw cRuntimeError("<wait>: time (attribute t) is negative (%g) at %s", t, stmt->getSourceLocation());

        nextChange += t;    // targetPosition is unchanged
    }
    else if (!strcmp(tag, "moveto")) {
        const char *xAttr = stmt->getAttribute("x");
        const char *yAttr = stmt->getAttribute("y");
        const char *tAttr = stmt->getAttribute("t");

        if (xAttr)
            targetPosition.x = getValue(xAttr);

        if (yAttr)
            targetPosition.y = getValue(yAttr);

        // travel to targetPosition at current speed, or get there in time t (ignoring current speed then)
        double t = tAttr ? getValue(tAttr) : lastPosition.distance(targetPosition) / speed;

        if (t < 0)
            throw cRuntimeError("<wait>: time (attribute t) is negative at %s",
                    stmt->getSourceLocation());

        nextChange += t;
    }
    else if (!strcmp(tag, "moveby")) {
        const char *xAttr = stmt->getAttribute("x");
        const char *yAttr = stmt->getAttribute("y");
        const char *tAttr = stmt->getAttribute("t");

        if (xAttr)
            targetPosition.x += getValue(xAttr);

        if (yAttr)
            targetPosition.y += getValue(yAttr);

        // travel to targetPosition at current speed, or get there in time t (ignoring current speed then)
        double t = tAttr ? getValue(tAttr) : lastPosition.distance(targetPosition) / speed;

        if (t < 0)
            throw cRuntimeError("<wait>: time (attribute t) is negative at %s",
                    stmt->getSourceLocation());

        nextChange += t;
    }
}

double UavMobility::getValue(const char *s)
{
    // first, textually replace $MAXX and $MAXY with their actual values
    std::string str;
    if (strchr(s, '$')) {
        char strMinX[32], strMinY[32];
        char strMaxX[32], strMaxY[32];
        sprintf(strMinX, "%g", constraintAreaMin.x);
        sprintf(strMinY, "%g", constraintAreaMin.y);
        sprintf(strMaxX, "%g", constraintAreaMax.x);
        sprintf(strMaxY, "%g", constraintAreaMax.y);

        str = s;
        std::string::size_type pos;

        while ((pos = str.find("$MINX")) != std::string::npos)
            str.replace(pos, sizeof("$MINX") - 1, strMinX);

        while ((pos = str.find("$MINY")) != std::string::npos)
            str.replace(pos, sizeof("$MINY") - 1, strMinY);

        while ((pos = str.find("$MAXX")) != std::string::npos)
            str.replace(pos, sizeof("$MAXX") - 1, strMaxX);

        while ((pos = str.find("$MAXY")) != std::string::npos)
            str.replace(pos, sizeof("$MAXY") - 1, strMaxY);

        s = str.c_str();
    }

    // then use cDynamicExpression to evaluate the string
    try {
        cDynamicExpression expr;
        expr.parse(s);
        return expr.doubleValue(this);
    }
    catch (std::exception& e) {
        throw cRuntimeError("Wrong value '%s' around %s: %s", s,
                nextStatement->getSourceLocation(), e.what());
    }
}

void UavMobility::gotoNextStatement()
{
    // "statement either doesn't have a child, or it's a <repeat> and loop count is already pushed on the stack"
    ASSERT(!nextStatement->getFirstChild() || (!strcmp(nextStatement->getTagName(), "repeat")
                                               && !loopVars.empty()));

    if (nextStatement->getFirstChild() && (loopVars.top() != 0 || (loopVars.pop(), false))) {    // !=0: positive or -1
        // statement must be a <repeat> if it has children; repeat count>0 must be
        // on the stack; let's start doing the body.
        nextStatement = nextStatement->getFirstChild();
    }
    else if (!nextStatement->getNextSibling()) {
        // no sibling -- either end of <repeat> body, or end of script
        ASSERT(nextStatement->getParentNode() == uavScript ? loopVars.empty() : !loopVars.empty());

        if (!loopVars.empty()) {
            // decrement and check loop counter
            if (loopVars.top() != -1) // -1 means infinity
                loopVars.top()--;

            if (loopVars.top() != 0) {    // positive or -1
                // go to beginning of <repeat> block again
                nextStatement = nextStatement->getParentNode()->getFirstChild();
            }
            else {
                // end of loop -- locate next statement after the <repeat>
                nextStatement = nextStatement->getParentNode();
                gotoNextStatement();
            }
        }
        else {
            // end of script
            nextStatement = nullptr;
        }
    }
    else {
        // go to next statement (must exist -- see "if" above)
        nextStatement = nextStatement->getNextSibling();
    }
}

void UavMobility::computeMaxSpeed(cXMLElement *nodes)
{
    // Recursively traverse the whole config file, looking for
    // speed attributes
    cXMLElementList childs = nodes->getChildren();
    for (auto & child : childs)
    {
        const char *speedAttr = child->getAttribute("speed");
        if (speedAttr)
        {
            double speed = atof(speedAttr);
            if (speed > maxSpeed)
                maxSpeed = speed;
        }
        computeMaxSpeed(child);
    }
}

Coord UavMobility::getTargetPosition(){
    Enter_Method("getTargetPosition()");
    return targetPosition;
}

Coord UavMobility::leaderPosition(){
    cModule *targetModule = getSimulation()->getModule(79);
    UAVUdpSink *target = check_and_cast<UAVUdpSink *>(targetModule);
    Coord position =  target->getLeaderPosition();
    return position;
}
} // namespace inet





