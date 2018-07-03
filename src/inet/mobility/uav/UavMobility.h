/*
 * UavMobility.h
 *
 *  Created on: 2018年5月14日
 *      Author: Jch
 */

#ifndef INET_MOBILITY_UAV_UAVMOBILITY_H_
#define INET_MOBILITY_UAV_UAVMOBILITY_H_

#include <stack>

#include "inet/common/INETDefs.h"

#include "inet/mobility/base/LineSegmentsMobilityBase.h"
#include <omnetpp.h>

namespace inet {

/**
 * @brief LOGO-style movement model, with the script coming from XML.
 * See NED file for more info.
 *
 * @ingroup mobility
 * @author Andras Varga
 */
class INET_API UavMobility : public LineSegmentsMobilityBase
{
  protected:
    // config
    cXMLElement *uavScript;

    // state
    cXMLElement *nextStatement;
    double speed;
    double angle;
    BorderPolicy borderPolicy;
    std::stack<long> loopVars;    // for <repeat>
    double maxSpeed;
    //是否是leader
    bool leader;
    //局部坐标系坐标系的位置
    double Ref_x;
    double Ref_y;
    //飞行的坐标次数
    int count = 0;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }


    /** @brief Initializes mobility model parameters.*/
    virtual void initialize(int stage) override;

    /** @brief Initializes the position according to the mobility model. */
    virtual void setInitialPosition() override;

    /** @brief Overridden from LineSegmentsMobilityBase. Invokes resumeScript().*/
    virtual void setTargetPosition() override;

    /** @brief Overridden from LineSegmentsMobilityBase.*/
    virtual void move() override;

    /** @brief Process next statements from script */
    virtual void resumeScript();

    /** @brief Execute the given statement*/
    virtual void executeStatement(cXMLElement *nextStatement);

    /** @brief Parse attrs in the script -- accepts things like "uniform(10,50) as well */
    virtual double getValue(const char *s);

    /** @brief Advance nextStatement pointer */
    virtual void gotoNextStatement();

    // XXX: In turtleScript xml config files, speed attributes may contain expressions (like uniform(10,30)),
    // in this case, we can't compute the maxSpeed
    virtual void computeMaxSpeed(cXMLElement *nodes);

    /** @brief 得到Leader的位置 */
    virtual Coord leaderPosition();

    /** 局部坐标系到全局坐标系转换*/
    virtual void convertCoordinate();

    /** @brief 队形调整*/
    virtual void formationChange();

  public:
    UavMobility();
    virtual double getMaxSpeed() const override { return maxSpeed; }
    /** @brief 为其他模块获取当前位置提供的函数*/
    virtual Coord getTargetPosition();
};

} // namespace inet



#endif /* INET_MOBILITY_UAV_UAVMOBILITY_H_ */
