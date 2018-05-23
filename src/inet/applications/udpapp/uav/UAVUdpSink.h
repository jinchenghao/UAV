/*
 * UAVUdpSink.h
 *
 *  Created on: 2018年5月16日
 *      Author: Jch
 */

#ifndef INET_APPLICATIONS_UDPAPP_UAV_UAVUDPSINK_H_
#define INET_APPLICATIONS_UDPAPP_UAV_UAVUDPSINK_H_

#include "inet/common/INETDefs.h"

#include "inet/applications/base/ApplicationBase.h"
#include "inet/transportlayer/contract/udp/UDPSocket.h"
#include "inet/applications/base/ApplicationPacket_m.h"
#include "inet/common/geometry/common/Coord.h"

namespace inet {

/**
 * Consumes and prints packets received from the UDP module. See NED for more info.
 */
class INET_API UAVUdpSink : public ApplicationBase
{
  protected:
    enum SelfMsgKinds { START = 1, STOP };

    UDPSocket socket;
    int localPort = -1;
    L3Address multicastGroup;
    simtime_t startTime;
    simtime_t stopTime;
    cMessage *selfMsg = nullptr;

    //接收到的leader坐标
    Coord position;

    int numReceived = 0;
    static simsignal_t rcvdPkSignal;

  public:
    UAVUdpSink() {}
    virtual ~UAVUdpSink();
    /** @brief 为mobility模块获取leader(通过UDP接收到的)的位置提供的函数*/
    virtual Coord getLeaderPosition();

  protected:
    virtual void processPacket(cPacket *msg);
    virtual void setSocketOptions();

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(cMessage *msg) override;
    virtual void finish() override;
    virtual void refreshDisplay() const override;

    virtual void processStart();
    virtual void processStop();

    virtual bool handleNodeStart(IDoneCallback *doneCallback) override;
    virtual bool handleNodeShutdown(IDoneCallback *doneCallback) override;
    virtual void handleNodeCrash() override;
};

} // namespace inet



#endif /* INET_APPLICATIONS_UDPAPP_UAV_UAVUAVUdpSink_H_ */
