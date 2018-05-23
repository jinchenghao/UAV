/*
 * UAVUdpSink.cc
 *
 *  Created on: 2018Äê5ÔÂ16ÈÕ
 *      Author: Jch
 */

#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/applications/udpapp/uav/UAVUdpSink.h"

#include "inet/common/ModuleAccess.h"
#include "inet/transportlayer/contract/udp/UDPControlInfo_m.h"
#include "inet/applications/udpapp/uav/packet/UavPacket_m.h"

namespace inet {

Define_Module(UAVUdpSink);

simsignal_t UAVUdpSink::rcvdPkSignal = registerSignal("rcvdPk");

UAVUdpSink::~UAVUdpSink()
{
    cancelAndDelete(selfMsg);
}

void UAVUdpSink::initialize(int stage)
{
    ApplicationBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        numReceived = 0;
        WATCH(numReceived);

        localPort = par("localPort");
        startTime = par("startTime").doubleValue();
        stopTime = par("stopTime").doubleValue();
        if (stopTime >= SIMTIME_ZERO && stopTime < startTime)
            throw cRuntimeError("Invalid startTime/stopTime parameters");
        selfMsg = new cMessage("UAVUdpSinkTimer");
    }
}

void UAVUdpSink::handleMessageWhenUp(cMessage *msg)
{
    if (msg->isSelfMessage()) {
        ASSERT(msg == selfMsg);
        switch (selfMsg->getKind()) {
            case START:
                processStart();
                break;

            case STOP:
                processStop();
                break;

            default:
                throw cRuntimeError("Invalid kind %d in self message", (int)selfMsg->getKind());
        }
    }
    else if (msg->getKind() == UDP_I_DATA) {
        // process incoming packet
        processPacket(PK(msg));
    }
    else if (msg->getKind() == UDP_I_ERROR) {
        EV_WARN << "Ignoring UDP error report\n";
        delete msg;
    }
    else {
        throw cRuntimeError("Unrecognized message (%s)%s", msg->getClassName(), msg->getName());
    }
}

void UAVUdpSink::refreshDisplay() const
{
    char buf[50];
    sprintf(buf, "rcvd: %d pks", numReceived);
    getDisplayString().setTagArg("t", 0, buf);
}

void UAVUdpSink::finish()
{
    ApplicationBase::finish();
    EV_INFO << getFullPath() << ": received " << numReceived << " packets\n";
}

void UAVUdpSink::setSocketOptions()
{
    bool receiveBroadcast = par("receiveBroadcast");
    if (receiveBroadcast)
        socket.setBroadcast(true);

    MulticastGroupList mgl = getModuleFromPar<IInterfaceTable>(par("interfaceTableModule"), this)->collectMulticastGroups();
    socket.joinLocalMulticastGroups(mgl);

    // join multicastGroup
    const char *groupAddr = par("multicastGroup");
    multicastGroup = L3AddressResolver().resolve(groupAddr);
    if (!multicastGroup.isUnspecified()) {
        if (!multicastGroup.isMulticast())
            throw cRuntimeError("Wrong multicastGroup setting: not a multicast address: %s", groupAddr);
        socket.joinMulticastGroup(multicastGroup);
    }
}

void UAVUdpSink::processStart()
{
    socket.setOutputGate(gate("udpOut"));
    socket.bind(localPort);
    setSocketOptions();

    if (stopTime >= SIMTIME_ZERO) {
        selfMsg->setKind(STOP);
        scheduleAt(stopTime, selfMsg);
    }
}

void UAVUdpSink::processStop()
{
    if (!multicastGroup.isUnspecified())
        socket.leaveMulticastGroup(multicastGroup); // FIXME should be done by socket.close()
    socket.close();
}

void UAVUdpSink::processPacket(cPacket *pk)
{
    UavPacket *recv_payload = (UavPacket*)pk;
    position.x =  recv_payload->getX();
    position.y = recv_payload->getY();
    EV_INFO << "Received UAVUdp_packet: " << UDPSocket::getReceivedPacketInfo(pk) << recv_payload->getNameJ() << endl;
    EV_INFO << "UAV_leader position is : (" <<position.x <<","<<position.y <<")"<<"id:"<<getId()<< endl;
    emit(rcvdPkSignal, pk);
    delete pk;

    numReceived++;
}

bool UAVUdpSink::handleNodeStart(IDoneCallback *doneCallback)
{
    simtime_t start = std::max(startTime, simTime());
    if ((stopTime < SIMTIME_ZERO) || (start < stopTime) || (start == stopTime && startTime == stopTime)) {
        selfMsg->setKind(START);
        scheduleAt(start, selfMsg);
    }
    return true;
}

bool UAVUdpSink::handleNodeShutdown(IDoneCallback *doneCallback)
{
    if (selfMsg)
        cancelEvent(selfMsg);
    //TODO if(socket.isOpened()) socket.close();
    return true;
}

void UAVUdpSink::handleNodeCrash()
{
    if (selfMsg)
        cancelEvent(selfMsg);
}

Coord UAVUdpSink::getLeaderPosition(){
    Enter_Method("getLeaderPosition()");
    return position;
}

} // namespace inet



