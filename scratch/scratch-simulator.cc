/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LrWpanMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <ns3/buildings-module.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>


#include <deque>
#include <algorithm>

using namespace ns3;

// CLASS SPACE =================================================================
class PacketStructure {
public:
  uint32_t SRN;       // 4 bytes: set of path field
  uint8_t direction;  // 1 byte : direction
  uint8_t seq;        // 1 byte : sequence
  double payload;     // 8 bytes: payload

  uint16_t GetDsn () {
    return (static_cast<uint16_t>(direction) *1000) + static_cast<uint16_t>(seq);
  }
};

class DeviceStructure {
public:
  int node_role = 0;    // Default : RELAY_NODE_ROLE

  double reference = 0;

  double U = 0;
  double Y = 0;

  double X[3]   = {0,};
  double X_t[3] = {0,};

  double error_sum  = 0;
  double error_last = 0;

  std::deque<uint16_t> DSN_Queue;

  uint32_t SRN = 0;
  uint8_t seq  = 0;

  bool rxTrigger = false;
};
// END CLASS SPACE =============================================================

// CONTSTANT ===================================================================
const int PLANT_SIZE             = 1;
const int NODE_SIZE              = 23;
const int CONTROLLER_ROLE        = 2;
const int PLANT_ROLE             = 1;
const int RELAY_NODE_ROLE        = 0;
const int CONTROLLER_DIRECTION   = 0;
const int PLANT_DIRECTION        = 1;
// END CONTSTANT ===============================================================

Ptr<Node> nodes[NODE_SIZE];
Ptr<LrWpanNetDevice> devices[NODE_SIZE];
Ptr<MobilityModel> mobilities[NODE_SIZE];
Ptr<MobilityBuildingInfo> buildingInfos[NODE_SIZE];

DeviceStructure _devices[NODE_SIZE];

double Kp = 0;
double Ki = 0;
double Kd = 0;

double _A[9] = {0,};
double _B[3] = {0,};
double _C[3] = {0,};

McpsDataRequestParams txParams;
// std::deque<uint16_t> DSN_Queue[NODE_SIZE];

uint32_t TriggerNode(int node_number) {
  return 1 << (node_number);
}

static void dequeue_handler (int idx) {
  _devices[idx].DSN_Queue.pop_front();
  // std::cout << "[" << Simulator::Now ().GetSeconds () << "] dequeue device " << idx << std::endl;
  // std::cout << "dequeue_handler: " << Simulator::Now ().GetSeconds () << " " << _devices[idx].DSN_Queue.size() << std::endl;
}

void TxPacket (int devIdx, uint32_t SRN, uint8_t direction, uint8_t seq, double payload) {
  PacketStructure pkt_form = {SRN, direction, seq, payload};
  uint8_t* serialized_pkt = static_cast<uint8_t*>(static_cast<void*>(&pkt_form));
  Ptr<Packet> pkt = Create<Packet> (serialized_pkt, sizeof(pkt_form));

  devices[devIdx]->GetMac ()->McpsDataRequest (txParams, pkt);  // tx
  _devices[devIdx].DSN_Queue.push_back(pkt_form.GetDsn());  // enqueue
  // std::cout << "[" << Simulator::Now ().GetSeconds () << "] enqueue device " << devIdx << std::endl;
  Simulator::Schedule(Seconds(1), &dequeue_handler, devIdx); // dequeue
}

static void RelayDeviceRxCallback (McpsDataIndicationParams rxParams, Ptr<Packet> p)
{
  // NS_LOG_UNCOND ("Received from " << rxParams.m_srcAddr << " to " << rxParams.m_dstAddr << " packet of size " << p->GetSize ());

  // deserialize the received packet: convert <Packet> to <PacketStructure> form
  uint8_t* serialized_pkt = new uint8_t[p->GetSize ()];
  p->CopyData (serialized_pkt, p->GetSize ());
  PacketStructure pkt = *reinterpret_cast<PacketStructure *>(serialized_pkt);
  // std::cout << pkt.SRN << std::endl;
  // std::cout << static_cast<int>(pkt.direction) << std::endl;
  // std::cout << static_cast<int>(pkt.seq) << std::endl;
  // std::cout << pkt.payload << std::endl;

  // Get my index
  uint8_t myAddr[2];  // if 00:01, [0] = 00 and [1] = 01
  rxParams.m_dstAddr.CopyTo (myAddr);
  int myIdx = myAddr[1];

  uint32_t pick = 1;
  uint16_t dsn = pkt.GetDsn();
  if (((pkt.SRN & (pick << myIdx)) == (pick << myIdx)) &&     // I'm a relay node of this packet
      (std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) == _devices[myIdx].DSN_Queue.end())) { // doesn't eixst the DSN
    devices[myIdx]->GetMac ()->McpsDataRequest (txParams, p); // tx
    _devices[myIdx].DSN_Queue.push_back(dsn);  // enqueue
    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] enqueue device " << myIdx << std::endl;
    Simulator::Schedule(Seconds(1), &dequeue_handler, myIdx); // dequeue
    // std::cout<<static_cast<int>(_devices[myIdx].DSN_Queue.front())<<std::endl;
    // std::cout << "hi" << static_cast<int>(myIdx) << std::endl;
    // std::cout << "before: " << Simulator::Now ().GetSeconds () << std::endl;

    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << myIdx << " relay " << rxParams.m_srcAddr << " -> " << rxParams.m_dstAddr << " -> broadcast " << std::endl;
  } else {
    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << myIdx << " discard " << rxParams.m_srcAddr << " -> " << rxParams.m_dstAddr << " ";
    // if (((pkt.SRN & (pick << myIdx)) != (pick << myIdx))) {
    //   std::cout << "[NOT MINE]" << std::endl;
    // } else if ((std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) != _devices[myIdx].DSN_Queue.end())) {
    //   std::cout << "[DSN]" << std::endl;
    // }
  }
}

static void ControllerRxCallback (McpsDataIndicationParams rxParams, Ptr<Packet> p)
{
  // deserialize the received packet: convert <Packet> to <PacketStructure> form
  uint8_t* serialized_pkt = new uint8_t[p->GetSize ()];
  p->CopyData (serialized_pkt, p->GetSize ());
  PacketStructure pkt = *reinterpret_cast<PacketStructure *>(serialized_pkt);

  // Get my index
  uint8_t myAddr[2];  // if 00:01, [0] = 00 and [1] = 01
  rxParams.m_dstAddr.CopyTo (myAddr);
  int myIdx = myAddr[1];

  uint32_t pick = 1;
  uint16_t dsn = pkt.GetDsn();
  if (((pkt.SRN & (pick << myIdx)) == (pick << myIdx)) &&     // I'm a relay node of this packet
      (std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) == _devices[myIdx].DSN_Queue.end())) { // doesn't eixst the DSN
    _devices[myIdx].Y = pkt.payload;
    _devices[myIdx].DSN_Queue.push_back(dsn);  // enqueue
    Simulator::Schedule(Seconds(1), &dequeue_handler, myIdx); // dequeue

    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] Rx Controller ["  << myIdx << "]: " << pkt.payload << std::endl;
    std::cout << pkt.payload << std::endl;
  } else {
    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << myIdx << " discard " << rxParams.m_srcAddr << " -> " << rxParams.m_dstAddr << " ";
    // if (((pkt.SRN & (pick << myIdx)) != (pick << myIdx))) {
    //   std::cout << "[NOT MINE]" << std::endl;
    // } else if ((std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) != _devices[myIdx].DSN_Queue.end())) {
    //   std::cout << "[DSN]" << std::endl;
    // }
  }
}

void ControllerTxCallback (int cycle, double interval, int myIdx) {
  if (cycle < 0) {
    return;
  }

  // Calculate plant
  double error = _devices[myIdx].reference - _devices[myIdx].Y;
  double U = Kp * error + Ki * _devices[myIdx].error_sum + Kd * ((error - _devices[myIdx].error_last) / 0.2);
  _devices[myIdx].error_sum += error * 0.2;
  _devices[myIdx].error_last = error;

  // uint32_t path = TriggerNode(0) +
  //                  TriggerNode(3) +
  //                  TriggerNode(8) +
  //                  TriggerNode(14) +
  //                  TriggerNode(16) +
  //                  TriggerNode(21) +
  //                  TriggerNode(22) +
  //                  TriggerNode(1);

  uint32_t path = TriggerNode(0) +
                   TriggerNode(3) +
                   TriggerNode(9) +
                   TriggerNode(15) +
                   TriggerNode(6) +
                   TriggerNode(17) +
                   TriggerNode(20) +
                   TriggerNode(1);

  // uint32_t path = TriggerNode(0) +
  //                  TriggerNode(4) +
  //                  TriggerNode(2) +
  //                  TriggerNode(10) +
  //                  TriggerNode(12) +
  //                  TriggerNode(16) +
  //                  TriggerNode(17) +
  //                  TriggerNode(20) +
  //                  TriggerNode(1);

  // uint32_t path = TriggerNode(0) +
  //                  TriggerNode(5) +
  //                  TriggerNode(11) +
  //                  TriggerNode(13) +
  //                  TriggerNode(19) +
  //                  TriggerNode(21) +
  //                  TriggerNode(22) +
  //                  TriggerNode(1);

  // uint32_t path = TriggerNode(0) +
  //                  TriggerNode(6) +
  //                  TriggerNode(7) +
  //                  TriggerNode(18) +
  //                  TriggerNode(1);

  // Tx a packet
  _devices[myIdx].SRN = path;
  uint8_t direction = PLANT_DIRECTION;
  _devices[myIdx].seq++;

  TxPacket(0, _devices[myIdx].SRN, direction, _devices[myIdx].seq, U);

  // std::cout << "Controller TX[" << Simulator::Now ().GetSeconds () << "]" << cycle << " times /// " << U << " " << _devices[myIdx].Y << std::endl;
  Simulator::Schedule(Seconds(interval), &ControllerTxCallback, cycle-1, interval, myIdx);
}

static void PlantRxCallback (McpsDataIndicationParams rxParams, Ptr<Packet> p)
{
  // deserialize the received packet: convert <Packet> to <PacketStructure> form
  uint8_t* serialized_pkt = new uint8_t[p->GetSize ()];
  p->CopyData (serialized_pkt, p->GetSize ());
  PacketStructure pkt = *reinterpret_cast<PacketStructure *>(serialized_pkt);

  // Get my index
  uint8_t myAddr[2];  // if 00:01, [0] = 00 and [1] = 01
  rxParams.m_dstAddr.CopyTo (myAddr);
  int myIdx = myAddr[1];

  uint32_t pick = 1;
  uint16_t dsn = pkt.GetDsn();
  if (((pkt.SRN & (pick << myIdx)) == (pick << myIdx)) &&     // I'm a relay node of this packet
      (std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) == _devices[myIdx].DSN_Queue.end())) { // doesn't eixst the DSN
    _devices[myIdx].U = pkt.payload;
    _devices[myIdx].DSN_Queue.push_back(dsn);  // enqueue
    Simulator::Schedule(Seconds(1), &dequeue_handler, myIdx); // dequeue
    _devices[myIdx].SRN = pkt.SRN;
    _devices[myIdx].seq = pkt.seq;
    _devices[myIdx].rxTrigger = true;

    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] Rx Plant ["  << myIdx << "]: " << pkt.payload << std::endl;
  } else {
    // std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << myIdx << " discard " << rxParams.m_srcAddr << " -> " << rxParams.m_dstAddr << " ";
    // if (((pkt.SRN & (pick << myIdx)) != (pick << myIdx))) {
    //   std::cout << "[NOT MINE]" << std::endl;
    // } else if ((std::find(_devices[myIdx].DSN_Queue.begin(), _devices[myIdx].DSN_Queue.end(), dsn) != _devices[myIdx].DSN_Queue.end())) {
    //   std::cout << "[DSN]" << std::endl;
    // }
  }
}

void PlantTxCallback (int cycle, double interval, int myIdx) {
  if (cycle < 0) {
    return;
  }

  // Calculate plant
  double Y = _C[0] * _devices[myIdx].X[0] + _C[1] * _devices[myIdx].X[1] + _C[2] * _devices[myIdx].X[2];

  _devices[myIdx].X_t[0] = _A[0] * _devices[myIdx].X[0] + _A[1] * _devices[myIdx].X[1] + _A[2] * _devices[myIdx].X[2] + _B[0] * _devices[myIdx].U;
  _devices[myIdx].X_t[1] = _A[3] * _devices[myIdx].X[0] + _A[4] * _devices[myIdx].X[1] + _A[5] * _devices[myIdx].X[2] + _B[1] * _devices[myIdx].U;
  _devices[myIdx].X_t[2] = _A[6] * _devices[myIdx].X[0] + _A[7] * _devices[myIdx].X[1] + _A[8] * _devices[myIdx].X[2] + _B[2] * _devices[myIdx].U;

  _devices[myIdx].X[0] = _devices[myIdx].X_t[0];
  _devices[myIdx].X[1] = _devices[myIdx].X_t[1];
  _devices[myIdx].X[2] = _devices[myIdx].X_t[2];

  // Tx a packet
  if (_devices[myIdx].rxTrigger == false) {
    _devices[myIdx].seq++;
  }
  TxPacket(myIdx, _devices[myIdx].SRN, CONTROLLER_DIRECTION, _devices[myIdx].seq, Y);
  _devices[myIdx].rxTrigger = false;

  // std::cout << _devices[myIdx].X[0] << "\t" << _devices[myIdx].X[1] << "\t" << _devices[myIdx].X[2] << std::endl;
  // std::cout << _devices[myIdx].X_t[0] << "\t" << _devices[myIdx].X_t[1] << "\t" << _devices[myIdx].X_t[2] << std::endl;
  // std::cout << "Plant TX [" << Simulator::Now ().GetSeconds () << "]" << cycle << " times /// " << Y << " " << _devices[myIdx].U << std::endl;
  Simulator::Schedule(Seconds(interval), &PlantTxCallback, cycle-1, interval, myIdx);
}

// static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
// {
//   NS_LOG_UNCOND (context << " state change at " << now.GetSeconds ()
//                          << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
//                          << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState));
// }

int main (int argc, char *argv[])
{
  // PI controller
  Kp = 0.060826;
  Ki = 0.030286;
  Kd = 0;

  // DC Motor Position Control
  _A[0] = 1; _A[1] = 0.0168850192401696;    _A[2] = 9.85229679853156e-05;
  _A[3] = 0; _A[4] = 7.17313525970281e-06;  _A[5] = 4.18564730646073e-08;
  _A[6] = 0; _A[7] = -4.91379773242829e-08; _A[8] = -2.86728515475999e-10;

  _B[0] = 6.56042165678637;
  _B[1] = 35.8265338128420;
  _B[2] = 0.00458824345371420;

  _C[0] = 1; _C[1] = 0; _C[2] = 0;

  // give the node to role
  _devices[0].node_role = CONTROLLER_ROLE;
  _devices[1].node_role = PLANT_ROLE;

  // initiating pacekt params
  txParams.m_dstPanId = 0;
  txParams.m_srcAddrMode = SHORT_ADDR;
  txParams.m_dstAddrMode = SHORT_ADDR;
  txParams.m_dstAddr = Mac16Address ("ff:ff");
  txParams.m_msduHandle = 0;
  txParams.m_txOptions = TX_OPTION_NONE;  // TX_OPTION_NONE, TX_OPTION_ACK, TX_OPTION_GTS, TX_OPTION_INDIRECT

  // Building(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
  Ptr<Building> building1 = CreateObject<Building> ();
  building1->SetBuildingType (Building::Residential);
  building1->SetExtWallsType (Building::ConcreteWithWindows);
  building1->SetBoundaries (Box (-20, 60, 0, 30, 0, 0));
  // building1->SetNRoomsX (16);
  // building1->SetNRoomsY (5);
  building1->SetNRoomsX (40);
  building1->SetNRoomsY (15);

  // Create nodes, and a NetDevice for each one
  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  // Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();

  // Ptr<RandomPropagationLossModel> propModel = CreateObject<RandomPropagationLossModel> ();
  // Ptr<ExponentialRandomVariable> expVar = CreateObjectWithAttributes<ExponentialRandomVariable> ("Mean", DoubleValue (5.0));
  // propModel->SetAttribute ("Variable", PointerValue (expVar));

  Ptr<HybridBuildingsPropagationLossModel> propModel = CreateObject<HybridBuildingsPropagationLossModel> ();

  // Ptr<FriisPropagationLossModel> propModel = CreateObject<FriisPropagationLossModel> ();

  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  double position[NODE_SIZE][3] = {
    {27,2,0},
    {44,27,0},
    {26,5,0},
    {19,8,0},
    {23,8,0},
    {29,8,0},
    {37,8,0},
    {48,8,0},
    {10,10,0},
    {20,11,0},
    {25,11,0},
    {30,11,0},
    {26,18,0},
    {31,18,0},
    {14,19,0},
    {22,20,0},
    {29,20,0},
    {37.5,19,0},
    {48,19,0},
    {31,22,0},
    {42,22,0},
    {34,25,0},
    {38,26,0},
  };

  for (int i = 0; i < NODE_SIZE; i++) {
    // init
    nodes[i] = CreateObject <Node> ();
    devices[i] = CreateObject<LrWpanNetDevice> ();
    mobilities[i] = CreateObject<ConstantPositionMobilityModel> ();
    buildingInfos[i] = CreateObject<MobilityBuildingInfo> ();

    // assign MAC
    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(4) << std::hex << i;
    std::string result = ss.str().substr(0, 2) + ":" + ss.str().substr(2, 2);

    devices[i]->SetAddress (Mac16Address (const_cast<char*>(result.c_str())));

    // assign channel
    devices[i]->SetChannel (channel);

    // To complete configuration, a LrWpanNetDevice must be added to a node
    nodes[i]->AddDevice (devices[i]);

    // std::cout << devices[i]->GetMac ()->GetShortAddress() << std::endl;
    // std::cout << devices[i]->GetAddress () << std::endl;
    // std::cout << devices[i]->GetBroadcast () << std::endl;
    // std::cout << Mac16Address (const_cast<char*>(result.c_str())) << std::endl;

    // configure position (m)
    mobilities[i]->SetPosition (Vector (position[i][0], position[i][1], position[i][2]));  // x,y,z
    mobilities[i]->AggregateObject (buildingInfos[i]);
    BuildingsHelper::MakeConsistent (mobilities[i]);
    devices[i]->GetPhy ()->SetMobility (mobilities[i]);

    // std::cout << mobilities[i]->GetPosition() << std::endl;

    if (_devices[i].node_role == CONTROLLER_ROLE) {
      devices[i]->GetMac ()->SetMcpsDataIndicationCallback (MakeCallback (&ControllerRxCallback));
      _devices[i].reference = 10;
      Simulator::Schedule(Seconds(0), &ControllerTxCallback, 100, 0.2, i);
      // std::cout << "control " << i << std::endl;

    } else if (_devices[i].node_role == PLANT_ROLE) {
      devices[i]->GetMac ()->SetMcpsDataIndicationCallback (MakeCallback (&PlantRxCallback));
      Simulator::Schedule(Seconds(0.1), &PlantTxCallback, 100, 0.2, i);
      // std::cout << "plant " << i << std::endl;

    } else {
      devices[i]->GetMac ()->SetMcpsDataIndicationCallback (MakeCallback (&RelayDeviceRxCallback));
      // std::cout << "relay " << i << std::endl;
    }
  }














  // Trace state changes in the phy
  // devices[0]->GetPhy ()->TraceConnect ("TrxState", std::string ("phy0"), MakeCallback (&StateChangeNotification));
  // devices[1]->GetPhy ()->TraceConnect ("TrxState", std::string ("phy1"), MakeCallback (&StateChangeNotification));
  // devices[15]->GetPhy ()->TraceConnect ("TrxState", std::string ("phy15"), MakeCallback (&StateChangeNotification));
  // devices[16]->GetPhy ()->TraceConnect ("TrxState", std::string ("phy16"), MakeCallback (&StateChangeNotification));





  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50




  // uint32_t SRN = TriggerNode(0);
  // uint8_t direction = PLANT_DIRECTION;
  // uint8_t seq = 255;
  // double payload = -25.86;
  //
  // TxPacket(0, SRN, direction, seq, payload);

  // Simulator::Schedule(Seconds(1), &TxPacket, 0, SRN, direction, seq, payload);
  // seq++;
  // Simulator::Schedule(Seconds(1.005), &TxPacket, 0, SRN, direction, seq, payload);
  //
  // Simulator::Schedule(Seconds(0), &SimInterval, 100, 0.2);




  // all packets with the same destination can not be arrived at the destination simultaneously, where the device status is [RX_ON] or [RX_BUSY].

  // seq = 0;
  // pkt = {SRN, direction, seq, payload};
  // serialized_pkt = static_cast<uint8_t*>(static_cast<void*>(&pkt));
  // p0 = Create<Packet> (serialized_pkt, sizeof(pkt));
  // devices[0]->GetMac ()->McpsDataRequest (txParams, p0);
  // DSN_Queue[0].push_back(pkt.GetDsn());  // enqueue
  // Simulator::Schedule(Seconds(Simulator::Now ().GetSeconds () + 1), &dequeue_handler, 0); // dequeue


  // Simulator::ScheduleWithContext (1, Seconds (0.0),
  //                                 &LrWpanMac::McpsDataRequest,
  //                                 devices[0]->GetMac (), params, p0);

  // Send a packet back at time 2 seconds
  // Ptr<Packet> p2 = Create<Packet> (60);  // 60 bytes of dummy data
  // if (!extended)
  //   {
  //     params.m_dstAddr = Mac16Address ("00:01");
  //   }
  // else
  //   {
  //     params.m_dstExtAddr = Mac64Address ("00:00:00:00:00:00:00:01");
  //   }
  // Simulator::ScheduleWithContext (2, Seconds (2.0),
  //                                 &LrWpanMac::McpsDataRequest,
  //                                 devices[1]->GetMac (), params, p2);

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}
