/*
 * IEEE 802.11p DTD component for a VANET simulation.
 *
 * This is a cleaned public-release version of legacy code developed for
 * ns-3.26. The original topology, mobility settings, traffic rate, packet
 * size, and per-vehicle transmission windows are preserved.
 *
 * Scope: this file models the IEEE 802.11p / delay-tolerant-data component.
 * It is not a complete implementation of the integrated IQDN SDN controller.
 */

#include <iostream>
#include <stdint.h>
#include <vector>

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Ieee80211pDtdComponent");

namespace
{

const double kThroughputIntervalSeconds = 0.1;
std::vector<Ptr<PacketSink> > g_packetSinks;
uint64_t g_lastTotalRxBytes = 0;

uint64_t
GetTotalReceivedBytes ()
{
  uint64_t totalRxBytes = 0;
  for (std::vector<Ptr<PacketSink> >::const_iterator it = g_packetSinks.begin ();
       it != g_packetSinks.end ();
       ++it)
    {
      totalRxBytes += (*it)->GetTotalRx ();
    }
  return totalRxBytes;
}

void
CalculateAggregateThroughput ()
{
  const uint64_t totalRxBytes = GetTotalReceivedBytes ();
  const uint64_t intervalRxBytes = totalRxBytes - g_lastTotalRxBytes;
  const double throughputKbps =
      intervalRxBytes * 8.0 / (kThroughputIntervalSeconds * 1000.0);

  std::cout << Simulator::Now ().GetSeconds () << "\t" << throughputKbps
            << std::endl;

  g_lastTotalRxBytes = totalRxBytes;
  Simulator::Schedule (Seconds (kThroughputIntervalSeconds),
                       &CalculateAggregateThroughput);
}

void
PrintFlowStatistics (Ptr<FlowMonitor> flowMonitor,
                     FlowMonitorHelper &flowMonitorHelper)
{
  flowMonitor->CheckForLostPackets ();

  Ptr<Ipv4FlowClassifier> classifier =
      DynamicCast<Ipv4FlowClassifier> (flowMonitorHelper.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats ();

  for (FlowMonitor::FlowStatsContainer::const_iterator it = stats.begin ();
       it != stats.end ();
       ++it)
    {
      const FlowMonitor::FlowStats &flow = it->second;
      if (flow.txPackets == 0 || flow.txBytes <= 10000)
        {
          continue;
        }

      const Ipv4FlowClassifier::FiveTuple tuple =
          classifier->FindFlow (it->first);
      const double packetLossPercent =
          100.0 * (flow.txPackets - flow.rxPackets) / flow.txPackets;

      std::cout << "Flow " << it->first << " (" << tuple.sourceAddress
                << " -> " << tuple.destinationAddress << ")\n"
                << "  Tx packets: " << flow.txPackets << "\n"
                << "  Rx packets: " << flow.rxPackets << "\n"
                << "  Tx bytes:   " << flow.txBytes << "\n"
                << "  Rx bytes:   " << flow.rxBytes << "\n"
                << "  Lost packets: " << flow.lostPackets << "\n"
                << "  Packet loss: " << packetLossPercent << " %\n";

      if (flow.rxPackets > 0)
        {
          const double meanDelayMs =
              flow.delaySum.GetSeconds () * 1000.0 / flow.rxPackets;
          std::cout << "  Mean delay: " << meanDelayMs << " ms\n";
        }
      else
        {
          std::cout << "  Mean delay: n/a (no packets received)\n";
        }

      if (flow.rxPackets > 1)
        {
          const double meanJitterMs =
              flow.jitterSum.GetSeconds () * 1000.0 / (flow.rxPackets - 1);
          std::cout << "  Mean jitter: " << meanJitterMs << " ms\n";
        }
      else
        {
          std::cout << "  Mean jitter: n/a\n";
        }

      const double activeDuration =
          (flow.timeLastRxPacket - flow.timeFirstTxPacket).GetSeconds ();
      if (activeDuration > 0.0)
        {
          const double throughputMbps =
              flow.rxBytes * 8.0 / (activeDuration * 1000.0 * 1000.0);
          std::cout << "  Throughput: " << throughputMbps << " Mbps\n";
        }
      else
        {
          std::cout << "  Throughput: n/a\n";
        }

      std::cout << std::endl;
    }
}

} // namespace

int
main (int argc, char *argv[])
{
  // Original simulation parameters.
  uint32_t size = 40;
  uint32_t nCsma = 41;
  uint16_t packetSize = 1000;
  std::string dataRate = "1Mbps";
  double totalTime = 180.0;
  uint32_t seed = 1234;
  std::string phyMode = "OfdmRate6MbpsBW10MHz";

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of additional CSMA nodes/devices", nCsma);
  cmd.AddValue ("size", "Number of mobile IEEE 802.11p nodes", size);
  cmd.AddValue ("time", "Simulation time in seconds", totalTime);
  cmd.AddValue ("seed", "Random-number seed", seed);
  cmd.AddValue ("packetSize", "DTD packet size in bytes", packetSize);
  cmd.AddValue ("dataRate", "DTD application data rate", dataRate);
  cmd.Parse (argc, argv);

  if (size < 40)
    {
      std::cerr << "This legacy scenario requires at least 40 mobile nodes."
                << std::endl;
      return 1;
    }
  if (nCsma < 40)
    {
      std::cerr << "This legacy scenario requires at least 40 additional "
                   "CSMA nodes."
                << std::endl;
      return 1;
    }

  SeedManager::SetSeed (seed);

  // --------------------------------------------------------------------------
  // Core network: point-to-point link and CSMA destination network.
  // --------------------------------------------------------------------------
  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("100Gb/s"));
  pointToPoint.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6)));
  NetDeviceContainer p2pDevices = pointToPoint.Install (p2pNodes);

  NodeContainer csmaNodes;
  csmaNodes.Add (p2pNodes.Get (1));
  csmaNodes.Create (nCsma);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Gb/s"));
  csma.SetChannelAttribute ("Delay", TimeValue (NanoSeconds (6)));
  NetDeviceContainer csmaDevices = csma.Install (csmaNodes);

  // The first point-to-point node also acts as the IEEE 802.11p roadside node.
  NodeContainer ap = p2pNodes.Get (0);
  NodeContainer staNodes;
  staNodes.Create (size);

  NodeContainer staticNodes;
  staticNodes.Create (4);

  std::cout << "Creating 1 roadside node\n";
  std::cout << "Creating " << size << " mobile nodes\n";

  // --------------------------------------------------------------------------
  // IEEE 802.11p devices.
  // --------------------------------------------------------------------------
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  // Original transmit power used to obtain an approximately 250 m range.
  wifiPhy.Set ("TxPowerStart", DoubleValue (26.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (26.0));
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);

  NqosWaveMacHelper waveMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager (
      "ns3::ConstantRateWifiManager",
      "DataMode",
      StringValue (phyMode),
      "ControlMode",
      StringValue (phyMode));

  NetDeviceContainer staDevices =
      wifi80211p.Install (wifiPhy, waveMac, staNodes);
  NetDeviceContainer staticDevices =
      wifi80211p.Install (wifiPhy, waveMac, staticNodes);
  NetDeviceContainer apDevices = wifi80211p.Install (wifiPhy, waveMac, ap);

  // --------------------------------------------------------------------------
  // Mobility.
  // --------------------------------------------------------------------------
  MobilityHelper mobility;

  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (3200.0),
      "MinY",
      DoubleValue (210.0),
      "DeltaX",
      DoubleValue (20.0),
      "DeltaY",
      DoubleValue (5.0),
      "GridWidth",
      UintegerValue (3),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (staticNodes);

  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (3250.0),
      "MinY",
      DoubleValue (230.0),
      "DeltaX",
      DoubleValue (1000.0),
      "DeltaY",
      DoubleValue (5.0),
      "GridWidth",
      UintegerValue (10),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (ap);

  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (0.0),
      "MinY",
      DoubleValue (235.0),
      "DeltaX",
      DoubleValue (150.0),
      "DeltaY",
      DoubleValue (10.0),
      "GridWidth",
      UintegerValue (20),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel (
      "ns3::RandomWalk2dMobilityModel",
      "Bounds",
      RectangleValue (Rectangle (-100.0, 8000.0, 190.0, 300.0)),
      "Speed",
      StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
      "Direction",
      StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  mobility.Install (staNodes);

  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (1000.0),
      "MinY",
      DoubleValue (500.0),
      "DeltaX",
      DoubleValue (250.0),
      "DeltaY",
      DoubleValue (70.0),
      "GridWidth",
      UintegerValue (10),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (csmaNodes);

  // --------------------------------------------------------------------------
  // Internet stack and addressing.
  // --------------------------------------------------------------------------
  InternetStackHelper stack;
  AodvHelper aodv;
  stack.SetRoutingHelper (aodv);
  stack.Install (csmaNodes);
  stack.Install (staNodes);
  stack.Install (ap);
  stack.Install (staticNodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staInterfaces = address.Assign (staDevices);
  Ipv4InterfaceContainer apInterface = address.Assign (apDevices);
  Ipv4InterfaceContainer staticInterfaces = address.Assign (staticDevices);

  address.SetBase ("10.1.2.0", "255.255.255.0");
  Ipv4InterfaceContainer csmaInterfaces = address.Assign (csmaDevices);

  address.SetBase ("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer p2pInterfaces = address.Assign (p2pDevices);

  // Avoid unused-variable warnings while preserving the original interface
  // containers for inspection and tracing.
  (void) staInterfaces;
  (void) apInterface;
  (void) staticInterfaces;
  (void) p2pInterfaces;

  // --------------------------------------------------------------------------
  // Delay-tolerant traffic.
  //
  // The following windows are the original manually scheduled contact periods
  // for mobile nodes 20-39. The loop replaces repeated blocks without changing
  // the node mapping, destination mapping, ports, or start/stop times.
  // --------------------------------------------------------------------------
  const double startTimes[20] = {
      150.0, 143.0, 135.0, 128.0, 120.0,
      113.0, 105.0, 98.0,  90.0,  83.0,
      75.0,  68.0,  60.0,  53.0,  45.0,
      38.0,  30.0,  23.0,  15.0,  8.0};
  const double stopTimes[20] = {
      170.0, 163.0, 155.0, 148.0, 140.0,
      133.0, 125.0, 118.0, 110.0, 103.0,
      95.0,  88.0,  80.0,  73.0,  65.0,
      58.0,  50.0,  43.0,  35.0,  28.0};

  ApplicationContainer serverApps;
  ApplicationContainer clientApps;

  for (uint32_t flowIndex = 0; flowIndex < 20; ++flowIndex)
    {
      const uint32_t sourceNodeIndex = 20 + flowIndex;
      const uint32_t destinationNodeIndex = 20 + flowIndex;
      const uint16_t port = static_cast<uint16_t> (921 + flowIndex);
      const Ipv4Address destinationAddress =
          csmaInterfaces.GetAddress (destinationNodeIndex);

      PacketSinkHelper sinkHelper (
          "ns3::UdpSocketFactory",
          InetSocketAddress (Ipv4Address::GetAny (), port));
      ApplicationContainer sinkApp =
          sinkHelper.Install (csmaNodes.Get (destinationNodeIndex));
      sinkApp.Start (Seconds (0.0));
      sinkApp.Stop (Seconds (totalTime));
      serverApps.Add (sinkApp);
      g_packetSinks.push_back (StaticCast<PacketSink> (sinkApp.Get (0)));

      OnOffHelper sourceHelper (
          "ns3::UdpSocketFactory",
          InetSocketAddress (destinationAddress, port));
      sourceHelper.SetAttribute (
          "OnTime",
          StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
      sourceHelper.SetAttribute (
          "OffTime",
          StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
      sourceHelper.SetConstantRate (DataRate (dataRate));
      sourceHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));

      ApplicationContainer sourceApp =
          sourceHelper.Install (staNodes.Get (sourceNodeIndex));
      sourceApp.Start (Seconds (startTimes[flowIndex]));
      sourceApp.Stop (Seconds (stopTimes[flowIndex]));
      clientApps.Add (sourceApp);
    }

  // --------------------------------------------------------------------------
  // Visualization and statistics.
  // --------------------------------------------------------------------------
  AnimationInterface animation ("ieee80211p-dtd-component.xml");

  for (uint32_t i = 0; i < staNodes.GetN (); ++i)
    {
      animation.UpdateNodeDescription (staNodes.Get (i), "Vehicle");
      animation.UpdateNodeColor (staNodes.Get (i), 255, 0, 0);
    }
  for (uint32_t i = 0; i < ap.GetN (); ++i)
    {
      animation.UpdateNodeDescription (ap.Get (i), "RSU");
      animation.UpdateNodeColor (ap.Get (i), 0, 255, 0);
    }
  for (uint32_t i = 0; i < staticNodes.GetN (); ++i)
    {
      animation.UpdateNodeDescription (staticNodes.Get (i), "StaticWaveNode");
      animation.UpdateNodeColor (staticNodes.Get (i), 0, 255, 0);
    }
  for (uint32_t i = 0; i < csmaNodes.GetN (); ++i)
    {
      animation.UpdateNodeDescription (csmaNodes.Get (i), "CoreNode");
      animation.UpdateNodeColor (csmaNodes.Get (i), 0, 255, 0);
    }

  FlowMonitorHelper flowMonitorHelper;
  Ptr<FlowMonitor> flowMonitor = flowMonitorHelper.InstallAll ();

  Simulator::Schedule (Seconds (kThroughputIntervalSeconds),
                       &CalculateAggregateThroughput);
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();

  PrintFlowStatistics (flowMonitor, flowMonitorHelper);
  flowMonitor->SerializeToXmlFile ("ieee80211p-dtd-component.flowmon",
                                   false,
                                   false);

  Simulator::Destroy ();
  return 0;
}
