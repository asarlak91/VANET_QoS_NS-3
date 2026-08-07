/*
 * LTE DID component for a VANET simulation.
 *
 * This is a cleaned public-release version of legacy code developed for
 * ns-3.26. The original LTE/EPC topology, four groups of 20 moving UEs,
 * 160-byte packets, 20 ms packet interval, and simulation duration are
 * preserved.
 *
 * Scope: this file models the LTE / delay-intolerant-data component. It is not
 * a complete implementation of the integrated IQDN SDN controller.
 */

#include <iostream>
#include <stdint.h>
#include <vector>

#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LteDidComponent");

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
InstallConstantVelocityMobility (NodeContainer nodes,
                                 double minX,
                                 double minY,
                                 double maxX)
{
  MobilityHelper mobility;
  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (minX),
      "MinY",
      DoubleValue (minY),
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
      RectangleValue (Rectangle (-100.0, maxX, 190.0, 300.0)),
      "Speed",
      StringValue ("ns3::ConstantRandomVariable[Constant=20]"),
      "Direction",
      StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  mobility.Install (nodes);
}

void
ConfigureUeDefaultRoutes (NodeContainer ueNodes,
                          Ptr<PointToPointEpcHelper> epcHelper)
{
  Ipv4StaticRoutingHelper routingHelper;
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      Ptr<Ipv4StaticRouting> ueRouting = routingHelper.GetStaticRouting (
          ueNodes.Get (i)->GetObject<Ipv4> ());
      ueRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
}

void
AttachUesToEnb (Ptr<LteHelper> lteHelper,
                 const NetDeviceContainer &ueDevices,
                 Ptr<NetDevice> enbDevice)
{
  for (uint32_t i = 0; i < ueDevices.GetN (); ++i)
    {
      lteHelper->Attach (ueDevices.Get (i), enbDevice);
    }
}

void
InstallDownlinkTraffic (NodeContainer ueNodes,
                        const Ipv4InterfaceContainer &ueInterfaces,
                        Ptr<Node> remoteHost,
                        uint16_t downlinkPort,
                        uint16_t packetSize,
                        double packetIntervalMs,
                        double startTime,
                        double stopTime,
                        ApplicationContainer &serverApps,
                        ApplicationContainer &clientApps)
{
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
      PacketSinkHelper sinkHelper (
          "ns3::UdpSocketFactory",
          InetSocketAddress (Ipv4Address::GetAny (), downlinkPort));
      ApplicationContainer sinkApp = sinkHelper.Install (ueNodes.Get (i));
      sinkApp.Start (Seconds (startTime));
      sinkApp.Stop (Seconds (stopTime));
      serverApps.Add (sinkApp);
      g_packetSinks.push_back (StaticCast<PacketSink> (sinkApp.Get (0)));

      UdpClientHelper downlinkClient (ueInterfaces.GetAddress (i),
                                      downlinkPort);
      downlinkClient.SetAttribute (
          "Interval",
          TimeValue (MilliSeconds (packetIntervalMs)));
      downlinkClient.SetAttribute ("MaxPackets", UintegerValue (10000000));
      downlinkClient.SetAttribute ("PacketSize", UintegerValue (packetSize));
      ApplicationContainer clientApp = downlinkClient.Install (remoteHost);
      clientApp.Start (Seconds (startTime));
      clientApp.Stop (Seconds (stopTime));
      clientApps.Add (clientApp);
    }
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
  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults ();

  // Original simulation parameters.
  uint16_t numberOfUesPerGroup = 20;
  uint16_t numberOfEnbs = 1;
  uint16_t packetSize = 160;
  double packetIntervalMs = 20.0;
  double enbTxPowerDbm = 46.0;
  uint32_t seed = 1234;
  double totalTime = 65.0;
  double applicationStartTime = 2.0;
  double applicationStopTime = 62.0;

  Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (160));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));

  CommandLine cmd;
  cmd.AddValue ("numberOfUesPerGroup",
                "Number of UEs in each of the four mobility groups",
                numberOfUesPerGroup);
  cmd.AddValue ("numberOfEnbs", "Number of LTE eNodeBs", numberOfEnbs);
  cmd.AddValue ("enbTxPowerDbm", "eNodeB transmit power in dBm", enbTxPowerDbm);
  cmd.AddValue ("packetSize", "DID packet size in bytes", packetSize);
  cmd.AddValue ("packetIntervalMs",
                "DID packet interval in milliseconds",
                packetIntervalMs);
  cmd.AddValue ("seed", "Random-number seed", seed);
  cmd.AddValue ("time", "Simulation time in seconds", totalTime);
  cmd.Parse (argc, argv);

  if (numberOfEnbs == 0 || numberOfUesPerGroup == 0)
    {
      std::cerr << "The scenario requires at least one eNodeB and one UE."
                << std::endl;
      return 1;
    }
  if (applicationStopTime > totalTime)
    {
      applicationStopTime = totalTime;
    }

  SeedManager::SetSeed (seed);

  // --------------------------------------------------------------------------
  // LTE/EPC core.
  // --------------------------------------------------------------------------
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper =
      CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  lteHelper->SetHandoverAlgorithmType (
      "ns3::A2A4RsrqHandoverAlgorithm");
  lteHelper->SetHandoverAlgorithmAttribute (
      "ServingCellThreshold",
      UintegerValue (30));
  lteHelper->SetHandoverAlgorithmAttribute (
      "NeighbourCellOffset",
      UintegerValue (1));

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);

  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate",
                           DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetInterfaces =
      ipv4h.Assign (internetDevices);
  const Ipv4Address remoteHostAddress = internetInterfaces.GetAddress (1);
  (void) remoteHostAddress;

  Ipv4StaticRoutingHelper routingHelper;
  Ptr<Ipv4StaticRouting> remoteHostRouting =
      routingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"),
                                        Ipv4Mask ("255.0.0.0"),
                                        1);

  // --------------------------------------------------------------------------
  // eNodeB and UE nodes.
  // --------------------------------------------------------------------------
  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);

  NodeContainer ueNodesA;
  NodeContainer ueNodesB;
  NodeContainer ueNodesC;
  NodeContainer ueNodesD;
  ueNodesA.Create (numberOfUesPerGroup);
  ueNodesB.Create (numberOfUesPerGroup);
  ueNodesC.Create (numberOfUesPerGroup);
  ueNodesD.Create (numberOfUesPerGroup);

  std::cout << "Creating " << numberOfEnbs << " eNodeB node(s)\n";
  std::cout << "Creating " << 4 * numberOfUesPerGroup << " UE node(s)\n";

  // Remote host mobility.
  MobilityHelper mobility;
  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (1000.0),
      "MinY",
      DoubleValue (30.0),
      "DeltaX",
      DoubleValue (1000.0),
      "DeltaY",
      DoubleValue (5.0),
      "GridWidth",
      UintegerValue (10),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (remoteHostContainer);

  // eNodeB mobility.
  mobility.SetPositionAllocator (
      "ns3::GridPositionAllocator",
      "MinX",
      DoubleValue (5000.0),
      "MinY",
      DoubleValue (220.0),
      "DeltaX",
      DoubleValue (3000.0),
      "DeltaY",
      DoubleValue (5.0),
      "GridWidth",
      UintegerValue (20),
      "LayoutType",
      StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enbNodes);

  // Four original UE groups: two lanes beginning at x=0 and two beginning at
  // x=5250, all moving at 20 m/s in the positive x direction.
  InstallConstantVelocityMobility (ueNodesA, 0.0, 235.0, 12000.0);
  InstallConstantVelocityMobility (ueNodesB, 0.0, 245.0, 12000.0);
  InstallConstantVelocityMobility (ueNodesC, 5250.0, 235.0, 12000.0);
  InstallConstantVelocityMobility (ueNodesD, 5250.0, 245.0, 12000.0);

  // --------------------------------------------------------------------------
  // LTE devices, IP addresses, routes, and attachment.
  //
  // The legacy source overwrote one UE device container four times. Separate
  // containers preserve all four groups and ensure that each node receives the
  // matching LTE device and IP address.
  // --------------------------------------------------------------------------
  Config::SetDefault ("ns3::LteEnbPhy::TxPower",
                      DoubleValue (enbTxPowerDbm));
  NetDeviceContainer enbDevices = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevicesA = lteHelper->InstallUeDevice (ueNodesA);
  NetDeviceContainer ueDevicesB = lteHelper->InstallUeDevice (ueNodesB);
  NetDeviceContainer ueDevicesC = lteHelper->InstallUeDevice (ueNodesC);
  NetDeviceContainer ueDevicesD = lteHelper->InstallUeDevice (ueNodesD);

  internet.Install (ueNodesA);
  internet.Install (ueNodesB);
  internet.Install (ueNodesC);
  internet.Install (ueNodesD);

  Ipv4InterfaceContainer ueInterfacesA =
      epcHelper->AssignUeIpv4Address (ueDevicesA);
  Ipv4InterfaceContainer ueInterfacesB =
      epcHelper->AssignUeIpv4Address (ueDevicesB);
  Ipv4InterfaceContainer ueInterfacesC =
      epcHelper->AssignUeIpv4Address (ueDevicesC);
  Ipv4InterfaceContainer ueInterfacesD =
      epcHelper->AssignUeIpv4Address (ueDevicesD);

  ConfigureUeDefaultRoutes (ueNodesA, epcHelper);
  ConfigureUeDefaultRoutes (ueNodesB, epcHelper);
  ConfigureUeDefaultRoutes (ueNodesC, epcHelper);
  ConfigureUeDefaultRoutes (ueNodesD, epcHelper);

  Ptr<NetDevice> servingEnb = enbDevices.Get (0);
  AttachUesToEnb (lteHelper, ueDevicesA, servingEnb);
  AttachUesToEnb (lteHelper, ueDevicesB, servingEnb);
  AttachUesToEnb (lteHelper, ueDevicesC, servingEnb);
  AttachUesToEnb (lteHelper, ueDevicesD, servingEnb);

  // Keep the unused groups' interfaces explicit: they are part of the original
  // 80-UE topology even though downlink applications were placed on groups A
  // and D in the recovered source.
  (void) ueInterfacesB;
  (void) ueInterfacesC;

  if (numberOfEnbs > 1)
    {
      lteHelper->AddX2Interface (enbNodes);
    }

  // --------------------------------------------------------------------------
  // Delay-intolerant downlink traffic.
  // --------------------------------------------------------------------------
  const uint16_t downlinkPort = 90;
  ApplicationContainer serverApps;
  ApplicationContainer clientApps;

  // The recovered code created the same 20 downlink flows for UE groups A and
  // D. The helper below removes repeated blocks while preserving that behavior.
  InstallDownlinkTraffic (ueNodesA,
                          ueInterfacesA,
                          remoteHost,
                          downlinkPort,
                          packetSize,
                          packetIntervalMs,
                          applicationStartTime,
                          applicationStopTime,
                          serverApps,
                          clientApps);
  InstallDownlinkTraffic (ueNodesD,
                          ueInterfacesD,
                          remoteHost,
                          downlinkPort,
                          packetSize,
                          packetIntervalMs,
                          applicationStartTime,
                          applicationStopTime,
                          serverApps,
                          clientApps);

  // --------------------------------------------------------------------------
  // Statistics.
  // --------------------------------------------------------------------------
  FlowMonitorHelper flowMonitorHelper;
  Ptr<FlowMonitor> flowMonitor = flowMonitorHelper.InstallAll ();

  Simulator::Schedule (Seconds (applicationStartTime),
                       &CalculateAggregateThroughput);
  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();

  PrintFlowStatistics (flowMonitor, flowMonitorHelper);
  flowMonitor->SerializeToXmlFile ("lte-did-component.flowmon", false, false);

  Simulator::Destroy ();
  return 0;
}
