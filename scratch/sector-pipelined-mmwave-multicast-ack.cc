/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ipv4-static-routing-helper.h"

// mmWave (NYU)
#include "ns3/mmwave-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"

// Antenna (PaperCone exists in src/antenna)
#include "ns3/antenna-module.h"

#include <cmath>
#include <vector>
#include <iostream>

using namespace ns3;
using namespace ns3::mmwave;

static inline double Deg2Rad (double deg) { return deg * M_PI / 180.0; }

static inline double WrapToPiLocal (double x)
{
  while (x > M_PI) x -= 2.0 * M_PI;
  while (x <= -M_PI) x += 2.0 * M_PI;
  return x;
}

struct UeInfo
{
  uint32_t idx{};
  double x{};
  double y{};
  double azRad{};
  uint32_t sector{};
};

class SectorPipeline
{
public:
  void Parse (int argc, char** argv)
  {
    CommandLine cmd;

    cmd.AddValue ("numUe", "Number Of UEs", m_numUe);
    cmd.AddValue ("radius", "UE Deployment Radius (m)", m_radius);
    cmd.AddValue ("simTime", "Simulation Time (s) (0=auto)", m_simTimeUser);

    // PaperCone params (Matlab 식)
    cmd.AddValue ("N", "Horizontal Antenna Elements N", m_N);
    cmd.AddValue ("beta", "Beta (Phase Excitation Difference)", m_beta);
    cmd.AddValue ("k3db", "3-dB Constant (2.782 Or 2.58)", m_k3db);

    // Sectorization
    cmd.AddValue ("numSectors", "0=Auto From HPBW, Else Fixed", m_numSectorsUser);

    // Pipeline timing
    cmd.AddValue ("dataMs", "DL Data Duration Per Sector (ms)", m_dataMs);
    cmd.AddValue ("ackGapMs", "ACK Listen Window Per Sector When BS Returns (ms)", m_ackGapMs);
    cmd.AddValue ("steps", "Number Of Sector Steps", m_steps);

    // Traffic sizes
    cmd.AddValue ("dataPktSize", "DL Packet Size (bytes)", m_dataPktSize);
    cmd.AddValue ("ackPktSize", "UL ACK Packet Size (bytes)", m_ackPktSize);

    // Channel state: 'l' LOS, 'n' NLOS
    cmd.AddValue ("channelState", "Channel State: l Or n", m_channelState);

    cmd.Parse (argc, argv);
  }

  void Run ()
  {
    ComputeHpbwAndSectors ();
    BuildNodesMobility ();
    BuildMmwaveCore ();
    InstallApps ();
    SchedulePipeline ();

    const double stopS = (m_simTimeUser > 0.0) ? m_simTimeUser : m_simTimeAuto;
    Simulator::Stop (Seconds (stopS));
    std::cout << "BeforeRun, StopAt=" << stopS << "s" << std::endl;
    Simulator::Run ();
    std::cout << "AfterRun" << std::endl;

    PrintResults ();
    Simulator::Destroy ();
  }

private:
  // ---------------- parameters ----------------
  uint32_t m_numUe {60};
  double   m_radius {100.0};
  double   m_simTimeUser {0.0};

  uint32_t m_N {64};
  double   m_beta {0.0};
  double   m_k3db {2.782};

  int32_t  m_numSectorsUser {0};
  uint32_t m_numSectors {6};

  double   m_dataMs {1.0};
  double   m_ackGapMs {0.2};
  uint32_t m_steps {60};

  uint32_t m_dataPktSize {1200};
  uint32_t m_ackPktSize {20};

  std::string m_channelState {"n"};

  // ---------------- derived ----------------
  double   m_hpbwDeg {30.0};
  double   m_hpbwRad {Deg2Rad (30.0)};
  double   m_cycleMs {0.0};
  double   m_simTimeAuto {2.0};

  // ---------------- ns-3 objects ----------------
  Ptr<MmWaveHelper> m_mmwHelper;
  Ptr<MmWavePointToPointEpcHelper> m_epcHelper;

  NodeContainer m_enbNodes;
  NodeContainer m_ueNodes;
  NodeContainer m_remoteHostContainer;

  NetDeviceContainer m_enbDevs;
  NetDeviceContainer m_ueDevs;

  Ptr<Node> m_remoteHost;
  Ipv4InterfaceContainer m_ueIfaces;
  Ipv4InterfaceContainer m_internetIfaces;

  // ---------------- bookkeeping ----------------
  std::vector<UeInfo> m_ueInfo;
  std::vector<Ptr<PacketSink>> m_dlSinks;
  Ptr<PacketSink> m_ackSink;

  uint64_t m_expectedDlPkts {0};
  uint64_t m_expectedAckPkts {0};

private:
  double ComputeHpbwDegPaper () const
  {
    // Matlab:
    // pl_th=acos(-beta+ k3db/(N*pi));
    // n_th =acos(-beta- k3db/(N*pi));
    // alpha=2*abs((pi/2-n_th))*180/pi;
    const double term = m_k3db / (static_cast<double>(m_N) * M_PI);
    const double n_th = std::acos (-m_beta - term);
    const double alpha = 2.0 * std::abs ((M_PI / 2.0) - n_th) * 180.0 / M_PI;
    return alpha;
  }

  uint32_t AutoNumSectors (double hpbwRad) const
  {
    const double val = (2.0 * M_PI) / std::max (1e-9, hpbwRad);
    const uint32_t k = static_cast<uint32_t> (std::ceil (val));
    return std::max (1u, k);
  }

  double SectorCenterAz (uint32_t s) const
  {
    const double width = (2.0 * M_PI) / static_cast<double>(m_numSectors);
    const double center = -M_PI + (static_cast<double>(s) + 0.5) * width;
    return WrapToPiLocal (center);
  }

  uint32_t AzToSector (double az) const
  {
    const double width = (2.0 * M_PI) / static_cast<double>(m_numSectors);
    const double shifted = az + M_PI; // [0, 2pi)
    uint32_t s = static_cast<uint32_t> (std::floor (shifted / width));
    if (s >= m_numSectors) s = m_numSectors - 1;
    return s;
  }

  void ComputeHpbwAndSectors ()
  {
    m_hpbwDeg = ComputeHpbwDegPaper ();
    m_hpbwRad = Deg2Rad (m_hpbwDeg);

    if (m_numSectorsUser <= 0) m_numSectors = AutoNumSectors (m_hpbwRad);
    else m_numSectors = static_cast<uint32_t>(m_numSectorsUser);

    m_cycleMs = static_cast<double>(m_numSectors) * (m_dataMs + m_ackGapMs);

    const double totalMs =
      static_cast<double>(m_steps) * (m_dataMs + m_ackGapMs) + m_cycleMs + 10.0;
    m_simTimeAuto = totalMs / 1000.0;

    std::cout
      << "ComputedHPBWdeg=" << m_hpbwDeg
      << ", NumSectors=" << m_numSectors
      << ", CycleMs=" << m_cycleMs
      << ", SimTimeAuto(s)=" << m_simTimeAuto
      << std::endl;
  }

  void BuildNodesMobility ()
  {
    m_enbNodes.Create (1);
    m_ueNodes.Create (m_numUe);
    m_remoteHostContainer.Create (1);

    m_remoteHost = m_remoteHostContainer.Get (0);

    MobilityHelper mob;
    mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mob.Install (m_enbNodes);
    mob.Install (m_ueNodes);
    mob.Install (m_remoteHostContainer);

    m_enbNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (0.0, 0.0, 10.0));
    m_remoteHost->GetObject<MobilityModel> ()->SetPosition (Vector (0.0, -200.0, 1.5));

    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
    m_ueInfo.clear ();
    m_ueInfo.reserve (m_numUe);

    for (uint32_t i = 0; i < m_numUe; ++i)
    {
      const double r = m_radius * std::sqrt (uv->GetValue (0.0, 1.0));
      const double th = uv->GetValue (-M_PI, M_PI);
      const double x = r * std::cos (th);
      const double y = r * std::sin (th);

      m_ueNodes.Get (i)->GetObject<MobilityModel> ()->SetPosition (Vector (x, y, 1.5));

      UeInfo u;
      u.idx = i;
      u.x = x;
      u.y = y;
      u.azRad = WrapToPiLocal (std::atan2 (y, x));
      u.sector = 0;
      m_ueInfo.push_back (u);
    }

    for (auto &u : m_ueInfo) u.sector = AzToSector (u.azRad);
  }

  void BuildMmwaveCore ()
  {
    // 기본 설정
    // 후보 2: 포크에 따라 CenterFreq로 존재하는 경우
    // Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (28e9));
    
    // helper
    m_mmwHelper = CreateObject<MmWaveHelper> ();
    m_mmwHelper->SetPathlossModelType ("ns3::MmWavePropagationLossModel");

    if (m_channelState == "l")
    {
      m_mmwHelper->SetChannelConditionModelType ("ns3::AlwaysLosChannelConditionModel");
    }
    else if (m_channelState == "n")
    {
      m_mmwHelper->SetChannelConditionModelType ("ns3::NeverLosChannelConditionModel");
    }
    else
    {
      NS_FATAL_ERROR ("Unknown channelState");
    }

    // EPC
    m_epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
    m_mmwHelper->SetEpcHelper (m_epcHelper);

    ConfigStore inputConfig;
    inputConfig.ConfigureDefaults ();

    // 둘 다 명시적으로 고정(덮어쓰기 방지)
    Config::SetDefault ("ns3::MmWavePhyMacCommon::CenterFreq", DoubleValue (28e9));
    Config::SetDefault ("ns3::MmWavePropagationLossModel::Frequency", DoubleValue (28e9));
    Config::SetDefault ("ns3::MmWaveAmc::Ber", DoubleValue (0.01));
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (true));
    Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (true));
    // 인터넷 구성
    Ptr<Node> pgw = m_epcHelper->GetPgwNode ();

    InternetStackHelper internet;
    internet.Install (m_remoteHostContainer);
    internet.Install (m_ueNodes);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (MicroSeconds (100.0)));
    NetDeviceContainer internetDevices = p2ph.Install (pgw, m_remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
    m_internetIfaces = ipv4h.Assign (internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
      ipv4RoutingHelper.GetStaticRouting (m_remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

    // mmWave devices
    m_enbDevs = m_mmwHelper->InstallEnbDevice (m_enbNodes);
    m_ueDevs  = m_mmwHelper->InstallUeDevice (m_ueNodes);

    // UE IP
    m_ueIfaces = m_epcHelper->AssignUeIpv4Address (NetDeviceContainer (m_ueDevs));
    for (uint32_t u = 0; u < m_numUe; ++u)
    {
      Ptr<Node> ueNode = m_ueNodes.Get (u);
      Ptr<Ipv4StaticRouting> ueStaticRouting =
        ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (m_epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    m_mmwHelper->AttachToClosestEnb (m_ueDevs, m_enbDevs);

    // PaperCone 파라미터(현재는 "스케줄링/출력용"으로도 사용)
    Config::SetDefault ("ns3::PaperConeAntennaModel::N", UintegerValue (m_N));
    Config::SetDefault ("ns3::PaperConeAntennaModel::Beta", DoubleValue (m_beta));
    Config::SetDefault ("ns3::PaperConeAntennaModel::K3db", DoubleValue (m_k3db));
    Config::SetDefault ("ns3::PaperConeAntennaModel::BoresightAzimuthRad", DoubleValue (SectorCenterAz (0)));

    m_mmwHelper->EnableTraces ();

    std::cout << "RemoteHostAddr=" << m_internetIfaces.GetAddress (1) << std::endl;
  }

  void InstallApps ()
  {
    const uint16_t dlPortBase = 10000;
    const uint16_t ackPort = 20000;

    m_dlSinks.assign (m_numUe, nullptr);

    // DL sink on each UE
    for (uint32_t u = 0; u < m_numUe; ++u)
    {
      const uint16_t dlPort = dlPortBase + static_cast<uint16_t>(u);

      PacketSinkHelper dlSink ("ns3::UdpSocketFactory",
                               InetSocketAddress (Ipv4Address::GetAny (), dlPort));
      ApplicationContainer apps = dlSink.Install (m_ueNodes.Get (u));
      apps.Start (Seconds (0.01));
      apps.Stop (Seconds ((m_simTimeUser > 0.0) ? m_simTimeUser : m_simTimeAuto));

      m_dlSinks[u] = DynamicCast<PacketSink> (apps.Get (0));
    }

    // ACK sink on remote host (UL)
    PacketSinkHelper ackSink ("ns3::UdpSocketFactory",
                              InetSocketAddress (Ipv4Address::GetAny (), ackPort));
    ApplicationContainer ackApps = ackSink.Install (m_remoteHost);
    ackApps.Start (Seconds (0.01));
    ackApps.Stop (Seconds ((m_simTimeUser > 0.0) ? m_simTimeUser : m_simTimeAuto));
    m_ackSink = DynamicCast<PacketSink> (ackApps.Get (0));
  }

  void SetBoresight (double azRad)
  {
    // helper 경로가 repo마다 달라서, 안 맞으면 아무 영향 없음(스케줄링 로직은 유지됨)
    Config::Set ("/NodeList/*/DeviceList/*/$ns3::MmWaveEnbNetDevice/*/Antenna/*/BoresightAzimuthRad",
               DoubleValue (azRad));
    // Config::Set ("/NodeList/*/DeviceList/*/$ns3::PaperConeAntennaModel/BoresightAzimuthRad",
    //              DoubleValue (azRad));
  }

  void ScheduleDlOnePkt (uint32_t ueIdx, Time tStart)
  {
    const uint16_t dlPort = 10000 + static_cast<uint16_t>(ueIdx);

    UdpClientHelper dlClient (m_ueIfaces.GetAddress (ueIdx), dlPort);
    dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (1.0)));
    dlClient.SetAttribute ("MaxPackets", UintegerValue (1));
    dlClient.SetAttribute ("PacketSize", UintegerValue (m_dataPktSize));

    ApplicationContainer app = dlClient.Install (m_remoteHost);
    app.Start (tStart);
    app.Stop (tStart + MilliSeconds (1.0));
  }

  void ScheduleAckOnePkt (uint32_t ueIdx, Time tStart)
  {
    const uint16_t ackPort = 20000;
    const Ipv4Address remoteHostAddr = m_internetIfaces.GetAddress (1);

    UdpClientHelper ackClient (remoteHostAddr, ackPort);
    ackClient.SetAttribute ("Interval", TimeValue (MilliSeconds (1.0)));
    ackClient.SetAttribute ("MaxPackets", UintegerValue (1));
    ackClient.SetAttribute ("PacketSize", UintegerValue (m_ackPktSize));

    ApplicationContainer app = ackClient.Install (m_ueNodes.Get (ueIdx));
    app.Start (tStart);
    app.Stop (tStart + MilliSeconds (1.0));
  }

  void Step (uint32_t k)
  {
    const uint32_t s = (k % m_numSectors);

    const double stepMs = (m_dataMs + m_ackGapMs);
    const Time t0 = MilliSeconds (static_cast<double>(k) * stepMs);

    // 1) 섹터 s로 스티어링(논리적) + DL 데이터 전송(섹터 내 UE들에게 동일 패킷 복제)
    SetBoresight (SectorCenterAz (s));

    for (const auto &u : m_ueInfo)
    {
      if (u.sector != s) continue;

      ScheduleDlOnePkt (u.idx, t0);
      m_expectedDlPkts += 1;

      // 2) ACK는 "한 바퀴 뒤 같은 섹터로 돌아온 시점"에 송신되도록 스케줄
      const Time tReturn = t0 + MilliSeconds (m_cycleMs);
      const Time tAckTx = tReturn + MilliSeconds (0.01);
      ScheduleAckOnePkt (u.idx, tAckTx);
      m_expectedAckPkts += 1;
    }

    // 3) 돌아온 시점에 다시 해당 섹터로 스티어링(ACK 수신 윈도우 의미)
    const Time tListen = t0 + MilliSeconds (m_cycleMs);
    Simulator::Schedule (tListen, &SectorPipeline::SetBoresight, this, SectorCenterAz (s));
  }

  void SchedulePipeline ()
  {
    for (uint32_t k = 0; k < m_steps; ++k)
    {
      Simulator::Schedule (MilliSeconds (static_cast<double>(k) * (m_dataMs + m_ackGapMs)),
                           &SectorPipeline::Step, this, k);
    }
    Simulator::Schedule (MilliSeconds (1), &SectorPipeline::Heartbeat, this);

  }

  void Heartbeat ()
  {
    std::cout << "Now=" << Simulator::Now ().GetSeconds () << "s" << std::endl;
    Simulator::Schedule (MilliSeconds (10), &SectorPipeline::Heartbeat, this);
  }


  void PrintResults ()
  {
    uint64_t rxDlPkts = 0;
    for (uint32_t u = 0; u < m_numUe; ++u)
    {
      if (!m_dlSinks[u]) continue;
      const uint64_t bytes = m_dlSinks[u]->GetTotalRx ();
      rxDlPkts += (bytes / m_dataPktSize);
    }

    uint64_t rxAckPkts = 0;
    if (m_ackSink)
    {
      const uint64_t bytes = m_ackSink->GetTotalRx ();
      rxAckPkts = (bytes / m_ackPktSize);
    }

    const double dlFer = (m_expectedDlPkts == 0) ? 0.0
      : 1.0 - (static_cast<double>(rxDlPkts) / static_cast<double>(m_expectedDlPkts));

    const double ackFer = (m_expectedAckPkts == 0) ? 0.0
      : 1.0 - (static_cast<double>(rxAckPkts) / static_cast<double>(m_expectedAckPkts));

    std::cout << "\n--- Results ---\n";
    std::cout << "ExpectedDlPkts=" << m_expectedDlPkts
              << ", RxDlPkts=" << rxDlPkts
              << ", DL_FER=" << dlFer << "\n";
    std::cout << "ExpectedAckPkts=" << m_expectedAckPkts
              << ", RxAckPkts=" << rxAckPkts
              << ", ACK_FER=" << ackFer << "\n";
  }
};

int main (int argc, char** argv)
{
  SectorPipeline sim;
  sim.Parse (argc, argv);
  sim.Run ();
  return 0;
}
