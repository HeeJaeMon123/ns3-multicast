/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-module.h"
#include "ns3/antenna-module.h"
#include "ns3/internet-module.h" 

#include "ns3/mobility-model.h"
#include <cmath>
#include <set>
#include <vector>
#include <map>

using namespace ns3;
using namespace ns3::mmwave;

namespace ns3 {
namespace mmwave {
    extern uint32_t g_currentTxSector;
}
}

NS_LOG_COMPONENT_DEFINE ("MmWaveMulticastResearch");

// ================= 전역 변수 =================
std::map<uint16_t, Ptr<MmWaveUeMac>> g_ueMacMap;
std::vector<uint32_t> g_activeSectors; 
uint32_t g_activeIdx = 0; 

// ================= 헬퍼 함수들 =================

// [App Trace] 송신 로그
void AppTxTrace (Ptr<const Packet> p) {
    // 필요 시 주석 해제하여 송신 확인
    // NS_LOG_UNCOND("App TX: " << p->GetSize() << " bytes");
}

// [IP Receive Trace] IP 계층 수신 확인
void TraceIpReceive (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface) {
    Ipv4Header header;
    packet->PeekHeader (header);
    if (header.GetDestination () == Ipv4Address ("224.0.1.1")) {
        NS_LOG_UNCOND ("IP LAYER: UE " << ipv4->GetAddress(1,0).GetLocal() 
                       << " received Multicast Packet! Size: " << packet->GetSize());
    }
}

// [Beam Steering] 기지국 빔 조향
void ApplyBeamSteering (Ptr<MmWaveEnbNetDevice> enbDev, double targetAzDeg, double targetElDeg) {
    Ptr<MmWaveEnbPhy> enbPhy = enbDev->GetPhy ();
    Ptr<MmWaveSpectrumPhy> enbSpecPhy = enbPhy->GetSpectrumPhy ();
    Ptr<PhasedArrayModel> enbAntenna = DynamicCast<PhasedArrayModel> (enbSpecPhy->GetAntenna ());
    
    if (enbAntenna) {
        double azRad = targetAzDeg * M_PI / 180.0;
        double elRad = targetElDeg * M_PI / 180.0;
        enbAntenna->SetBeamformingVector (enbAntenna->GetBeamformingVector (Angles (azRad, elRad)));
        std::cout << "성공: 빔 조향 완료 (Az: " << targetAzDeg << "°, El: " << targetElDeg << "°)" << std::endl;
    }
}

// [Research Cycle] 연구용 빔 스윕 사이클
enum ResearchPhase { DATA_TX, PARALLEL_PROCESSING, ACK_RX };
ResearchPhase g_phase = DATA_TX;
void ExecuteResearchCycle (Ptr<MmWaveEnbNetDevice> enbDev, NodeContainer enbNodes, NodeContainer ueNodes, double hBeamWidth)
{
    if (g_activeSectors.empty()) return;

    uint32_t currentSector = g_activeSectors[g_activeIdx];
    uint32_t nextActiveIdx = (g_activeIdx + 1) % g_activeSectors.size();
    uint32_t nextSector = g_activeSectors[nextActiveIdx];

    // [핵심] 빔을 물리적으로 돌리기 직전에 "정답지(전역변수)"를 업데이트합니다.
    // 이제 모든 UE는 이 값을 보고 "아, 지금은 0번 섹터 차례구나" 하고 알게 됩니다.
    ns3::mmwave::g_currentTxSector = currentSector;

    double targetAz;

    switch (g_phase) {
        case DATA_TX: 
            targetAz = (currentSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 1: Data TX -> Sector " << currentSector 
                           << " (RNTI " << (1001 + currentSector) << ")");
            g_phase = PARALLEL_PROCESSING;
            break;

        case PARALLEL_PROCESSING: 
            targetAz = (nextSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 2: Data TX -> Sector " << nextSector 
                           << " | Sector " << currentSector << " is Aggregating ACKs");
            
            // Phase 2에서는 빔이 다음 섹터로 넘어갔으므로 정답지도 업데이트
            ns3::mmwave::g_currentTxSector = nextSector; 
            
            g_phase = ACK_RX;
            break;

        case ACK_RX: 
            targetAz = (currentSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 3: Listening ACK from Sector " << currentSector);
            
            // Phase 3는 다시 이전 섹터를 보므로 복구
            ns3::mmwave::g_currentTxSector = currentSector;
            
            g_activeIdx = nextActiveIdx; 
            g_phase = DATA_TX;
            break;
    }
    Simulator::Schedule (MilliSeconds (100), &ExecuteResearchCycle, enbDev, enbNodes, ueNodes, hBeamWidth);
}

// [Angles] 각도 계산
std::pair<double, double> GetUeAngles (Ptr<Node> enb, Ptr<Node> ue) {
    Ptr<MobilityModel> enbMob = enb->GetObject<MobilityModel> ();
    Ptr<MobilityModel> ueMob = ue->GetObject<MobilityModel> ();
    Vector enbPos = enbMob->GetPosition ();
    Vector uePos = ueMob->GetPosition ();
    double azRad = atan2 (uePos.y - enbPos.y, uePos.x - enbPos.x);
    double azDeg = azRad * 180.0 / M_PI;
    double dist = sqrt (pow (uePos.x - enbPos.x, 2) + pow (uePos.y - enbPos.y, 2));
    double elRad = atan2 (dist, uePos.z - enbPos.z);
    double elDeg = elRad * 180.0 / M_PI;
    return {azDeg, elDeg};
}

// [Monitor] 수신 상태 모니터링
void MonitorMulticastReception (NodeContainer enbNodes, NodeContainer ueNodes, double hBeamWidth) {
    std::cout << "\n=== [Time: " << Simulator::Now ().GetSeconds () << "s] Multicast Reception Monitor ===" << std::endl;
    uint32_t nSectors = static_cast<uint32_t> (ceil (360.0 / hBeamWidth));

    for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
        Ptr<Node> ueNode = ueNodes.Get (i);
        uint32_t receivedPackets = 0;

        for (uint32_t j = 0; j < ueNode->GetNApplications(); ++j) {
            Ptr<PacketSink> sink = DynamicCast<PacketSink> (ueNode->GetApplication (j));
            if (sink) {
                AddressValue localAddrValue;
                sink->GetAttribute("Local", localAddrValue);
                Address localAddr = localAddrValue.Get();
                InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(localAddr);
                uint64_t totalRxBytes = sink->GetTotalRx();
                
                std::cout << "  [UE " << i << " App " << j << "] Listening: " 
                          << inetAddr.GetIpv4() << ":" << inetAddr.GetPort()
                          << " | RawRx: " << totalRxBytes << " bytes" << std::endl;
                receivedPackets += (totalRxBytes / 1200);
            }
        }

        auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNode);
        double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;
        uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / hBeamWidth) % nSectors;
        double beamCenter = (sectorIdx * hBeamWidth) + (hBeamWidth / 2.0);
        double angleError = std::abs (normalizedAz - beamCenter);
        bool isInside = (angleError <= (hBeamWidth / 2.0));

        std::cout << "UE " << i << " | Received: " << receivedPackets 
                  << " | Angle: " << azimuth << "° | Sector: " << sectorIdx 
                  << " | Center: " << beamCenter << "° | InBeam: " << (isInside ? "YES" : "NO") << std::endl;
    }
    Simulator::Schedule (MilliSeconds (200), &MonitorMulticastReception, enbNodes, ueNodes, hBeamWidth);
}

// [main.cc 수정]
void ForwardToIp (Ptr<Node> node, Ptr<Packet> p) {
    Ptr<Ipv4L3Protocol> ipv4 = node->GetObject<Ipv4L3Protocol> ();
    
    // [핵심] 보통 Loopback은 0번, mmWave는 1번 인터페이스입니다.
    // NetDevice 포인터를 찾는 번거로운 과정 없이, 
    // "1번 인터페이스(mmWave)에 연결된 장치"를 바로 가져옵니다.
    int32_t interfaceIndex = 1; 
    
    // 안전장치: 인터페이스가 2개 이상인지 확인 (0:lo, 1:mmwave)
    if (ipv4->GetNInterfaces() <= interfaceIndex) {
        return; 
    }

    Ptr<NetDevice> mmWaveDevice = ipv4->GetNetDevice(interfaceIndex);

    if (ipv4 && mmWaveDevice) {
        // [디버깅] 패킷이 IP 계층으로 진입함을 알림
        NS_LOG_UNCOND("ForwardToIp: Injecting packet to Node " << node->GetId());

        ipv4->Receive (
            mmWaveDevice, 
            p, 
            0x0800, // IPv4 Protocol
            Address(), 
            Address(), 
            NetDevice::PACKET_HOST 
        );
    }
}
// [Initial Setup] 스케줄러, PHY, MAC 연결 및 초기화
void ConnectSchedulerAndPhy (Ptr<MmWaveEnbNetDevice> enbDev, NetDeviceContainer ueNetDevs, Ptr<MmWaveHelper> helper) {
    Ptr<MmWaveEnbMac> enbMac = enbDev->GetMac ();
    if (!enbMac) return;
    Ptr<MmWaveEnbPhy> enbPhy = enbDev->GetPhy ();
    
    Ptr<MmWaveFlexTtiMacScheduler> scheduler = DynamicCast<MmWaveFlexTtiMacScheduler> (enbMac->GetScheduler ());
    if (!scheduler) {
        scheduler = CreateObject<MmWaveFlexTtiMacScheduler> ();
        Ptr<MmWavePhyMacCommon> config = enbMac->GetConfigurationParameters (); 
        if (config) {
            scheduler->ConfigureCommonParameters (config);
            std::cout << "성공: 스케줄러 설정 동기화 완료." << std::endl;
        } else {
            Simulator::Schedule (MicroSeconds (1), &ConnectSchedulerAndPhy, enbDev, ueNetDevs, helper);
            return;
        }
        enbMac->SetMmWaveMacSchedSapProvider (scheduler->GetMacSchedSapProvider ());
        scheduler->SetMacSchedSapUser (enbMac->GetMmWaveMacSchedSapUser ());
        enbDev->AggregateObject (scheduler);
    }

    EpsBearer mcastBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT); 
    helper->ActivateDataRadioBearer (ueNetDevs, mcastBearer);




    Ptr<MmWaveSpectrumPhy> enbSpecPhy = enbPhy->GetSpectrumPhy ();
    if (enbSpecPhy) {
        scheduler->SetPhy (enbSpecPhy);
        Ptr<PhasedArrayModel> enbAntenna = DynamicCast<PhasedArrayModel> (enbSpecPhy->GetAntenna ());
        if (enbAntenna) {
            enbAntenna->SetBeamformingVector (enbAntenna->GetBeamformingVector (Angles (0, 0)));
        }
    }

    for (uint32_t i = 0; i < ueNetDevs.GetN (); ++i) {
        Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDevs.Get (i));
        if (ueDev) {
            Ptr<MmWaveUePhy> uePhy = ueDev->GetPhy ();
            Ptr<MmWaveSpectrumPhy> ueSpecPhy = uePhy->GetDlSpectrumPhy (); 
            if (ueSpecPhy) {
                Ptr<PhasedArrayModel> ueAntenna = DynamicCast<PhasedArrayModel> (ueSpecPhy->GetAntenna ());
                if (ueAntenna) {
                    ueAntenna->SetBeamformingVector (ueAntenna->GetBeamformingVector (Angles (0, 0)));
                }
            }
            // MAC Callback 연결
            Ptr<MmWaveUeMac> ueMac = ueDev->GetMac ();
            Ptr<Node> node = ueNetDevs.Get(i)->GetNode(); 
            ueMac->SetForwardUpCallback (MakeBoundCallback (&ForwardToIp, node));
        }
    }
    std::cout << "성공: 기지국 및 " << ueNetDevs.GetN () << "대 단말의 빔/MAC 초기화 완료." << std::endl;
}

// ================= MAIN FUNCTION =================
int main (int argc, char *argv[])
{
    uint32_t nUes = 40; 
    double simTime = 10.0;

    CommandLine cmd;
    cmd.AddValue ("nUes", "Number of UEs", nUes);
    cmd.Parse (argc, argv);

    // 연구용 설정: HARQ 끄기, Shadowing 끄기
    Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (false)); 
    Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue (false));

    Ptr<ThreeGppAntennaModel> tempAnt = CreateObject<ThreeGppAntennaModel>();
    double hBeamWidth = tempAnt->GetHorizontalBeamwidth(); 
    uint32_t nSectors = static_cast<uint32_t>(ceil(360.0 / hBeamWidth));

    Ptr<MmWaveHelper> mmWaveHelper = CreateObject<MmWaveHelper> ();
    mmWaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));
    mmWaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");

    // 1. 노드 생성 및 모빌리티
    NodeContainer enbNodes; enbNodes.Create (1);
    NodeContainer ueNodes; ueNodes.Create (nUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    mobility.Install (enbNodes);
    
    Ptr<PositionAllocator> posAlloc = CreateObject<UniformDiscPositionAllocator> ();
    posAlloc->SetAttribute ("rho", DoubleValue (50.0)); 
    mobility.SetPositionAllocator (posAlloc);
    mobility.Install (ueNodes);

    // 2. 디바이스 설치
    NetDeviceContainer enbNetDev = mmWaveHelper->InstallEnbDevice (enbNodes);
    NetDeviceContainer ueNetDev = mmWaveHelper->InstallUeDevice (ueNodes);
    mmWaveHelper->AttachToClosestEnb (ueNetDev, enbNetDev);

    // 3. IP 스택 및 주소 할당
    InternetStackHelper internet;
    internet.Install (enbNodes);
    internet.Install (ueNodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase ("10.1.1.0", "255.255.255.0");
    ipv4.Assign (enbNetDev);
    ipv4.Assign (ueNetDev);

    std::cout << "성공: 모든 노드에 IP 스택 및 10.1.1.x 주소 할당 완료." << std::endl;

    // 4. 섹터 및 RNTI 매핑 (초기화)
    g_activeSectors.clear();
    std::set<uint32_t> activeSectorsSet;

    for (uint32_t i = 0; i < nUes; ++i) {
        Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDev.Get (i));
        if (ueDev && ueDev->GetMac ()) {
            uint16_t rnti = ueDev->GetMac ()->GetRnti ();
            g_ueMacMap[rnti] = ueDev->GetMac (); 
            auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNodes.Get (i));
            double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;
            uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / hBeamWidth) % nSectors;
            activeSectorsSet.insert(sectorIdx); 
            NS_LOG_UNCOND("UE RNTI " << rnti << " -> Sector " << sectorIdx);
        }
    }
    for (uint32_t s : activeSectorsSet) g_activeSectors.push_back(s);

    // 5. 라우팅 설정 (기지국)
    Ipv4Address mcastAddr ("224.0.1.1");
    Ptr<Node> enbNode = enbNodes.Get(0);
    Ipv4StaticRoutingHelper staticRoutingHelper;
    Ptr<Ipv4StaticRouting> staticRouting = staticRoutingHelper.GetStaticRouting(enbNode->GetObject<Ipv4>());
    uint32_t mmWaveIfIndex = 1; 
    std::vector<uint32_t> outputInterfaces;
    outputInterfaces.push_back(mmWaveIfIndex);
    
    // 멀티캐스트 라우팅 경로 추가
    staticRouting->AddMulticastRoute(Ipv4Address::GetAny(), mcastAddr, mmWaveIfIndex, outputInterfaces);
    staticRouting->AddMulticastRoute(Ipv4Address::GetAny(), mcastAddr, 0, outputInterfaces); 
    staticRouting->SetDefaultMulticastRoute(mmWaveIfIndex);

    // 6. 라우팅 설정 (단말)
    for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
        Ptr<Ipv4> ueIpv4 = ueNodes.Get (i)->GetObject<Ipv4> (); 
        Ptr<Ipv4StaticRouting> ueStaticRouting = staticRoutingHelper.GetStaticRouting (ueIpv4);
        uint32_t interfaceIndex = 1; 
        for (uint32_t j = 0; j < ueIpv4->GetNInterfaces(); j++) {
            if (ueIpv4->GetAddress(j, 0).GetLocal() != Ipv4Address::GetLoopback()) {
                interfaceIndex = j; break;
            }
        }
        ueStaticRouting->AddMulticastRoute (Ipv4Address::GetAny (), mcastAddr, interfaceIndex, std::vector<uint32_t> ());
    }
    std::cout << "성공: 기지국/단말 멀티캐스트 라우팅 완료." << std::endl;

    // 7. 애플리케이션 설정 (수신 - PacketSink)
    uint16_t dlPort = 1234;
    PacketSinkHelper dlPacketStats ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address ("224.0.1.1"), 1234));    ApplicationContainer serverApps = dlPacketStats.Install (ueNodes);
    serverApps.Start (Seconds (0.1));
    serverApps.Stop (Seconds (simTime));

    // 8. 애플리케이션 설정 (송신 - OnOffHelper + IP 고정)
    // [핵심] OnOffHelper로 IP 강제 고정 (10.1.1.1)
    OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress (mcastAddr, dlPort)));
    onoff.SetConstantRate (DataRate ("960kbps")); 
    onoff.SetAttribute ("PacketSize", UintegerValue (1200));
    // 102.x.x.x 문제 해결을 위한 Local Address 고정
    onoff.SetAttribute ("Local", AddressValue (InetSocketAddress (Ipv4Address ("10.1.1.1"), 0)));

    ApplicationContainer clientApps = onoff.Install (enbNodes.Get (0));
    clientApps.Start (Seconds (3.5)); 
    clientApps.Stop (Seconds (simTime));

    // 9. Trace 연결
    if (clientApps.GetN() > 0) {
        clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&AppTxTrace));
    }
    for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
        ueNodes.Get(i)->GetObject<Ipv4L3Protocol> ()->TraceConnectWithoutContext ("Rx", MakeCallback (&TraceIpReceive));
    }

    // 10. 스케줄링 및 시뮬레이션 실행
    Ptr<MmWaveEnbNetDevice> enbDev = DynamicCast<MmWaveEnbNetDevice> (enbNetDev.Get (0));
    
    // 스케줄러-PHY 연결 (초기 1회)
    Simulator::Schedule (Seconds (0.5), &ConnectSchedulerAndPhy, enbDev, ueNetDev, mmWaveHelper);
    
    // 연구용 빔 사이클 시작
    Simulator::Schedule (Seconds (3.5), &ExecuteResearchCycle, enbDev, enbNodes, ueNodes, hBeamWidth);
    
    // 모니터링 로그 출력 시작
    Simulator::Schedule (Seconds (4.0), &MonitorMulticastReception, enbNodes, ueNodes, hBeamWidth);

    mmWaveHelper->EnableTraces ();
    Simulator::Run ();
    
    std::cout << "--- Research Simulation Result ---" << std::endl;
    std::cout << "Total UEs: " << nUes << std::endl;

    Simulator::Destroy ();
    return 0;
}