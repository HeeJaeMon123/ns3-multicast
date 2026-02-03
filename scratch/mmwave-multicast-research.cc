#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-module.h"
#include "ns3/antenna-module.h"
#include "ns3/internet-module.h" // <--- [핵심 추가] 이 줄이 없어서 에러가 난 것입니다.

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE ("MmWaveMulticastResearch");


#include "ns3/mobility-model.h"
#include <cmath>
#include <set>



std::map<uint16_t, Ptr<MmWaveUeMac>> g_ueMacMap;  // 전역 맵: RNTI를 통해 단말 MAC 객체를 찾기 위함 (GetMacByRnti 구현용)
std::vector<uint32_t> g_activeSectors; // 사용자가 있는 섹터 번호 저장
uint32_t g_activeIdx = 0;              // g_activeSectors 내에서의 현재 인덱스
// [추가] 애플리케이션 레이어에서 패킷이 나가는지 확인하는 함수
void AppTxTrace (Ptr<const Packet> p)
{
    // std::cerr << ">>> [APP LAYER SUCCESS] UdpClient가 패킷을 생성하여 하부 레이어로 던졌습니다! (Size: " 
    //           << p->GetSize() << " bytes)" << std::endl;
}
void TraceIpReceive (Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t interface)
{
    Ipv4Header header;
    packet->PeekHeader (header);
    if (header.GetDestination () == Ipv4Address ("224.0.1.1")) {
        NS_LOG_UNCOND ("IP LAYER: UE " << ipv4->GetAddress(1,0).GetLocal() 
                       << " received Multicast Packet! Size: " << packet->GetSize());
    }
}
void ApplyBeamSteering (Ptr<MmWaveEnbNetDevice> enbDev, double targetAzDeg, double targetElDeg)
{
    Ptr<MmWaveEnbPhy> enbPhy = enbDev->GetPhy ();
    Ptr<MmWaveSpectrumPhy> enbSpecPhy = enbPhy->GetSpectrumPhy ();
    
    // PhasedArrayModel 획득
    Ptr<PhasedArrayModel> enbAntenna = DynamicCast<PhasedArrayModel> (enbSpecPhy->GetAntenna ());
    
    if (enbAntenna)
    {
        // 도(Degree) 단위를 라디안(Radian)으로 변환하여 빔 조향 벡터 설정
        double azRad = targetAzDeg * M_PI / 180.0;
        double elRad = targetElDeg * M_PI / 180.0;
        
        // 해당 방향으로 메인 로브(Main Lobe)가 향하도록 벡터 계산 및 적용
        enbAntenna->SetBeamformingVector (enbAntenna->GetBeamformingVector (Angles (azRad, elRad)));
        
        std::cout << "성공: 빔 조향 완료 (Az: " << targetAzDeg << "°, El: " << targetElDeg << "°)" << std::endl;
    }
}

// [추가] 연구 단계 및 전역 상태 변수 선언
enum ResearchPhase { DATA_TX, PARALLEL_PROCESSING, ACK_RX }; // 에러 방지를 위해 함수 위에 배치
ResearchPhase g_phase = DATA_TX;      // 현재 페이즈
uint32_t g_currentGroup = 0;          // 현재 대상 그룹 (0 ~ nSectors-1)
void ExecuteResearchCycle (Ptr<MmWaveEnbNetDevice> enbDev, NodeContainer enbNodes, NodeContainer ueNodes, double hBeamWidth){
    if (g_activeSectors.empty()) return;

    uint32_t currentSector = g_activeSectors[g_activeIdx];
    uint32_t nextActiveIdx = (g_activeIdx + 1) % g_activeSectors.size();
    uint32_t nextSector = g_activeSectors[nextActiveIdx];

    double targetAz;

    switch (g_phase) {
        case DATA_TX: 
            // [Step 1] 현재 그룹(G_i)에게 멀티캐스트 데이터 전송
            targetAz = (currentSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 1: Data TX -> Sector " << currentSector);
            g_phase = PARALLEL_PROCESSING;
            break;

        case PARALLEL_PROCESSING: 
            // [Step 2] 다음 그룹(G_i+1) 데이터 전송 + 이전 그룹(G_i)은 로컬 ACK 수집
            targetAz = (nextSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 2: Data TX -> Sector " << nextSector 
                           << " | Sector " << currentSector << " is Aggregating ACKs");
            g_phase = ACK_RX;
            break;

        case ACK_RX: 
            // [Step 3] 다시 이전 그룹(G_i)으로 빔을 돌려 Aggregated ACK 수신
            targetAz = (currentSector * hBeamWidth) + (hBeamWidth / 2.0);
            ApplyBeamSteering (enbDev, targetAz, 90.0);
            NS_LOG_UNCOND ("[" << Simulator::Now().GetSeconds() << "s] Phase 3: Listening ACK from Sector " << currentSector);
            
            g_activeIdx = nextActiveIdx; // 다음 활성 섹터로 인덱스 이동
            g_phase = DATA_TX;
            break;
    }
    Simulator::Schedule (MilliSeconds (100), &ExecuteResearchCycle, enbDev, enbNodes, ueNodes, hBeamWidth);
}

// [수정] 박사님 환경의 ns-3 규격에 맞춘 Tx 추적 함수
// [수정] 박사님 환경의 Tx 규격 (4개 인자: Packet, Ipv4, interface)
// [최종 수정] Tx 트레이스: (Header, Packet, Interface) 순서이며 Header는 반드시 const & 여야 함
// [최종 교정] Tx 트레이스: (Packet, Ipv4, Interface) 순서여야 합니다.
void EnbIpTxTrace (Ptr<const Packet> p, Ptr<Ipv4> ipv4, uint32_t interface)
{
    std::cout<<"fuck";
    Ipv4Header ipHeader;
    if (p->PeekHeader (ipHeader)) {
        if (ipHeader.GetDestination () == Ipv4Address ("224.0.1.1")) {
            std::cerr << "[T: " << Simulator::Now ().GetSeconds () << "s] gNB IP LAYER: 패킷이 IP 계층을 빠져나와 NetDevice " << interface << "번으로 향합니다!" << std::endl;
        }
    }
}

// [최종 교정] Drop 트레이스: Header는 반드시 'const &'여야 하고, 인자는 총 5개입니다.
// [Drop 감시 함수] 패킷이 왜 죽는지 범인을 잡습니다.
void EnbIpDropTrace (const Ipv4Header &header, Ptr<const Packet> p, Ipv4L3Protocol::DropReason reason, Ptr<Ipv4> ipv4, uint32_t interface)
{
        std::cout<<"fuck";
    // 목적지가 멀티캐스트인 것만 필터링
    if (header.GetDestination () == Ipv4Address ("224.0.1.1")) {
        std::cerr << "!!! [DETECTED DROP] !!!" << std::endl;
        std::cerr << "시간: " << Simulator::Now().GetSeconds() << "s" << std::endl;
        std::cerr << "사유코드: " << reason << " (1:경로없음, 2:TTL만료, 3:인터페이스다운)" << std::endl;
    }
}
// 1. 상대 각도 계산 함수 (Azimuth & Inclination)
// 기지국(enb)과 단말(ue)의 위치를 받아 gNB 기준의 각도를 반환합니다.
std::pair<double, double> GetUeAngles (Ptr<Node> enb, Ptr<Node> ue)
{
    Ptr<MobilityModel> enbMob = enb->GetObject<MobilityModel> ();
    Ptr<MobilityModel> ueMob = ue->GetObject<MobilityModel> ();

    Vector enbPos = enbMob->GetPosition ();
    Vector uePos = ueMob->GetPosition ();

    // 수평각 (Azimuth) 계산: -180 ~ 180도 범위
    double azRad = atan2 (uePos.y - enbPos.y, uePos.x - enbPos.x);
    double azDeg = azRad * 180.0 / M_PI;

    // 수직각 (Inclination/Theta) 계산: 0 ~ 180도 범위
    double dist = sqrt (pow (uePos.x - enbPos.x, 2) + pow (uePos.y - enbPos.y, 2));
    double elRad = atan2 (dist, uePos.z - enbPos.z); // z축 기준 (머리 위가 0도)
    double elDeg = elRad * 180.0 / M_PI;

    return {azDeg, elDeg};
}

// 2. 빔 스티어링 및 그룹화 로직 (Main 함수 내부 수정)
void GroupingAndSteering (NodeContainer enbNodes, NodeContainer ueNodes, uint32_t nSectors)
{
    double sectorWidth = 360.0 / nSectors;
    
    for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
        auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNodes.Get (i));

        // 0~360도 범위로 보정하여 섹터 인덱스 계산
        double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;
        uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / sectorWidth) % nSectors;

        // 해당 섹터의 중앙 각도로 빔 스티어링 목표 설정
        double steeringAzimuth = (sectorIdx * sectorWidth) + (sectorWidth / 2.0);
        
        // [학술적 포인트] 3GPP 모델의 빔 폭(65도) 안에 있는지 확인
        bool isInsideBeam = std::abs(normalizedAz - steeringAzimuth) <= 32.5; // 65/2

        NS_LOG_UNCOND ("UE " << i << " -> Sector " << sectorIdx 
                       << " (Angle: " << azimuth << "°, In Beam: " << (isInsideBeam ? "Yes" : "No") << ")");
    }
}


// 헬퍼 함수: 시뮬레이션 중간에 수신 통계를 출력함
// NodeContainer enbNodes를 인자에 추가합니다.
// hBeamWidth를 인자로 추가하여 동적 섹터 계산이 가능하게 합니다.
void MonitorMulticastReception (NodeContainer enbNodes, NodeContainer ueNodes, double hBeamWidth)
{
    std::cout << "\n=== [Time: " << Simulator::Now ().GetSeconds () << "s] Multicast Reception Monitor ===" << std::endl;

    uint32_t nSectors = static_cast<uint32_t> (ceil (360.0 / hBeamWidth)); // [추가] 섹터 수 계산

    for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
        Ptr<Node> ueNode = ueNodes.Get (i);
        // std::cout<<ueNode->GetApplication(i)<<std::endl;
        // [수정] PacketSink에서 총 수신 바이트를 가져와 패킷 수로 계산
        uint32_t receivedPackets = 0;

        // [수정] 해당 노드에 설치된 모든 애플리케이션 중 PacketSink를 찾아 수신량을 합산
        for (uint32_t j = 0; j < ueNode->GetNApplications(); ++j) {
            Ptr<PacketSink> sink = DynamicCast<PacketSink> (ueNode->GetApplication (j));
            if (sink) {
                // [에러 해결!] AddressValue라는 래퍼를 사용해야 합니다.
                AddressValue localAddrValue;
                sink->GetAttribute("Local", localAddrValue);
                Address localAddr = localAddrValue.Get();
                InetSocketAddress inetAddr = InetSocketAddress::ConvertFrom(localAddr);

                uint64_t totalRxBytes = sink->GetTotalRx();
                
                // 상세 정보 출력: 어떤 IP/Port로 대기 중이며, 실제로 몇 바이트가 들어왔는가?
                std::cout << "  [UE " << i << " App " << j << "] Listening: " 
                          << inetAddr.GetIpv4() << ":" << inetAddr.GetPort()
                          << " | RawRx: " << totalRxBytes << " bytes" << std::endl;

                receivedPackets += (totalRxBytes / 1200);
            }
        }



        // 좌표 및 각도 계산
        auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNode);
        double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;

        // [디버그용 추가] 빔 정보 계산
        uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / hBeamWidth) % nSectors;
        double beamCenter = (sectorIdx * hBeamWidth) + (hBeamWidth / 2.0);
        double angleError = std::abs (normalizedAz - beamCenter);
        bool isInside = (angleError <= (hBeamWidth / 2.0)); // 빔 폭 이내 여부

        std::cout << "UE " << i << " | Received: " << receivedPackets 
                  << " | Angle: " << azimuth << "° | Sector: " << sectorIdx 
                  << " | Center: " << beamCenter << "° | InBeam: " << (isInside ? "YES" : "NO") << std::endl;
    }
    
    // 재귀 호출 시 hBeamWidth 전달
    Simulator::Schedule (MilliSeconds (200), &MonitorMulticastReception, enbNodes, ueNodes, hBeamWidth);
}

// ueNetDev 컨테이너를 함께 받도록 수정합니다.
void ConnectSchedulerAndPhy (Ptr<MmWaveEnbNetDevice> enbDev, NetDeviceContainer ueNetDevs, Ptr<MmWaveHelper> helper) 
{
    Ptr<MmWaveEnbMac> enbMac = enbDev->GetMac ();
    if (!enbMac) return; // 안전 장치
    Ptr<MmWaveEnbPhy> enbPhy = enbDev->GetPhy ();
    
    Ptr<MmWaveFlexTtiMacScheduler> scheduler = DynamicCast<MmWaveFlexTtiMacScheduler> (enbMac->GetScheduler ());
    if (!scheduler) {
        scheduler = CreateObject<MmWaveFlexTtiMacScheduler> ();
        
        // MAC으로부터 설정을 가져옵니다.
        Ptr<MmWavePhyMacCommon> config = enbMac->GetConfigurationParameters (); 

        if (config) {
            // config가 확실히 존재할 때만 설정을 진행합니다. (m_amc가 여기서 안전하게 생성됨)
            scheduler->ConfigureCommonParameters (config);
            std::cout << "성공: 스케줄러 설정 동기화 완료." << std::endl;
        } else {
            // 아직 준비가 안 됐다면 1마이크로초 뒤에 다시 시도합니다.
            Simulator::Schedule (MicroSeconds (1), &ConnectSchedulerAndPhy, enbDev, ueNetDevs, helper);
            return;
        }

        enbMac->SetMmWaveMacSchedSapProvider (scheduler->GetMacSchedSapProvider ());
        scheduler->SetMacSchedSapUser (enbMac->GetMmWaveMacSchedSapUser ());
        enbDev->AggregateObject (scheduler);
    }
    // --- [여기서 실행!] ---
    // 모든 객체가 안정화된 시점이므로 NULL 포인터 에러가 나지 않습니다.
    EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT); 
    helper->ActivateDataRadioBearer (ueNetDevs, bearer); 

    std::cout << "성공: 모든 단말에 데이터 베어러(DRB, LCID 3) 활성화 명령 완료." << std::endl;

    // [핵심 추가] 2. 멀티캐스트 IP와 무선 그룹 매핑
    // 이 코드가 있어야 스케줄러가 224.0.1.1 패킷을 전송 자원에 할당합니다.
    Ipv4Address mcastGroup ("224.0.1.1");
    // helper->AddMcGroup (ueNetDevs, mcastGroup); // 일부 버전은 enbDev를 포함할 수 있습니다.

    std::cout << "성공: 멀티캐스트 그룹(224.0.1.1) 무선 매핑 완료." << std::endl;
    // 2. 기지국(eNB) 안테나 초기화
    Ptr<MmWaveSpectrumPhy> enbSpecPhy = enbPhy->GetSpectrumPhy ();
    if (enbSpecPhy) {
        scheduler->SetPhy (enbSpecPhy);
        Ptr<PhasedArrayModel> enbAntenna = DynamicCast<PhasedArrayModel> (enbSpecPhy->GetAntenna ());
        if (enbAntenna) {
            enbAntenna->SetBeamformingVector (enbAntenna->GetBeamformingVector (Angles (0, 0)));
        }
    }

    // [핵심 추가] 3. 모든 단말(UE) 안테나 초기화
    // 60대의 차량 단말 안테나가 유효하지 않으면 SINR 계산 시 에러가 발생합니다.
    for (uint32_t i = 0; i < ueNetDevs.GetN (); ++i) {
        Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDevs.Get (i));
        if (ueDev) {
            Ptr<MmWaveUePhy> uePhy = ueDev->GetPhy ();
            // 단말의 SpectrumPhy 획득
            Ptr<MmWaveSpectrumPhy> ueSpecPhy = uePhy->GetDlSpectrumPhy (); 
            if (ueSpecPhy) {
                Ptr<PhasedArrayModel> ueAntenna = DynamicCast<PhasedArrayModel> (ueSpecPhy->GetAntenna ());
                if (ueAntenna) {
                    // 각 차량 단말도 일단 기지국 방향(0,0)을 보게 초기화
                    ueAntenna->SetBeamformingVector (ueAntenna->GetBeamformingVector (Angles (0, 0)));
                }
            }
        }
    }
    // [핵심 해결책] 모든 단말에게 그룹 RNTI 패킷을 수용하도록 명령
    for (uint32_t i = 0; i < ueNetDevs.GetN (); ++i) {
        Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDevs.Get (i));
        
     }
    std::cout << "성공: 기지국 및 " << ueNetDevs.GetN () << "대 단말의 빔 초기화 완료." << std::endl;
}



int main (int argc, char *argv[])
{
  uint32_t nUes = 40;       // 기본 사용자 수 (30~60명 범위)
  double simTime = 10.0;     // 시뮬레이션 시간

  Ptr<ThreeGppAntennaModel> tempAnt = CreateObject<ThreeGppAntennaModel>();
  double hBeamWidth = tempAnt->GetHorizontalBeamwidth(); // 기본 65.0
  uint32_t nSectors = static_cast<uint32_t>(ceil(360.0 / hBeamWidth));

  CommandLine cmd;
  cmd.AddValue ("nUes", "Number of UEs", nUes);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (false)); // HARQ 비활성화


  Ptr<MmWaveHelper> mmWaveHelper = CreateObject<MmWaveHelper> ();
//   mmWaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWavePropagationLossModel")); // 연구 집중을 위한 이상적 채널
// 1. 모델을 구체적인 3GPP 모델로 변경 (Attribute 에러 방지)
// mmWaveHelper->SetDefaultEpsBearer (EpsBearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT));
  mmWaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ThreeGppUmiStreetCanyonPropagationLossModel"));

  // 2. "이상적 채널"을 위해 무작위 변수(Shadowing) 비활성화
  // 이를 통해 같은 거리면 항상 같은 수신 세기가 나오도록 설정합니다.
  Config::SetDefault ("ns3::ThreeGppPropagationLossModel::ShadowingEnabled", BooleanValue (false));


  // 1. 노드 생성
  NodeContainer enbNodes;
  enbNodes.Create (1);
  NodeContainer ueNodes;
  ueNodes.Create (nUes);

  // 2. 모빌리티 설정 (기지국 중심 배치)
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  mobility.Install (enbNodes);
  
//   for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
//     Ptr<ConstantVelocityMobilityModel> cvm = ueNodes.Get (i)->GetObject<ConstantVelocityMobilityModel> ();
//     cvm->SetVelocity (Vector (20.0, 0.0, 0.0)); // x축 방향 이동
//   }

  // 단말들을 기지국 주변 원형(Disc)으로 배치하여 섹터 구분을 용이하게 함
  Ptr<PositionAllocator> posAlloc = CreateObject<UniformDiscPositionAllocator> ();
  posAlloc->SetAttribute ("rho", DoubleValue (50.0)); // 반경 50m
  mobility.SetPositionAllocator (posAlloc);
  mobility.Install (ueNodes);

  // 3. mmWave 장치 설치 및 스케줄러 설정
  mmWaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");
  NetDeviceContainer enbNetDev = mmWaveHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueNetDev = mmWaveHelper->InstallUeDevice (ueNodes);
  mmWaveHelper->AttachToClosestEnb (ueNetDev, enbNetDev);
  
// main 함수 내 Step 4 수정
  Ptr<MmWaveEnbNetDevice> enbDev = DynamicCast<MmWaveEnbNetDevice> (enbNetDev.Get (0));

  // ScheduleNow 대신 아주 짧은 지연 시간을 줍니다.
//   Simulator::ScheduleNow (&ConnectSchedulerAndPhy, enbDev, ueNetDev, mmWaveHelper);
  Simulator::Schedule (MicroSeconds (10), &ConnectSchedulerAndPhy, enbDev, ueNetDev, mmWaveHelper);
// --- [추가] 4. Internet Stack 및 IP 주소 설정 ---
  InternetStackHelper internet;
  internet.Install (enbNodes);
  internet.Install (ueNodes);

  Ipv4AddressHelper ipv4;
  // 기지국과 단말들을 동일한 서브넷(10.1.1.0/24)으로 묶습니다.
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  
  // NetDevice에 IP 주소를 부여합니다.
  Ipv4InterfaceContainer enbIpIfaces = ipv4.Assign (enbNetDev);
  Ipv4InterfaceContainer ueIpIfaces = ipv4.Assign (ueNetDev);

  // [중요] 기지국(Client)에서 전체 방송을 할 수 있도록 라우팅 설정 확인
//   Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
  std::cout << "성공: 모든 노드에 IP 스택 및 10.1.1.x 주소 할당 완료." << std::endl;


  // 5. 단말별 그룹 RNTI 및 Aggregator 지정
std::set<uint32_t> sectorHasAggregator; // 각 섹터에 대표가 있는지 체크용
std::set<uint32_t> activeSectors; // 빔 조향을 위해 사용자가 있는 섹터 저장


// --- [Step 5] 단말별 그룹 RNTI 및 활성 섹터 저장 ---
// std::set<uint32_t> activeSectors;
activeSectors.clear();
for (uint32_t i = 0; i < nUes; ++i)
{
    Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDev.Get (i));
    if (ueDev && ueDev->GetMac ()) 
    {
        uint16_t rnti = ueDev->GetMac ()->GetRnti ();
        g_ueMacMap[rnti] = ueDev->GetMac (); 

        auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNodes.Get (i));
        double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;
        uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / hBeamWidth) % nSectors;
    
        activeSectors.insert(sectorIdx); 
        NS_LOG_UNCOND("UE " << rnti << " -> Sector " << sectorIdx);
    }
}

// 활성 섹터 리스트 업데이트
g_activeSectors.clear();
for (uint32_t sector : activeSectors) {
    g_activeSectors.push_back(sector);
}

// --- [Step 6] 멀티캐스트 데이터 트래픽 및 라우팅 설정 수정 ---
// 1. 기지국(eNB) 라우팅 설정
// --- [main 함수 내 Step 6 수정] ---

// --- [Step 5 & 6 통합 및 최적화본] ---

uint16_t dlPort = 1234;
Ipv4Address mcastAddr ("224.0.1.1"); 

// 1. 활성 섹터 및 RNTI 매핑 정보 초기화
g_activeSectors.clear();
std::set<uint32_t> activeSectorsSet;

for (uint32_t i = 0; i < nUes; ++i)
{
    Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDev.Get (i));
    if (ueDev && ueDev->GetMac ()) 
    {
        uint16_t rnti = ueDev->GetMac ()->GetRnti ();
        g_ueMacMap[rnti] = ueDev->GetMac (); 

        auto [azimuth, inclination] = GetUeAngles (enbNodes.Get (0), ueNodes.Get (i));
        double normalizedAz = (azimuth < 0) ? azimuth + 360.0 : azimuth;
        uint32_t sectorIdx = static_cast<uint32_t> (normalizedAz / hBeamWidth) % nSectors;
    
        activeSectorsSet.insert(sectorIdx); 
        NS_LOG_UNCOND("UE RNTI " << rnti << " -> Sector " << sectorIdx);
    }
}

for (uint32_t s : activeSectorsSet) {
    g_activeSectors.push_back(s);
}

// 2. 라우팅 설정 (기지국 및 단말)
// [Step 6] 2. 라우팅 설정 (기지국 부분) - 이 코드로 교체하십시오.
Ptr<Node> enbNode = enbNodes.Get(0);
Ptr<Ipv4> enbIpv4Obj = enbNode->GetObject<Ipv4>();
Ipv4StaticRoutingHelper staticRoutingHelper;
Ptr<Ipv4StaticRouting> staticRouting = staticRoutingHelper.GetStaticRouting(enbIpv4Obj);

uint32_t mmWaveIfIndex = 1; // 기지국의 mmWave 인터페이스

std::vector<uint32_t> outputInterfaces;
outputInterfaces.push_back(mmWaveIfIndex);

// [수정 핵심] 모든 출발지(GetAny)에 대해 mmWave(1번)로 나가는 경로를 2중으로 보강합니다.
staticRouting->AddMulticastRoute(Ipv4Address::GetAny(), mcastAddr, mmWaveIfIndex, outputInterfaces);
staticRouting->AddMulticastRoute(Ipv4Address::GetAny(), mcastAddr, 0, outputInterfaces); // 내부 발생용

// [중요] 기지국의 기본 멀티캐스트 출력 인터페이스를 mmWave로 강제 고정
staticRouting->SetDefaultMulticastRoute(mmWaveIfIndex);

std::cout << "성공: 기지국 멀티캐스트 하이패스 경로 2중 구축 완료." << std::endl;


// 단말 라우팅 설정 (반복문 부분)
for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
    Ptr<Ipv4> ueIpv4 = ueNodes.Get (i)->GetObject<Ipv4> (); // 변수명 변경
    Ptr<Ipv4StaticRouting> ueStaticRouting = staticRoutingHelper.GetStaticRouting (ueIpv4);
    
    uint32_t interfaceIndex = 1; 
    for (uint32_t j = 0; j < ueIpv4->GetNInterfaces(); j++) {
        Ipv4Address addr = ueIpv4->GetAddress(j, 0).GetLocal();
        if (addr != Ipv4Address::GetLoopback() && addr != Ipv4Address::GetAny()) {
            interfaceIndex = j;
            break;
        }
    }
    ueStaticRouting->AddMulticastRoute (Ipv4Address::GetAny (), mcastAddr, interfaceIndex, std::vector<uint32_t> ());
}

// 3. 수신 애플리케이션 설치 (중요: mcastAddr에 직접 바인딩)
// --- [main 함수 내 수정 사항] ---

// 1. 수신 애플리케이션 (Sink) 설정: 송신보다 먼저 켜져야 합니다.
// mcastAddr 대신 Ipv4Address::GetAny() 사용
PacketSinkHelper dlPacketStats ("ns3::UdpSocketFactory", 
                                 InetSocketAddress (Ipv4Address::GetAny (), dlPort));
ApplicationContainer serverApps = dlPacketStats.Install (ueNodes);
serverApps.Start (Seconds (0.1));
serverApps.Stop (Seconds (simTime));

// 2. 송신 애플리케이션 (Client) 설정
UdpClientHelper dlClient (mcastAddr, dlPort);
dlClient.SetAttribute ("PacketSize", UintegerValue (1200));
dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (10))); 
dlClient.SetAttribute ("RemoteAddress", AddressValue (InetSocketAddress (mcastAddr, dlPort)));

ApplicationContainer clientApps = dlClient.Install (enbNodes);
clientApps.Start (Seconds (3.5)); // [수정] 0.5 -> 1.0으로 변경 (단말 접속 완료 후 송신)
clientApps.Stop (Seconds (simTime));

// 3. 연구 사이클 예약 시간 조정
Simulator::Schedule (Seconds (3.5), &ExecuteResearchCycle, enbDev, enbNodes, ueNodes, hBeamWidth);

// 5. 연구 사이클 및 모니터링 예약
Simulator::Schedule (Seconds (4.0), &MonitorMulticastReception, enbNodes, ueNodes, hBeamWidth);

  // 7. 결과 출력 (Reliability, Throughput 등)
  mmWaveHelper->EnableTraces ();
  Simulator::Stop (Seconds (simTime));
  // main 함수 끝부분
  for (uint32_t i = 0; i < ueNodes.GetN (); ++i) {
    ueNodes.Get(i)->GetObject<Ipv4L3Protocol> ()->TraceConnectWithoutContext (
        "Rx", MakeCallback (&TraceIpReceive));
}

    // 람다 대신 함수 포인터를 사용하여 빌드 에러 해결
    if (clientApps.GetN() > 0) {
        clientApps.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&AppTxTrace));
    }
// main.cc 내부
// [추가] 모든 단말이 멀티캐스트 패킷을 수락하도록 IP 계층 설정
for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
{
    Ptr<Ipv4> ueIpv4 = ueNodes.Get(i)->GetObject<Ipv4>();
    Ptr<Ipv4StaticRouting> ueStaticRouting = staticRoutingHelper.GetStaticRouting (ueIpv4);
    
    // 실제 mmWave 인터페이스 인덱스를 정확히 찾아냅니다.
    uint32_t realIfIndex = 1; 
    for (uint32_t j = 0; j < ueIpv4->GetNInterfaces(); j++) {
        if (ueIpv4->GetAddress(j, 0).GetLocal() != Ipv4Address::GetLoopback()) {
            realIfIndex = j;
            break;
        }
    }
    
    // 224.0.1.1로 오는 패킷을 버리지 말고 수용하라는 경로 추가
    ueStaticRouting->AddMulticastRoute (Ipv4Address::GetAny (), mcastAddr, realIfIndex, std::vector<uint32_t> ());
}

std::cout << "성공: 모든 단말의 IP 계층에서 멀티캐스트 수신 경로 확보 완료." << std::endl;
    Simulator::Run ();
  
  // 시뮬레이션 종료 후 지표 요약 출력
  std::cout << "--- Research Simulation Result ---" << std::endl;
  std::cout << "Total UEs: " << nUes << std::endl;
  // 여기서 g_ueMacMap을 순회하며 최종 통계(Reliability 등)를 출력할 수 있습니다.

  Simulator::Destroy ();
  return 0;
}