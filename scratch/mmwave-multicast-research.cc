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

// 전역 맵: RNTI를 통해 단말 MAC 객체를 찾기 위함 (GetMacByRnti 구현용)
std::map<uint16_t, Ptr<MmWaveUeMac>> g_ueMacMap;

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
    std::cout << "성공: 데이터 베어러(Bearer) 활성화 완료 ($10us)" << std::endl;
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
    std::cout << "성공: 기지국 및 " << ueNetDevs.GetN () << "대 단말의 빔 초기화 완료." << std::endl;
}



int main (int argc, char *argv[])
{
  uint32_t nUes = 40;       // 기본 사용자 수 (30~60명 범위)
  double simTime = 1.0;     // 시뮬레이션 시간
  uint32_t nSectors = 6;    // 6개 섹터 (60도씩 분할)
  
  CommandLine cmd;
  cmd.AddValue ("nUes", "Number of UEs", nUes);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (false)); // HARQ 비활성화


  Ptr<MmWaveHelper> mmWaveHelper = CreateObject<MmWaveHelper> ();
//   mmWaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::MmWavePropagationLossModel")); // 연구 집중을 위한 이상적 채널
// 1. 모델을 구체적인 3GPP 모델로 변경 (Attribute 에러 방지)
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
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enbNodes);
  
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
  for (uint32_t i = 0; i < nUes; ++i)
  {
    Ptr<MmWaveUeNetDevice> ueDev = DynamicCast<MmWaveUeNetDevice> (ueNetDev.Get (i));
    if (ueDev && ueDev->GetMac ()) // [수정] 둘 다 존재할 때만 실행
    {
        uint16_t rnti = ueDev->GetMac ()->GetRnti ();
        g_ueMacMap[rnti] = ueDev->GetMac (); // 전역 맵에 등록

        // 섹터별 배치 로직 (간략화: RNTI 순서대로 섹터 할당)
        uint32_t assignedSector = i % nSectors;
        uint16_t groupRnti = 1001 + assignedSector;
        NS_LOG_INFO("UE " << rnti << " assigned to Group " << groupRnti);
        // 대표 노드 선정 (각 섹터의 첫 번째 노드)
        if (i < nSectors) {
            // ueDev->GetMac ()->SetAsAggregator (true); // Aggregator 역할 부여
        }
    }
  }

  // 6. 멀티캐스트 데이터 트래픽 설정 (UDP)
  uint16_t dlPort = 1234;
  
  // 모든 단말에 서버 설치 (기존 코드 유지)
  UdpServerHelper dlPacketStats (dlPort);
  ApplicationContainer serverApps = dlPacketStats.Install (ueNodes);
  serverApps.Start (Seconds (0.1));

  // [핵심 수정] 목적지 주소를 브로드캐스트 대신 첫 번째 단말(UE 0)의 IP로 설정
  // 이렇게 하면 RRC가 "아, UE 0에게 가는 데이터구나" 하고 정상적으로 태그를 붙여줍니다.
  // 1. 목적지 설정 (RRC 태그 생성을 위한 '가짜 티켓')
  Ipv4Address ue0Ip = ueIpIfaces.GetAddress (0); 
  UdpClientHelper dlClient (ue0Ip, dlPort); 

  // 2. 속성 설정 (반드시 Install 전에 수행해야 합니다)
  uint32_t payloadSize = 1200; 
  dlClient.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
  // 100 마이크로초 간격으로 1200바이트를 계속 쏩니다.
  dlClient.SetAttribute ("Interval", TimeValue (MicroSeconds (100))); 

  // 3. 애플리케이션 설치
  ApplicationContainer clientApps = dlClient.Install (enbNodes);

  // 4. 실행 시간 설정 (RRC 연결이 완료된 충분한 시점)
//   serverApps.Start (Seconds (0.1));   // 서버 먼저 대기
  clientApps.Start (Seconds (1.0));   // 0.5초에 클라이언트 전송 시작
  clientApps.Stop (Seconds (simTime));

  std::cout << "성공: RRC 태그 생성을 위한 데이터 베어러 활성화 완료." << std::endl;

  // 7. 결과 출력 (Reliability, Throughput 등)
  mmWaveHelper->EnableTraces ();

  Simulator::Stop (Seconds (simTime));

  Simulator::Run ();
  std::cout << "1!" << std::endl;
  
  // 시뮬레이션 종료 후 지표 요약 출력
  std::cout << "--- Research Simulation Result ---" << std::endl;
  std::cout << "Total UEs: " << nUes << std::endl;
  // 여기서 g_ueMacMap을 순회하며 최종 통계(Reliability 등)를 출력할 수 있습니다.

  Simulator::Destroy ();
  return 0;
}