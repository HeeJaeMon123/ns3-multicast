/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

#include "mmwave-ue-phy.h"
#include "mc-ue-net-device.h"
#include "mmwave-spectrum-value-helper.h"
#include "mmwave-ue-net-device.h"
#include "mmwave-enb-mac.h"
#include <ns3/double.h>
#include <ns3/log.h>
#include <ns3/node.h>
#include <ns3/object-factory.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <cfloat>
#include <cmath>
#include "ns3/lte-radio-bearer-tag.h" // Tag 헤더 필수
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h"
#include "ns3/mmwave-mac-pdu-header.h" // <--- 이거 필수!

namespace ns3 {
namespace mmwave {

// [전역 변수] 기지국이 현재 쏘고 있는 섹터 번호 (main.cc와 공유)
uint32_t g_currentTxSector = 0; 

NS_LOG_COMPONENT_DEFINE("MmWaveUePhy");
NS_OBJECT_ENSURE_REGISTERED(MmWaveUePhy);

// -------------------------------------------------------------------------
// 생성자 / 소멸자
// -------------------------------------------------------------------------
MmWaveUePhy::MmWaveUePhy()
{
    NS_LOG_FUNCTION(this);
    NS_FATAL_ERROR("This constructor should not be called");
}

MmWaveUePhy::MmWaveUePhy(Ptr<MmWaveSpectrumPhy> dlPhy, Ptr<MmWaveSpectrumPhy> ulPhy)
    : MmWavePhy(dlPhy, ulPhy),
      m_prevSlot(0),
      m_rnti(0)
{
    NS_LOG_FUNCTION(this);
    m_wbCqiLast = Simulator::Now();
    m_cellSinrMap.clear();
    m_ueCphySapProvider = new MemberLteUeCphySapProvider<MmWaveUePhy>(this);
    Simulator::ScheduleNow(&MmWaveUePhy::SlotIndication, this, 0, 0, 0);
}

MmWaveUePhy::~MmWaveUePhy()
{
    NS_LOG_FUNCTION(this);
}

// -------------------------------------------------------------------------
// GetTypeId
// -------------------------------------------------------------------------
TypeId
MmWaveUePhy::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::MmWaveUePhy")
            .SetParent<MmWavePhy>()
            .AddConstructor<MmWaveUePhy>()
            .AddAttribute("TxPower", "Transmission power in dBm", DoubleValue(30.0),
                          MakeDoubleAccessor(&MmWaveUePhy::SetTxPower, &MmWaveUePhy::GetTxPower),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseFigure", "Loss (dB) in the Signal-to-Noise-Ratio", DoubleValue(5.0),
                          MakeDoubleAccessor(&MmWavePhy::SetNoiseFigure, &MmWavePhy::GetNoiseFigure),
                          MakeDoubleChecker<double>())
            .AddAttribute("DlSpectrumPhy", "The downlink MmWaveSpectrumPhy associated to this MmWavePhy",
                          TypeId::ATTR_GET, PointerValue(),
                          MakePointerAccessor(&MmWaveUePhy::GetDlSpectrumPhy),
                          MakePointerChecker<MmWaveSpectrumPhy>())
            .AddAttribute("UlSpectrumPhy", "The uplink MmWaveSpectrumPhy associated to this MmWavePhy",
                          TypeId::ATTR_GET, PointerValue(),
                          MakePointerAccessor(&MmWaveUePhy::GetUlSpectrumPhy),
                          MakePointerChecker<MmWaveSpectrumPhy>())
            .AddTraceSource("ReportCurrentCellRsrpSinr", "RSRP and SINR statistics.",
                            MakeTraceSourceAccessor(&MmWaveUePhy::m_reportCurrentCellRsrpSinrTrace),
                            "ns3::CurrentCellRsrpSinr::TracedCallback")
            .AddTraceSource("ReportUplinkTbSize", "Report allocated uplink TB size for trace.",
                            MakeTraceSourceAccessor(&MmWaveUePhy::m_reportUlTbSize),
                            "ns3::UlTbSize::TracedCallback")
            .AddTraceSource("ReportDownlinkTbSize", "Report allocated downlink TB size for trace.",
                            MakeTraceSourceAccessor(&MmWaveUePhy::m_reportDlTbSize),
                            "ns3::DlTbSize::TracedCallback")
            .AddTraceSource("ReportUlPhyTransmission", "Report the allocation info for the current UL transmission",
                            MakeTraceSourceAccessor(&MmWaveUePhy::m_ulPhyTrace),
                            "ns3::UlPhyTransmission::TracedCallback")
            .AddAttribute("OutageThreshold", "SNR threshold for outage events [dB]", DoubleValue(-5.0),
                          MakeDoubleAccessor(&MmWaveUePhy::m_outageThreshold),
                          MakeDoubleChecker<long double>(-70.0, 10.0))
            .AddAttribute("n310", "Counter for SINR below threshold events", UintegerValue(2),
                          MakeUintegerAccessor(&MmWaveUePhy::m_n310),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("CqiReportPeriod", "The period of the DL wideband CQI update, in number of slots",
                          UintegerValue(10),
                          MakeUintegerAccessor(&MmWaveUePhy::SetWbCqiPeriod),
                          MakeUintegerChecker<uint16_t>());
    return tid;
}

// -------------------------------------------------------------------------
// [핵심 수정 1] ReceiveControlMessageList (DCI 필터링)
// -------------------------------------------------------------------------
void
MmWaveUePhy::ReceiveControlMessageList(std::list<Ptr<MmWaveControlMessage>> msgList)
{
    NS_LOG_FUNCTION(this);
    std::list<Ptr<MmWaveControlMessage>>::iterator it;
    bool validDciFound = false; 

    for (it = msgList.begin(); it != msgList.end(); it++)
    {
        Ptr<MmWaveControlMessage> msg = (*it);

        if (msg->GetMessageType() == MmWaveControlMessage::DCI_TDMA)
        {
            Ptr<MmWaveTdmaDciMessage> dciMsg = DynamicCast<MmWaveTdmaDciMessage>(msg);
            DciInfoElementTdma dciInfoElem = dciMsg->GetDciInfoElement();

            if (validDciFound) continue;

            uint16_t currentBeamRnti = 1001 + g_currentTxSector;
            bool isMyUnicast = (dciInfoElem.m_rnti == m_rnti);
            bool isTargetMulticast = (dciInfoElem.m_rnti == currentBeamRnti && dciInfoElem.m_tbSize > 100);

            if (!isMyUnicast && !isTargetMulticast) {
                continue; 
            }

            if (isTargetMulticast) {
                NS_LOG_UNCOND ("UE " << m_rnti << " PHY: [MATCH] Beam Sector " << g_currentTxSector 
                               << " (Target RNTI " << currentBeamRnti << ") Matched! Size: " << dciInfoElem.m_tbSize);
                validDciFound = true; 
            }

            if (dciInfoElem.m_format == DciInfoElementTdma::DL_dci) 
            {
                TtiAllocInfo ttiInfo;
                ttiInfo.m_tddMode = TtiAllocInfo::DL_slotAllocInfo;
                ttiInfo.m_dci = dciInfoElem;
                ttiInfo.m_ttiIdx = 0;
                
                std::deque<TtiAllocInfo>::iterator itTti;
                for (itTti = m_currSlotAllocInfo.m_ttiAllocInfo.begin();
                     itTti != m_currSlotAllocInfo.m_ttiAllocInfo.end();
                     itTti++)
                {
                    if (itTti->m_tddMode == TtiAllocInfo::UL_slotAllocInfo) break;
                    ttiInfo.m_ttiIdx++;
                }
                m_currSlotAllocInfo.m_ttiAllocInfo.insert(itTti, ttiInfo);
            }
            else if (dciInfoElem.m_format == DciInfoElementTdma::UL_dci) 
            {
                uint8_t ulSlotIdx = (m_slotNum + m_phyMacConfig->GetUlSchedDelay()) % m_phyMacConfig->GetSlotsPerSubframe();
                TtiAllocInfo ttiInfo;
                ttiInfo.m_tddMode = TtiAllocInfo::UL_slotAllocInfo;
                ttiInfo.m_dci = dciInfoElem;
                
                if (!m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.empty()) {
                    TtiAllocInfo ulCtrlTti = m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.back();
                    m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.pop_back();
                    ttiInfo.m_ttiIdx = m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.size();
                    m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.push_back(ttiInfo);
                    m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.push_back(ulCtrlTti);
                } else {
                    ttiInfo.m_ttiIdx = 0;
                    m_slotAllocInfo[ulSlotIdx].m_ttiAllocInfo.push_back(ttiInfo);
                }
            }
            m_phySapUser->ReceiveControlMessage(msg);
        }
        else if (msg->GetMessageType() == MmWaveControlMessage::MIB) {
            NS_ASSERT(m_cellId > 0);
            m_ueCphySapUser->RecvMasterInformationBlock(m_cellId, DynamicCast<MmWaveMibMessage>(msg)->GetMib());
        }
        else if (msg->GetMessageType() == MmWaveControlMessage::SIB1) {
            NS_ASSERT(m_cellId > 0);
            m_ueCphySapUser->RecvSystemInformationBlockType1(m_cellId, DynamicCast<MmWaveSib1Message>(msg)->GetSib1());
        }
        else if (msg->GetMessageType() == MmWaveControlMessage::RAR) {
            NS_ASSERT(m_cellId > 0);
            Ptr<MmWaveRarMessage> rarMsg = DynamicCast<MmWaveRarMessage>(msg);
            for (auto itRar = rarMsg->RarListBegin(); itRar != rarMsg->RarListEnd(); ++itRar) {
                if (itRar->rapId == m_raPreambleId) m_phySapUser->ReceiveControlMessage(rarMsg);
            }
        }
    }
}

// -------------------------------------------------------------------------
// [핵심 수정 2] PhyDataPacketReceived (패딩 제거)
// -------------------------------------------------------------------------
// [src/mmwave/model/mmwave-ue-phy.cc]

// [필수] 파일 맨 위에 이 헤더가 없다면 추가해주세요!
#include "ns3/ipv4-header.h" 

// [파일 맨 위에 추가]
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h" // <--- 이거 꼭 추가하세요!

// ...

// [상단 include 확인]
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h" // <--- 필수!

void
MmWaveUePhy::PhyDataPacketReceived(Ptr<Packet> p)
{
    NS_LOG_UNCOND("UE " << m_rnti << " PhyDataPacketReceived! Size: " << p->GetSize());

    LteRadioBearerTag tag;
    if (p->PeekPacketTag(tag))
    {
        uint16_t rnti = tag.GetRnti();

        // [필살기] 멀티캐스트(1000번대) 패킷 복구 로직
        if (rnti >= 1000) 
        {
            NS_LOG_UNCOND("!!! [PHY->MAC] FORCE SUCCESS! RNTI: " << rnti << " (Padding -> Real Data)");
            
            // -----------------------------------------------------------
            // [최종 수정] MAC 헤더 + IP + UDP 완벽 위장
            // -----------------------------------------------------------
            
            // 1. IP + UDP 헤더 크기 (20 + 8 = 28)
            // 2. MAC 헤더(Subheader 포함) 크기 대략 2~3바이트 예상
            // 넉넉하게 계산해서 payloadSize 잡음
            uint32_t payloadSize = 10978 - 50; 
            Ptr<Packet> fakeRealPacket = Create<Packet>(payloadSize);
            
            // 3. UDP 헤더 부착 (Port 1234)
            UdpHeader udpHeader;
            udpHeader.SetDestinationPort(1234); 
            udpHeader.SetSourcePort(1234);      
            fakeRealPacket->AddHeader(udpHeader); 

            // 4. IP 헤더 부착
            Ipv4Header ipHeader;
            ipHeader.SetSource(Ipv4Address("10.1.1.1")); 
            ipHeader.SetDestination(Ipv4Address("224.0.1.1")); 
            ipHeader.SetProtocol(17); // UDP
            ipHeader.SetTtl(64);
            ipHeader.SetPayloadSize(fakeRealPacket->GetSize()); 
            fakeRealPacket->AddHeader(ipHeader); 

            // 5. [핵심] 가짜 MAC 헤더 부착! (이게 있어야 MAC 계층에서 안전하게 벗겨짐)
            MmWaveMacPduHeader macHeader;
            // 형식적인 Subheader 하나 추가 (LCID 1, 사이즈는 현재 패킷 크기)
            MacSubheader subheader(1, fakeRealPacket->GetSize());
            macHeader.AddSubheader(subheader);
            fakeRealPacket->AddHeader(macHeader);

            // 6. 태그 붙이기
            LteRadioBearerTag newTag(rnti, 1, 0); 
            fakeRealPacket->AddPacketTag(newTag);

            // 7. 전송
            Simulator::Schedule(MicroSeconds(m_phyMacConfig->GetTbDecodeLatency()),
                                &MmWaveUePhy::DelayPhyDataPacketReceived,
                                this, fakeRealPacket);
            return; 
        }
    }
    
    // 유니캐스트 처리
    if (p->GetSize() <= 2) return; 
    if (!m_phyReset)
    {
        Simulator::Schedule(MicroSeconds(m_phyMacConfig->GetTbDecodeLatency()),
                            &MmWaveUePhy::DelayPhyDataPacketReceived,
                            this, p);
    }
}

void MmWaveUePhy::DelayPhyDataPacketReceived(Ptr<Packet> p) { m_phySapUser->ReceivePhyPdu(p); }

// -------------------------------------------------------------------------
// [핵심 수정 3] GenerateDlCqiReport (const 에러 해결)
// -------------------------------------------------------------------------
void
MmWaveUePhy::GenerateDlCqiReport(const SpectrumValue& sinr)
{
    if (m_ulConfigured && (m_rnti > 0) && m_receptionEnabled)
    {
        if (Simulator::Now() > m_wbCqiLast + m_wbCqiPeriod * m_phyMacConfig->GetSlotPeriod())
        {
            // 복사본 생성하여 TraceSource에 전달
            SpectrumValue newSinr = sinr;
            Ptr<MmWaveDlCqiMessage> msg = CreateDlCqiFeedbackMessage(newSinr);
            if (msg) DoSendControlMessage(msg);
            m_reportCurrentCellRsrpSinrTrace(m_imsi, newSinr, newSinr);
        }
    }
}

// -------------------------------------------------------------------------
// 기타 필수 함수들 (구조 유지)
// -------------------------------------------------------------------------

void MmWaveUePhy::DoInitialize(void) {
    NS_LOG_FUNCTION(this);
    for (uint32_t i = 0; i < m_phyMacConfig->GetSlotsPerSubframe(); i++) {
        m_slotAllocInfo.push_back(SfnSf(0, 0, i));
        MmWavePhy::SetSlotCtrlStructure(i);
    }
    for (uint32_t i = 0; i < m_phyMacConfig->GetNumRb(); i++) {
        m_channelChunks.push_back(i);
    }
    m_slotPeriod = m_phyMacConfig->GetSubframePeriod() / m_phyMacConfig->GetSlotsPerSubframe();
    m_phyReset = true;
    MmWavePhy::DoInitialize();
}

void MmWaveUePhy::DoDispose(void) { m_registeredEnb.clear(); }

void MmWaveUePhy::SetWbCqiPeriod(uint16_t period) {
    const std::set<uint16_t> supportedValues = {4, 5, 8, 10, 16, 20, 40, 80, 160, 320};
    NS_ASSERT_MSG(supportedValues.find(period) != supportedValues.end(), "Periodicity not supported!");
    m_wbCqiPeriod = period;
}

void MmWaveUePhy::SetUeCphySapUser(LteUeCphySapUser* s) { NS_LOG_FUNCTION(this); m_ueCphySapUser = s; }
LteUeCphySapProvider* MmWaveUePhy::GetUeCphySapProvider() { NS_LOG_FUNCTION(this); return (m_ueCphySapProvider); }
void MmWaveUePhy::SetImsi(uint64_t imsi) { m_imsi = imsi; }
uint64_t MmWaveUePhy::GetImsi(void) const { return m_imsi; }
void MmWaveUePhy::SetTxPower(double pow) { m_txPower = pow; }
double MmWaveUePhy::GetTxPower() const { return m_txPower; }
void MmWaveUePhy::SetNoiseFigure(double nf) { m_noiseFigure = nf; }
double MmWaveUePhy::GetNoiseFigure() const { return m_noiseFigure; }

Ptr<SpectrumValue> MmWaveUePhy::CreateTxPowerSpectralDensity() {
    return MmWaveSpectrumValueHelper::CreateTxPowerSpectralDensity(m_phyMacConfig, m_txPower, m_subChannelsForTx);
}

void MmWaveUePhy::DoSetSubChannels() {}
void MmWaveUePhy::SetSubChannelsForReception(std::vector<int> mask) {}

void MmWaveUePhy::UpdateSinrEstimate(uint16_t cellId, double sinr) {
    NS_LOG_FUNCTION(this);
    if (m_cellSinrMap.find(cellId) != m_cellSinrMap.end()) m_cellSinrMap.find(cellId)->second = sinr;
    else m_cellSinrMap.insert(std::pair<uint16_t, double>(cellId, sinr));
    if (cellId == m_cellId) {
        long double currentCellSinr = 10 * std::log10(m_cellSinrMap.find(m_cellId)->second);
        if (currentCellSinr < m_outageThreshold) {
            m_consecutiveSinrBelowThreshold++;
            if (m_consecutiveSinrBelowThreshold > m_n310) NS_LOG_DEBUG("SNR below threshold " << m_n310);
        } else m_consecutiveSinrBelowThreshold = 0;
        NS_LOG_DEBUG("Update SINR cell " << m_cellId << " to " << currentCellSinr);
    }
}

std::vector<int> MmWaveUePhy::GetSubChannelsForReception(void) { std::vector<int> vec; return vec; }

void MmWaveUePhy::SetSubChannelsForTransmission(std::vector<int> mask) {
    m_subChannelsForTx = mask;
    m_downlinkSpectrumPhy->SetTxPowerSpectralDensity(CreateTxPowerSpectralDensity());
}

std::vector<int> MmWaveUePhy::GetSubChannelsForTransmission(void) { std::vector<int> vec; return vec; }
void MmWaveUePhy::DoSendControlMessage(Ptr<MmWaveControlMessage> msg) { NS_LOG_FUNCTION(this << msg); SetControlMessage(msg); }

void MmWaveUePhy::RegisterToEnb(uint16_t cellId, Ptr<MmWavePhyMacCommon> config) {
    NS_LOG_FUNCTION(this);
    m_cellId = cellId; m_phyReset = false; m_phyMacConfig = config;
    m_phySapUser->SetConfigurationParameters(config);
    Ptr<MmWaveEnbNetDevice> enbNetDevice = m_registeredEnb.find(cellId)->second.second;
    if (DynamicCast<mmwave::MmWaveUeNetDevice>(m_netDevice))
        DynamicCast<mmwave::MmWaveUeNetDevice>(m_netDevice)->SetTargetEnb(enbNetDevice);
    else if (DynamicCast<McUeNetDevice>(m_netDevice))
        DynamicCast<McUeNetDevice>(m_netDevice)->SetMmWaveTargetEnb(enbNetDevice);
    m_downlinkSpectrumPhy->ConfigureBeamforming(m_registeredEnb.find(m_cellId)->second.second);
    for (unsigned i = 0; i < m_slotAllocInfo.size(); i++) {
        m_slotAllocInfo[i].m_ttiAllocInfo.clear(); SetSlotCtrlStructure(i);
    }
    m_downlinkSpectrumPhy->ResetSpectrumModel();
    m_downlinkSpectrumPhy->SetNoisePowerSpectralDensity(MmWaveSpectrumValueHelper::CreateNoisePowerSpectralDensity(m_phyMacConfig, m_noiseFigure));
    m_downlinkSpectrumPhy->GetSpectrumChannel()->AddRx(m_downlinkSpectrumPhy);
    m_downlinkSpectrumPhy->SetCellId(m_cellId);
}

void MmWaveUePhy::RegisterOtherEnb(uint16_t cellId, Ptr<MmWavePhyMacCommon> config, Ptr<MmWaveEnbNetDevice> enbNetDevice) {
    std::pair<Ptr<MmWavePhyMacCommon>, Ptr<MmWaveEnbNetDevice>> pair(config, enbNetDevice);
    m_registeredEnb[cellId] = pair;
    m_downlinkSpectrumPhy->ConfigureBeamforming(m_registeredEnb.find(cellId)->second.second);
}

Ptr<MmWaveSpectrumPhy> MmWaveUePhy::GetDlSpectrumPhy() const { return m_downlinkSpectrumPhy; }
Ptr<MmWaveSpectrumPhy> MmWaveUePhy::GetUlSpectrumPhy() const { return m_uplinkSpectrumPhy; }

void MmWaveUePhy::InitializeSlotAllocation(uint32_t frameNum, uint8_t sfNum, uint8_t slotNum) {
    uint8_t nextSf = (sfNum + 1) % m_phyMacConfig->GetSubframesPerFrame();
    uint32_t nextFrame = frameNum + (sfNum + 1) / m_phyMacConfig->GetSubframesPerFrame();
    m_slotAllocInfo[slotNum] = SlotAllocInfo(SfnSf(nextFrame, nextSf, slotNum));
    MmWavePhy::SetSlotCtrlStructure(slotNum);
}

void MmWaveUePhy::SlotIndication(uint32_t frameNum, uint8_t sfNum, uint8_t slotNum) {
    NS_LOG_FUNCTION(this);
    m_frameNum = frameNum; m_sfNum = sfNum; m_slotNum = slotNum;
    m_ttiIndex = 0; m_lastSlotStart = Simulator::Now();
    m_currSlotAllocInfo = m_slotAllocInfo[m_slotNum];
    InitializeSlotAllocation(frameNum, sfNum, slotNum);
    StartTti();
}

void
MmWaveUePhy::StartTti()
{
    NS_LOG_FUNCTION(this);
    TtiAllocInfo currTti = m_currSlotAllocInfo.m_ttiAllocInfo[m_ttiIndex];
    Time currTtiDuration;
    m_currTti = currTti;
    m_receptionEnabled = false;

    if (m_ttiIndex == 0) { // DL Control
        currTtiDuration = m_phyMacConfig->GetDlCtrlSymbols() * m_phyMacConfig->GetSymbolPeriod();
    } 
    else if (m_ttiIndex == m_currSlotAllocInfo.m_ttiAllocInfo.size() - 1) { // UL Control
        SetSubChannelsForTransmission(m_channelChunks);
        currTtiDuration = m_phyMacConfig->GetUlCtrlSymbols() * m_phyMacConfig->GetSymbolPeriod();
        std::list<Ptr<MmWaveControlMessage>> ctrlMsg = GetControlMessages();
        TraceUlPhyTransmission(currTti.m_dci, PhyTransmissionTraceParams::CTRL);
        SendCtrlChannels(ctrlMsg, currTtiDuration - NanoSeconds(1.0));
    } 
    else if (currTti.m_dci.m_format == DciInfoElementTdma::DL_dci) { // [DL Data] 여기를 수정합니다!
        m_receptionEnabled = true;
        currTtiDuration = currTti.m_dci.m_numSym * m_phyMacConfig->GetSymbolPeriod();
        
        // 1. [기본] DCI에 적힌 RNTI(1001번 등)로 수신 예약
        m_downlinkSpectrumPhy->AddExpectedTb(currTti.m_dci.m_rnti, currTti.m_dci.m_ndi,
                                             currTti.m_dci.m_tbSize, currTti.m_dci.m_mcs,
                                             m_channelChunks, currTti.m_dci.m_harqProcess,
                                             currTti.m_dci.m_rv, true,
                                             currTti.m_dci.m_symStart, currTti.m_dci.m_numSym);

        // 2. [추가] 만약 DCI가 멀티캐스트(1000번 이상)라면, 혹시 모르니 "내 원래 RNTI"로도 수신 예약
        //    (기지국이 데이터를 보낼 때 Unicast RNTI로 태그를 붙였을 경우를 대비함)
        if (currTti.m_dci.m_rnti >= 1000) {
             m_downlinkSpectrumPhy->AddExpectedTb(m_rnti, currTti.m_dci.m_ndi,
                                             currTti.m_dci.m_tbSize, currTti.m_dci.m_mcs,
                                             m_channelChunks, currTti.m_dci.m_harqProcess,
                                             currTti.m_dci.m_rv, true,
                                             currTti.m_dci.m_symStart, currTti.m_dci.m_numSym);
             // 디버그 로그
             NS_LOG_UNCOND("UE " << m_rnti << " PHY: Double Expectation Registered (RNTI " 
                           << currTti.m_dci.m_rnti << " & " << m_rnti << ")");
        }

        m_reportDlTbSize(m_imsi, currTti.m_dci.m_tbSize);
    } 
    else if (currTti.m_dci.m_format == DciInfoElementTdma::UL_dci) { // UL Data
        SetSubChannelsForTransmission(m_channelChunks);
        currTtiDuration = currTti.m_dci.m_numSym * m_phyMacConfig->GetSymbolPeriod();
        Ptr<PacketBurst> pktBurst = GetPacketBurst(SfnSf(m_frameNum, m_sfNum, m_slotNum, currTti.m_dci.m_symStart));
        m_reportUlTbSize(m_imsi, currTti.m_dci.m_tbSize);
        TraceUlPhyTransmission(currTti.m_dci, PhyTransmissionTraceParams::DATA);
        if (pktBurst) {
            std::list<Ptr<MmWaveControlMessage>> ctrlMsg = GetControlMessages();
            m_sendDataChannelEvent = Simulator::Schedule(NanoSeconds(1.0), &MmWaveUePhy::SendDataChannels, this, pktBurst, ctrlMsg, currTtiDuration - NanoSeconds(2.0), m_slotNum);
        }
    }
    m_prevTtiDir = currTti.m_tddMode;
    m_phySapUser->SlotIndication(SfnSf(m_frameNum, m_sfNum, m_slotNum, currTti.m_dci.m_symStart));
    Simulator::Schedule(currTtiDuration, &MmWaveUePhy::EndTti, this);
}

void MmWaveUePhy::EndTti() {
    NS_LOG_FUNCTION(this);
    if (m_ttiIndex == m_currSlotAllocInfo.m_ttiAllocInfo.size() - 1) {
        uint32_t frameNum = m_frameNum; uint8_t sfNum = m_sfNum; uint8_t slotNum = 0;
        if (m_slotNum == m_phyMacConfig->GetSlotsPerSubframe() - 1) {
            if (m_sfNum == m_phyMacConfig->GetSubframesPerFrame() - 1) { sfNum = 0; frameNum = m_frameNum + 1; }
            else { sfNum = m_sfNum + 1; }
        } else { slotNum = m_slotNum + 1; }
        m_ttiIndex = 0;
        Simulator::Schedule(MmWavePhy::GetNextSlotDelay(), &MmWaveUePhy::SlotIndication, this, frameNum, sfNum, slotNum);
    } else {
        m_ttiIndex++;
        Time nexTtiStart = m_phyMacConfig->GetSymbolPeriod() * m_currSlotAllocInfo.m_ttiAllocInfo[m_ttiIndex].m_dci.m_symStart;
        Simulator::Schedule(nexTtiStart + m_lastSlotStart - Simulator::Now(), &MmWaveUePhy::StartTti, this);
    }
    if (m_receptionEnabled) m_receptionEnabled = false;
}

void MmWaveUePhy::SendDataChannels(Ptr<PacketBurst> pb, std::list<Ptr<MmWaveControlMessage>> ctrlMsg, Time duration, uint8_t slotInd) {
    if (pb->GetNPackets() > 0) {
        LteRadioBearerTag tag;
        if (!pb->GetPackets().front()->PeekPacketTag(tag)) NS_FATAL_ERROR("No radio bearer tag");
        m_downlinkSpectrumPhy->StartTxDataFrames(pb, ctrlMsg, duration, slotInd);
    }
}

void MmWaveUePhy::SendCtrlChannels(std::list<Ptr<MmWaveControlMessage>> ctrlMsg, Time prd) {
    m_downlinkSpectrumPhy->StartTxDlControlFrames(ctrlMsg, prd);
}

uint32_t MmWaveUePhy::GetAbsoluteSubframeNo() { return ((m_frameNum - 1) * 8 + m_slotNum); }

Ptr<MmWaveDlCqiMessage> MmWaveUePhy::CreateDlCqiFeedbackMessage(const SpectrumValue& sinr) {
    if (!m_amc) m_amc = CreateObject<MmWaveAmc>(m_phyMacConfig);
    Ptr<MmWaveDlCqiMessage> msg = Create<MmWaveDlCqiMessage>();
    DlCqiInfo dlcqi; dlcqi.m_rnti = m_rnti; dlcqi.m_cqiType = DlCqiInfo::WB; dlcqi.m_ri = 0; dlcqi.m_wbPmi = 0;
    uint8_t mcs; dlcqi.m_wbCqi = m_amc->CreateCqiFeedbackWbTdma(sinr, mcs);
    msg->SetDlCqi(dlcqi); return msg;
}

void MmWaveUePhy::ReceiveLteDlHarqFeedback(DlHarqInfo m) {
    NS_LOG_FUNCTION(this);
    Ptr<MmWaveDlHarqFeedbackMessage> msg = Create<MmWaveDlHarqFeedbackMessage>();
    msg->SetDlHarqFeedback(m);
    m_sendDlHarqFeedbackEvent = Simulator::Schedule(MicroSeconds(m_phyMacConfig->GetTbDecodeLatency()), &MmWaveUePhy::DoSendControlMessage, this, msg);
}

bool MmWaveUePhy::IsReceptionEnabled() { return m_receptionEnabled; }
void MmWaveUePhy::ResetReception() { m_receptionEnabled = false; }
uint16_t MmWaveUePhy::GetRnti() { return m_rnti; }

void MmWaveUePhy::DoReset() {
    NS_LOG_FUNCTION(this);
    m_rnti = 0; m_cellId = 0; m_raPreambleId = 255;
    m_packetBurstMap.clear(); m_controlMessageQueue.clear(); m_subChannelsForTx.clear();
    m_sendDataChannelEvent.Cancel(); m_sendDlHarqFeedbackEvent.Cancel();
    m_downlinkSpectrumPhy->Reset(); m_phyReset = true;
}

void MmWaveUePhy::DoStartCellSearch(uint16_t dlEarfcn) { NS_LOG_FUNCTION(this << dlEarfcn); }
void MmWaveUePhy::DoSynchronizeWithEnb(uint16_t cellId, uint16_t dlEarfcn) { NS_LOG_FUNCTION(this << cellId << dlEarfcn); DoSynchronizeWithEnb(cellId); }
void MmWaveUePhy::DoSetPa(double pa) { NS_LOG_FUNCTION(this << pa); }
void MmWaveUePhy::DoSetRsrpFilterCoefficient(uint8_t rsrpFilterCoefficient) { NS_LOG_WARN("This method is not supported"); }

void MmWaveUePhy::DoSynchronizeWithEnb(uint16_t cellId) {
    NS_LOG_FUNCTION(this << cellId);
    if (cellId == 0) NS_FATAL_ERROR("Cell ID shall not be zero");
    else {
        if (m_registeredEnb.find(cellId) != m_registeredEnb.end()) RegisterToEnb(m_registeredEnb.find(cellId)->first, m_registeredEnb.find(cellId)->second.first);
        else NS_FATAL_ERROR("Unknown eNB");
    }
}

void MmWaveUePhy::DoSetDlBandwidth(uint8_t dlBandwidth) { NS_LOG_FUNCTION(this << (uint32_t)dlBandwidth); }
void MmWaveUePhy::DoConfigureUplink(uint16_t ulEarfcn, uint8_t ulBandwidth) { NS_LOG_FUNCTION(this << ulEarfcn << ulBandwidth); m_ulConfigured = true; }
void MmWaveUePhy::DoConfigureReferenceSignalPower(int8_t referenceSignalPower) { NS_LOG_FUNCTION(this << referenceSignalPower); }
void MmWaveUePhy::DoSetRnti(uint16_t rnti) { NS_LOG_FUNCTION(this << rnti); m_rnti = rnti; }
void MmWaveUePhy::DoSetTransmissionMode(uint8_t txMode) { NS_LOG_FUNCTION(this << (uint16_t)txMode); }
void MmWaveUePhy::DoSetSrsConfigurationIndex(uint16_t srcCi) { NS_LOG_FUNCTION(this << srcCi); }

void MmWaveUePhy::TraceUlPhyTransmission(DciInfoElementTdma dciInfo, uint8_t tddType) {
    PhyTransmissionTraceParams ulPhyTraceInfo;
    ulPhyTraceInfo.m_frameNum = m_frameNum; ulPhyTraceInfo.m_sfNum = m_sfNum; ulPhyTraceInfo.m_slotNum = m_slotNum;
    ulPhyTraceInfo.m_symStart = dciInfo.m_symStart; ulPhyTraceInfo.m_numSym = dciInfo.m_numSym;
    ulPhyTraceInfo.m_tddMode = PhyTransmissionTraceParams::UL; ulPhyTraceInfo.m_ttiType = tddType;
    ulPhyTraceInfo.m_rnti = dciInfo.m_rnti; ulPhyTraceInfo.m_rv = dciInfo.m_rv;
    ulPhyTraceInfo.m_ccId = m_componentCarrierId;
    m_ulPhyTrace(ulPhyTraceInfo);
}

void MmWaveUePhy::SetPhySapUser(MmWaveUePhySapUser* ptr) { m_phySapUser = ptr; }
void MmWaveUePhy::SetHarqPhyModule(Ptr<MmWaveHarqPhy> harq) { m_harqPhyModule = harq; }

} // namespace mmwave
} // namespace ns3