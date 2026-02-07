/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
 *   Copyright (c) 2016, 2018, University of Padova, Dep. of Information Engineering, SIGNET lab.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *   Author: Marco Miozzo <marco.miozzo@cttc.es>
 *           Nicola Baldo  <nbaldo@cttc.es>
 *
 *   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
 *                         Sourjya Dutta <sdutta@nyu.edu>
 *                         Russell Ford <russell.ford@nyu.edu>
 *                         Menglei Zhang <menglei@nyu.edu>
 *
 * Modified by: Michele Polese <michele.polese@gmail.com>
 *                 Dual Connectivity and Handover functionalities
 *
 * Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                               Integration of Carrier Aggregation
 */

#include "mmwave-enb-mac.h"
#include <queue>
#include <vector>
#include "mmwave-mac-pdu-header.h"
#include "mmwave-mac-sched-sap.h"
#include "mmwave-mac-scheduler.h"
#include "mmwave-phy-mac-common.h"

#include <ns3/log.h>
#include <ns3/lte-enb-cmac-sap.h>
#include <ns3/lte-mac-sap.h>
uint32_t g_activeIdx = 0;
extern std::vector<uint32_t> g_activeSectors;


namespace ns3
{

namespace mmwave
{

NS_LOG_COMPONENT_DEFINE("MmWaveEnbMac");

NS_OBJECT_ENSURE_REGISTERED(MmWaveEnbMac);

// //////////////////////////////////////
// member SAP forwarders
// //////////////////////////////////////

class MmWaveEnbMacMemberEnbCmacSapProvider : public LteEnbCmacSapProvider
{
  public:
    MmWaveEnbMacMemberEnbCmacSapProvider(MmWaveEnbMac* mac);

    // inherited from LteEnbCmacSapProvider
    virtual void ConfigureMac(uint8_t ulBandwidth, uint8_t dlBandwidth);
    virtual void AddUe(uint16_t rnti);
    virtual void RemoveUe(uint16_t rnti);
    virtual void AddLc(LcInfo lcinfo, LteMacSapUser* msu);
    virtual void ReconfigureLc(LcInfo lcinfo);
    virtual void ReleaseLc(uint16_t rnti, uint8_t lcid);
    virtual void UeUpdateConfigurationReq(UeConfig params);
    virtual RachConfig GetRachConfig();
    virtual AllocateNcRaPreambleReturnValue AllocateNcRaPreamble(uint16_t rnti);

  private:
    MmWaveEnbMac* m_mac;
};

MmWaveEnbMacMemberEnbCmacSapProvider::MmWaveEnbMacMemberEnbCmacSapProvider(MmWaveEnbMac* mac)
    : m_mac(mac)
{
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::ConfigureMac(uint8_t ulBandwidth, uint8_t dlBandwidth)
{
    m_mac->DoConfigureMac(ulBandwidth, dlBandwidth);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::AddUe(uint16_t rnti)
{
    m_mac->DoAddUe(rnti);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::RemoveUe(uint16_t rnti)
{
    m_mac->DoRemoveUe(rnti);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::AddLc(LcInfo lcinfo, LteMacSapUser* msu)
{
    m_mac->DoAddLc(lcinfo, msu);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::ReconfigureLc(LcInfo lcinfo)
{
    m_mac->DoReconfigureLc(lcinfo);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::ReleaseLc(uint16_t rnti, uint8_t lcid)
{
    m_mac->DoReleaseLc(rnti, lcid);
}

void
MmWaveEnbMacMemberEnbCmacSapProvider::UeUpdateConfigurationReq(UeConfig params)
{
    m_mac->UeUpdateConfigurationReq(params);
}

LteEnbCmacSapProvider::RachConfig
MmWaveEnbMacMemberEnbCmacSapProvider::GetRachConfig()
{
    return m_mac->DoGetRachConfig();
}

LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue
MmWaveEnbMacMemberEnbCmacSapProvider::AllocateNcRaPreamble(uint16_t rnti)
{
    return m_mac->DoAllocateNcRaPreamble(rnti);
}

// SAP
// ENB MAC-Phy
class MmWaveMacEnbMemberPhySapUser : public MmWaveEnbPhySapUser
{
  public:
    MmWaveMacEnbMemberPhySapUser(MmWaveEnbMac* mac);

    virtual void ReceivePhyPdu(Ptr<Packet> p);

    virtual void ReceiveControlMessage(Ptr<MmWaveControlMessage> msg);

    virtual void SlotIndication(SfnSf snf);

    virtual void UlCqiReport(MmWaveMacSchedSapProvider::SchedUlCqiInfoReqParameters cqi);

    virtual void ReceiveRachPreamble(uint32_t raId);

    virtual void UlHarqFeedback(UlHarqInfo params);

  private:
    MmWaveEnbMac* m_mac;
};

MmWaveMacEnbMemberPhySapUser::MmWaveMacEnbMemberPhySapUser(MmWaveEnbMac* mac)
    : m_mac(mac)
{
}

void
MmWaveMacEnbMemberPhySapUser::ReceivePhyPdu(Ptr<Packet> p)
{
    m_mac->DoReceivePhyPdu(p);
}

void
MmWaveMacEnbMemberPhySapUser::ReceiveControlMessage(Ptr<MmWaveControlMessage> msg)
{
    m_mac->DoReceiveControlMessage(msg);
}

void
MmWaveMacEnbMemberPhySapUser::SlotIndication(SfnSf sfn)
{
    m_mac->DoSlotIndication(sfn);
}

void
MmWaveMacEnbMemberPhySapUser::UlCqiReport(
    MmWaveMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi)
{
    m_mac->DoUlCqiReport(ulcqi);
}

void
MmWaveMacEnbMemberPhySapUser::ReceiveRachPreamble(uint32_t raId)
{
    m_mac->ReceiveRachPreamble(raId);
}

void
MmWaveMacEnbMemberPhySapUser::UlHarqFeedback(UlHarqInfo params)
{
    m_mac->DoUlHarqFeedback(params);
}

// MAC Sched

class MmWaveMacMemberMacSchedSapUser : public MmWaveMacSchedSapUser
{
  public:
    MmWaveMacMemberMacSchedSapUser(MmWaveEnbMac* mac);
    virtual void SchedConfigInd(const struct SchedConfigIndParameters& params);

  private:
    MmWaveEnbMac* m_mac;
};

MmWaveMacMemberMacSchedSapUser::MmWaveMacMemberMacSchedSapUser(MmWaveEnbMac* mac)
    : m_mac(mac)
{
    //   Some blank spaces
}

void
MmWaveMacMemberMacSchedSapUser::SchedConfigInd(const struct SchedConfigIndParameters& params)
{
    m_mac->DoSchedConfigIndication(params);
}

class MmWaveMacMemberMacCschedSapUser : public MmWaveMacCschedSapUser
{
  public:
    MmWaveMacMemberMacCschedSapUser(MmWaveEnbMac* mac);

    virtual void CschedCellConfigCnf(
        const struct MmWaveMacCschedSapUser::CschedCellConfigCnfParameters& params);
    virtual void CschedUeConfigCnf(
        const struct MmWaveMacCschedSapUser::CschedUeConfigCnfParameters& params);
    virtual void CschedLcConfigCnf(
        const struct MmWaveMacCschedSapUser::CschedLcConfigCnfParameters& params);
    virtual void CschedLcReleaseCnf(
        const struct MmWaveMacCschedSapUser::CschedLcReleaseCnfParameters& params);
    virtual void CschedUeReleaseCnf(
        const struct MmWaveMacCschedSapUser::CschedUeReleaseCnfParameters& params);
    virtual void CschedUeConfigUpdateInd(
        const struct MmWaveMacCschedSapUser::CschedUeConfigUpdateIndParameters& params);
    virtual void CschedCellConfigUpdateInd(
        const struct MmWaveMacCschedSapUser::CschedCellConfigUpdateIndParameters& params);

  private:
    MmWaveEnbMac* m_mac;
};

MmWaveMacMemberMacCschedSapUser::MmWaveMacMemberMacCschedSapUser(MmWaveEnbMac* mac)
    : m_mac(mac)
{
}

void
MmWaveMacMemberMacCschedSapUser::CschedCellConfigCnf(
    const struct CschedCellConfigCnfParameters& params)
{
    m_mac->DoCschedCellConfigCnf(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedUeConfigCnf(const struct CschedUeConfigCnfParameters& params)
{
    m_mac->DoCschedUeConfigCnf(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedLcConfigCnf(const struct CschedLcConfigCnfParameters& params)
{
    m_mac->DoCschedLcConfigCnf(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedLcReleaseCnf(
    const struct CschedLcReleaseCnfParameters& params)
{
    m_mac->DoCschedLcReleaseCnf(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedUeReleaseCnf(
    const struct CschedUeReleaseCnfParameters& params)
{
    m_mac->DoCschedUeReleaseCnf(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedUeConfigUpdateInd(
    const struct CschedUeConfigUpdateIndParameters& params)
{
    m_mac->DoCschedUeConfigUpdateInd(params);
}

void
MmWaveMacMemberMacCschedSapUser::CschedCellConfigUpdateInd(
    const struct CschedCellConfigUpdateIndParameters& params)
{
    m_mac->DoCschedCellConfigUpdateInd(params);
}

// Enb Mac Sap Provider

template <class C>
class EnbMacMemberMmWaveMacSapProvider : public LteMacSapProvider
{
  public:
    EnbMacMemberMmWaveMacSapProvider(C* mac);

    // inherited from LteMacSapProvider
    virtual void TransmitPdu(TransmitPduParameters params);
    virtual void ReportBufferStatus(ReportBufferStatusParameters params);

  private:
    C* m_mac;
};

template <class C>
EnbMacMemberMmWaveMacSapProvider<C>::EnbMacMemberMmWaveMacSapProvider(C* mac)
    : m_mac(mac)
{
}

template <class C>
void
EnbMacMemberMmWaveMacSapProvider<C>::TransmitPdu(TransmitPduParameters params)
{
    m_mac->DoTransmitPdu(params);
}

template <class C>
void
EnbMacMemberMmWaveMacSapProvider<C>::ReportBufferStatus(ReportBufferStatusParameters params)
{
    m_mac->DoReportBufferStatus(params);
}

TypeId
MmWaveEnbMac::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::MmWaveEnbMac")
            .SetParent<MmWaveMac>()
            .AddConstructor<MmWaveEnbMac>()
            .AddAttribute("NumberOfRaPreambles",
                          "how many random access preambles are available for the contention based "
                          "RACH process",
                          UintegerValue(50),
                          MakeUintegerAccessor(&MmWaveEnbMac::m_numberOfRaPreambles),
                          MakeUintegerChecker<uint8_t>(4, 64))
            .AddAttribute("PreambleTransMax",
                          "Maximum number of random access preamble transmissions",
                          UintegerValue(50),
                          MakeUintegerAccessor(&MmWaveEnbMac::m_preambleTransMax),
                          MakeUintegerChecker<uint8_t>(3, 200))
            .AddAttribute("RaResponseWindowSize",
                          "length of the window (in TTIs) for the reception of the random access "
                          "response (RAR); the resulting RAR timeout is this value + 3 ms",
                          UintegerValue(3),
                          MakeUintegerAccessor(&MmWaveEnbMac::m_raResponseWindowSize),
                          MakeUintegerChecker<uint8_t>(2, 10))
            .AddAttribute("ComponentCarrierId",
                          "ComponentCarrier Id, needed to reply on the appropriate sap.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&MmWaveEnbMac::m_componentCarrierId),
                          MakeUintegerChecker<uint8_t>(0, 4))
            .AddTraceSource("DlMacTxCallback",
                            "MAC transmission with tb size and number of retx.",
                            MakeTraceSourceAccessor(&MmWaveEnbMac::m_macDlTxSizeRetx),
                            "ns3::LteRlc::RetransmissionCountCallback")
            .AddTraceSource("TxMacPacketTraceEnb",
                            "Packets transmitted by EnbMac",
                            MakeTraceSourceAccessor(&MmWaveEnbMac::m_txMacPacketTraceEnb),
                            "ns3::EnbTxRxPacketCount::TracedCallback")
            .AddTraceSource("SchedulingTraceEnb",
                            "Information regarding scheduling allocation.",
                            MakeTraceSourceAccessor(&MmWaveEnbMac::m_schedEnbInfo),
                            "ns3::MmWaveEnbMac::SchedAllocTracedCallback");
    return tid;
}

MmWaveEnbMac::MmWaveEnbMac(void)
    : m_ccmMacSapUser(0),
      m_frameNum(0),
      m_sfNum(0),
      m_slotNum(0),
      m_tbUid(0)
{
    NS_LOG_FUNCTION(this);
    m_cmacSapProvider = new MmWaveEnbMacMemberEnbCmacSapProvider(this);
    m_macSapProvider = new EnbMacMemberMmWaveMacSapProvider<MmWaveEnbMac>(this);
    m_phySapUser = new MmWaveMacEnbMemberPhySapUser(this);
    m_macSchedSapUser = new MmWaveMacMemberMacSchedSapUser(this);
    m_macCschedSapUser = new MmWaveMacMemberMacCschedSapUser(this);

    m_ccmMacSapProvider = new MemberLteCcmMacSapProvider<MmWaveEnbMac>(this);
    Initialize();
}

MmWaveEnbMac::~MmWaveEnbMac(void)
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbMac::DoDispose(void)
{
    NS_LOG_FUNCTION(this);
    m_dlCqiReceived.clear();
    m_ulCqiReceived.clear();
    m_ulCeReceived.clear();
    //  m_dlHarqInfoListReceived.clear ();
    //  m_ulHarqInfoListReceived.clear ();
    m_miDlHarqProcessesPackets.clear();
    delete m_macSapProvider;
    delete m_cmacSapProvider;
    delete m_macSchedSapUser;
    //  delete m_macCschedSapUser;
    delete m_phySapUser;
    delete m_ccmMacSapProvider;
}

void
MmWaveEnbMac::SetComponentCarrierId(uint8_t index)
{
    m_componentCarrierId = index;
}

void
MmWaveEnbMac::SetConfigurationParameters(Ptr<MmWavePhyMacCommon> ptrConfig)
{
    m_phyMacConfig = ptrConfig;
}

Ptr<MmWavePhyMacCommon>
MmWaveEnbMac::GetConfigurationParameters(void) const
{
    return m_phyMacConfig;
}

void
MmWaveEnbMac::ReceiveRachPreamble(uint32_t raId)
{
    ++m_receivedRachPreambleCount[raId];
}

LteMacSapProvider*
MmWaveEnbMac::GetUeMacSapProvider(void)
{
    return m_macSapProvider;
}

LteEnbCmacSapProvider*
MmWaveEnbMac::GetEnbCmacSapProvider(void)
{
    return m_cmacSapProvider;
}

void
MmWaveEnbMac::SetEnbCmacSapUser(LteEnbCmacSapUser* s)
{
    m_cmacSapUser = s;
}

void
MmWaveEnbMac::DoSlotIndication(SfnSf sfnSf)
{
    NS_LOG_FUNCTION(this);
    m_frameNum = sfnSf.m_frameNum;
    m_sfNum = sfnSf.m_sfNum;
    m_slotNum = sfnSf.m_slotNum;
    bool slotStart = (sfnSf.m_symStart == 0);

    // --- DOWNLINK ---
    // Send Dl-CQI info to the scheduler    if(m_dlCqiReceived.size () > 0)
    {
        MmWaveMacSchedSapProvider::SchedDlCqiInfoReqParameters dlCqiInfoReq;
        dlCqiInfoReq.m_sfnsf = sfnSf;

        dlCqiInfoReq.m_cqiList.insert(dlCqiInfoReq.m_cqiList.begin(),
                                      m_dlCqiReceived.begin(),
                                      m_dlCqiReceived.end());
        m_dlCqiReceived.erase(m_dlCqiReceived.begin(), m_dlCqiReceived.end());

        m_macSchedSapProvider->SchedDlCqiInfoReq(dlCqiInfoReq);
    }

    if (!m_receivedRachPreambleCount.empty())
    {
        // process received RACH preambles and notify the scheduler
        Ptr<MmWaveRarMessage> rarMsg = Create<MmWaveRarMessage>();

        for (std::map<uint8_t, uint32_t>::const_iterator it = m_receivedRachPreambleCount.begin();
             it != m_receivedRachPreambleCount.end();
             ++it)
        {
            uint16_t rnti;
            std::map<uint8_t, NcRaPreambleInfo>::iterator jt =
                m_allocatedNcRaPreambleMap.find(it->first);
            NS_LOG_LOGIC("received RapId " << (uint16_t)it->first);
            if (jt != m_allocatedNcRaPreambleMap.end())
            {
                rnti = jt->second.rnti;
                NS_LOG_INFO("preambleId previously allocated for NC based RA, RNTI ="
                            << (uint32_t)rnti << ", sending RAR");
                m_allocatedNcRaPreambleMap.erase(jt);
            }
            else
            {
                rnti = m_cmacSapUser->AllocateTemporaryCellRnti();
                NS_LOG_INFO("preambleId " << (uint32_t)it->first << ": allocated T-C-RNTI "
                                          << (uint32_t)rnti << ", sending RAR");
            }
            MmWaveRarMessage::Rar rar = MmWaveRarMessage::Rar();
            rar.rapId = (*it).first;
            rar.rarPayload.m_rnti = rnti;
            rarMsg->AddRar(rar);
            // NS_ASSERT_MSG((*it).second ==1, "Two user send the same Rach ID, collision
            // detected");
        }
        m_phySapProvider->SendControlMessage(rarMsg);
        m_receivedRachPreambleCount.clear();
    }

    // --- UPLINK ---
    // Send UL-CQI info to the scheduler
    std::vector<MmWaveMacSchedSapProvider::SchedUlCqiInfoReqParameters>::iterator itCqi;
    for (uint16_t i = 0; i < m_ulCqiReceived.size(); i++)
    {
        // m_ulCqiReceived.at (i).m_sfnSf = ((0x3FF & frameNum) << 16) | ((0xFF & subframeNum) << 8)
        // | (0xFF & slotNum);
        m_macSchedSapProvider->SchedUlCqiInfoReq(m_ulCqiReceived.at(i));
    }
    m_ulCqiReceived.clear();

    // Send UL BSR reports to the scheduler
    if (m_ulCeReceived.size() > 0)
    {
        MmWaveMacSchedSapProvider::SchedUlMacCtrlInfoReqParameters ulMacReq;
        ulMacReq.m_sfnSf = sfnSf;
        NS_LOG_DEBUG("ulMacReq.m_macCeList size " << m_ulCeReceived.size());
        ulMacReq.m_macCeList.insert(ulMacReq.m_macCeList.begin(),
                                    m_ulCeReceived.begin(),
                                    m_ulCeReceived.end());
        m_ulCeReceived.erase(m_ulCeReceived.begin(), m_ulCeReceived.end());
        m_macSchedSapProvider->SchedUlMacCtrlInfoReq(ulMacReq);
    }

    if (slotStart)
    {
        NS_LOG_LOGIC("Starting a new NR slot - DoSlotIndication");
        NS_LOG_DEBUG("Current frame " << sfnSf.m_frameNum << " subframe " << (unsigned)sfnSf.m_sfNum
                                      << " slot " << (unsigned)sfnSf.m_slotNum);
        // Trigger scheduler, taking into consideration the L1L2 delay

        uint8_t delayedSlotNum =
            (m_slotNum + m_phyMacConfig->GetL1L2Latency()) % m_phyMacConfig->GetSlotsPerSubframe();
        uint8_t deltaSubframe =
            (m_slotNum + m_phyMacConfig->GetL1L2Latency()) / m_phyMacConfig->GetSlotsPerSubframe();
        uint8_t delayedSchedSfNum =
            (m_sfNum + deltaSubframe) % m_phyMacConfig->GetSubframesPerFrame();
        uint32_t delayedSchedFrameNum =
            m_frameNum + ((m_sfNum + deltaSubframe) / m_phyMacConfig->GetSubframesPerFrame());

        NS_ASSERT((delayedSlotNum < m_phyMacConfig->GetSlotsPerSubframe()) &&
                  (delayedSchedSfNum < m_phyMacConfig->GetSubframesPerFrame()) &&
                  (deltaSubframe >= 0) && (delayedSlotNum >= 0) && (delayedSchedSfNum >= 0) &&
                  (delayedSchedFrameNum >= m_frameNum));

        NS_LOG_DEBUG("Requesting scheduler allocation for frame "
                     << delayedSchedFrameNum << " subframe " << (unsigned)delayedSchedSfNum
                     << " slot " << (unsigned)delayedSlotNum);

        MmWaveMacSchedSapProvider::SchedTriggerReqParameters params;
        SfnSf schedSfn(delayedSchedFrameNum, delayedSchedSfNum, delayedSlotNum);
        params.m_snfSf = schedSfn;

        // Forward DL HARQ feebacks collected during last subframe TTI
        if (m_dlHarqInfoReceived.size() > 0)
        {
            params.m_dlHarqInfoList = m_dlHarqInfoReceived;
            // empty local buffer
            m_dlHarqInfoReceived.clear();
        }

        // Forward UL HARQ feebacks collected during last TTI
        if (m_ulHarqInfoReceived.size() > 0)
        {
            params.m_ulHarqInfoList = m_ulHarqInfoReceived;
            // empty local buffer
            m_ulHarqInfoReceived.clear();
        }

        params.m_ueList = m_associatedUe;
        m_macSchedSapProvider->SchedTriggerReq(params);
    }
}

void
MmWaveEnbMac::SetCellId(uint16_t cellId)
{
    m_cellId = cellId;
}

void
MmWaveEnbMac::SetMcs(int mcs)
{
    m_macSchedSapProvider->SchedSetMcs(mcs);
}

void
MmWaveEnbMac::AssociateUeMAC(uint64_t imsi)
{
    // NS_LOG_UNCOND (this<<"Associate UE (imsi:"<< imsi<<" ) with enb");

    // m_associatedUe.push_back (imsi);
}

void
MmWaveEnbMac::SetForwardUpCallback(Callback<void, Ptr<Packet>> cb)
{
    m_forwardUpCallback = cb;
}

void
MmWaveEnbMac::DoReceivePhyPdu(Ptr<Packet> p)
{
    NS_LOG_FUNCTION(this);
    LteRadioBearerTag tag;
    p->RemovePacketTag(tag);
    uint16_t rnti = tag.GetRnti();
    MmWaveMacPduHeader macHeader;
    p->RemoveHeader(macHeader);
    std::map<uint16_t, std::map<uint8_t, LteMacSapUser*>>::iterator rntiIt =
        m_rlcAttached.find(rnti);
    NS_ASSERT_MSG(rntiIt != m_rlcAttached.end(), "could not find RNTI" << rnti);
    std::vector<MacSubheader> macSubheaders = macHeader.GetSubheaders();
    uint32_t currPos = 0;
    for (unsigned ipdu = 0; ipdu < macSubheaders.size(); ipdu++)
    {
        if (macSubheaders[ipdu].m_size == 0)
        {
            continue;
        }
        std::map<uint8_t, LteMacSapUser*>::iterator lcidIt =
            rntiIt->second.find(macSubheaders[ipdu].m_lcid);
        NS_ASSERT_MSG(lcidIt != rntiIt->second.end(),
                      "could not find LCID" << macSubheaders[ipdu].m_lcid);
        Ptr<Packet> rlcPdu;
        if ((p->GetSize() - currPos) < (uint32_t)macSubheaders[ipdu].m_size)
        {
            NS_LOG_ERROR("Packet size less than specified in MAC header (actual= "
                         << p->GetSize() << " header= " << (uint32_t)macSubheaders[ipdu].m_size
                         << ")");
        }
        else if ((p->GetSize() - currPos) > (uint32_t)macSubheaders[ipdu].m_size)
        {
            NS_LOG_DEBUG(
                "Fragmenting MAC PDU (packet size greater than specified in MAC header (actual= "
                << p->GetSize() << " header= " << (uint32_t)macSubheaders[ipdu].m_size << ")");
            rlcPdu = p->CreateFragment(currPos, macSubheaders[ipdu].m_size);
            currPos += macSubheaders[ipdu].m_size;

            LteMacSapUser::ReceivePduParameters rxPduParams;
            rxPduParams.p = rlcPdu;
            rxPduParams.rnti = rnti;
            rxPduParams.lcid = macSubheaders[ipdu].m_lcid;
            (*lcidIt).second->ReceivePdu(rxPduParams);
        }
        else
        {
            rlcPdu = p->CreateFragment(currPos, p->GetSize() - currPos);
            currPos = p->GetSize();

            LteMacSapUser::ReceivePduParameters rxPduParams;
            rxPduParams.p = rlcPdu;
            rxPduParams.rnti = rnti;
            rxPduParams.lcid = macSubheaders[ipdu].m_lcid;
            (*lcidIt).second->ReceivePdu(rxPduParams);
        }
        NS_LOG_INFO("MmWave Enb Mac Rx Packet, Rnti:" << rnti << " lcid:"
                                                      << (uint32_t)macSubheaders[ipdu].m_lcid
                                                      << " size:" << macSubheaders[ipdu].m_size);
    }
}

MmWaveEnbPhySapUser*
MmWaveEnbMac::GetPhySapUser()
{
    return m_phySapUser;
}

void
MmWaveEnbMac::SetLteCcmMacSapUser(LteCcmMacSapUser* s)
{
    m_ccmMacSapUser = s;
}

LteCcmMacSapProvider*
MmWaveEnbMac::GetLteCcmMacSapProvider()
{
    return m_ccmMacSapProvider;
}

void
MmWaveEnbMac::SetPhySapProvider(MmWavePhySapProvider* ptr)
{
    m_phySapProvider = ptr;
}

MmWaveMacSchedSapUser*
MmWaveEnbMac::GetMmWaveMacSchedSapUser()
{
    return m_macSchedSapUser;
}

void
MmWaveEnbMac::SetMmWaveMacSchedSapProvider(MmWaveMacSchedSapProvider* ptr)
{
    m_macSchedSapProvider = ptr;
}

MmWaveMacCschedSapUser*
MmWaveEnbMac::GetMmWaveMacCschedSapUser()
{
    return m_macCschedSapUser;
}

void
MmWaveEnbMac::SetMmWaveMacCschedSapProvider(MmWaveMacCschedSapProvider* ptr)
{
    m_macCschedSapProvider = ptr;
}

void
MmWaveEnbMac::DoUlCqiReport(MmWaveMacSchedSapProvider::SchedUlCqiInfoReqParameters ulcqi)
{
    if (ulcqi.m_ulCqi.m_type == UlCqiInfo::PUSCH)
    {
        NS_LOG_DEBUG(this << " eNB rxed an PUSCH UL-CQI");
    }
    else if (ulcqi.m_ulCqi.m_type == UlCqiInfo::SRS)
    {
        NS_LOG_DEBUG(this << " eNB rxed an SRS UL-CQI");
    }
    // ulcqi.m_sfnSf = SfnSf(m_frameNum, m_sfNum, m_slotNum);
    NS_LOG_INFO("*** UL CQI report SINR "
                << LteFfConverter::fpS11dot3toDouble(ulcqi.m_ulCqi.m_sinr[0]) << " frame "
                << m_frameNum << " subframe " << m_sfNum << " slot " << m_slotNum);

    m_ulCqiReceived.push_back(ulcqi);
}

void
MmWaveEnbMac::DoReceiveControlMessage(Ptr<MmWaveControlMessage> msg)
{
    NS_LOG_FUNCTION(this << msg);

    switch (msg->GetMessageType())
    {
    case (MmWaveControlMessage::DL_CQI): {
        Ptr<MmWaveDlCqiMessage> cqi = DynamicCast<MmWaveDlCqiMessage>(msg);
        DlCqiInfo cqiElement = cqi->GetDlCqi();
        NS_ASSERT(cqiElement.m_rnti != 0);
        m_dlCqiReceived.push_back(cqiElement);
        break;
    }
    case (MmWaveControlMessage::BSR): {
        Ptr<MmWaveBsrMessage> bsr = DynamicCast<MmWaveBsrMessage>(msg);
        MacCeElement macCeElement = bsr->GetBsr();

        // Conversion from MacCeElement to MacCeListElement_s needed to use the lte CCM-MAC SAP
        MacCeValue_u macCeValue;
        macCeValue.m_phr = macCeElement.m_macCeValue.m_phr;
        macCeValue.m_crnti = macCeElement.m_macCeValue.m_crnti;
        macCeValue.m_bufferStatus = macCeElement.m_macCeValue.m_bufferStatus;

        MacCeListElement_s macCeListElement;
        macCeListElement.m_rnti = macCeElement.m_rnti;
        macCeListElement.m_macCeType = MacCeListElement_s::BSR;
        macCeListElement.m_macCeValue = macCeValue;

        m_ccmMacSapUser->UlReceiveMacCe(macCeListElement, m_componentCarrierId);

        break;
    }
    case (MmWaveControlMessage::DL_HARQ): {
        Ptr<MmWaveDlHarqFeedbackMessage> dlharq = DynamicCast<MmWaveDlHarqFeedbackMessage>(msg);
        DoDlHarqFeedback(dlharq->GetDlHarqFeedback());
        break;
    }
    default:
        NS_LOG_INFO("Control message not supported/expected");
    }
}

void
MmWaveEnbMac::DoReportMacCeToScheduler(MacCeListElement_s bsr)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG(this << " bsr Size " << (uint16_t)m_ulCeReceived.size());

    // Conversion from MacCeListElement_s to MacCeElement
    MacCeElement macCeElement;
    macCeElement.m_rnti = bsr.m_rnti;
    macCeElement.m_macCeType = MacCeElement::BSR;

    MacCeValue macCeValue;
    macCeValue.m_phr = bsr.m_macCeValue.m_phr;
    macCeValue.m_crnti = bsr.m_macCeValue.m_crnti;
    macCeValue.m_bufferStatus = bsr.m_macCeValue.m_bufferStatus;
    macCeElement.m_macCeValue = macCeValue;

    // send to LteCcmMacSapUser
    m_ulCeReceived.push_back(
        macCeElement); // this to called when LteUlCcmSapProvider::ReportMacCeToScheduler is called
    NS_LOG_DEBUG(this << " bsr Size after push_back " << (uint16_t)m_ulCeReceived.size());
}

void
MmWaveEnbMac::DoUlHarqFeedback(UlHarqInfo params)
{
    NS_LOG_FUNCTION(this);
    m_ulHarqInfoReceived.push_back(params);
}

void
MmWaveEnbMac::DoDlHarqFeedback(DlHarqInfo params)
{
    NS_LOG_FUNCTION(this);
    // Update HARQ buffer
    std::map<uint16_t, MmWaveDlHarqProcessesBuffer_t>::iterator it =
        m_miDlHarqProcessesPackets.find(params.m_rnti);
    NS_ASSERT(it != m_miDlHarqProcessesPackets.end());

    if (params.m_harqStatus == DlHarqInfo::ACK)
    {
        // discard buffer
        Ptr<PacketBurst> emptyBuf = CreateObject<PacketBurst>();
        (*it).second.at(params.m_harqProcessId).m_pktBurst = emptyBuf;
        NS_LOG_DEBUG(this << " HARQ-ACK UE " << params.m_rnti << " harqId "
                          << (uint16_t)params.m_harqProcessId);
    }
    else if (params.m_harqStatus == DlHarqInfo::NACK)
    {
        /*if (params.m_numRetx == 3)
        {
                std::map <uint16_t, std::map<uint8_t, LteMacSapUser*> >::iterator rntiIt =
        m_rlcAttached.find (params.m_rnti); for (unsigned i = 0; i < (*it).second.at
        (params.m_harqProcessId).m_lcidList.size (); i++)
                {
                                std::map<uint8_t, LteMacSapUser*>::iterator lcidIt =
                                                rntiIt->second.find ((*it).second.at
        (params.m_harqProcessId).m_lcidList[i]); NS_ASSERT (lcidIt != rntiIt->second.end ());
                                lcidIt->second->NotifyDlHarqDeliveryFailure
        (params.m_harqProcessId);
                }
        }*/
        NS_LOG_DEBUG(this << " HARQ-NACK UE " << params.m_rnti << " harqId "
                          << (uint16_t)params.m_harqProcessId);
    }
    else
    {
        NS_FATAL_ERROR(" HARQ functionality not implemented");
    }

    m_dlHarqInfoReceived.push_back(params);
}

void
MmWaveEnbMac::DoReportBufferStatus(LteMacSapProvider::ReportBufferStatusParameters params)
{
    NS_LOG_FUNCTION(this);
    MmWaveMacSchedSapProvider::SchedDlRlcBufferReqParameters schedParams;
    schedParams.m_logicalChannelIdentity = params.lcid;
    schedParams.m_rlcRetransmissionHolDelay = params.retxQueueHolDelay;
    schedParams.m_rlcRetransmissionQueueSize = params.retxQueueSize;
    schedParams.m_rlcStatusPduSize = params.statusPduSize;
    schedParams.m_rlcTransmissionQueueHolDelay = params.txQueueHolDelay;
    schedParams.m_rlcTransmissionQueueSize = params.txQueueSize;
    schedParams.m_rnti = params.rnti;

    schedParams.m_txPacketSizes = params.txPacketSizes;
    schedParams.m_txPacketDelays = params.txPacketDelays;
    schedParams.m_retxPacketSizes = params.retxPacketSizes;
    schedParams.m_retxPacketDelays = params.retxPacketDelays;
    schedParams.m_arrivalRate = params.arrivalRate;

    NS_LOG_LOGIC("ReportBufferStatus for lcid " << (uint16_t)params.lcid << " rnti " << params.rnti
                                                << " txPacketSizes "
                                                << params.txPacketSizes.size());

    m_macSchedSapProvider->SchedDlRlcBufferReq(schedParams);
}

// forwarded from LteMacSapProvider
void 
MmWaveEnbMac::DoTransmitPdu (LteMacSapProvider::TransmitPduParameters params)
{
    NS_LOG_FUNCTION (this);
    if (params.rnti >= 1000) {
        m_multicastQueue.push(params.pdu->Copy()); // 창고 입고
        // NS_LOG_UNCOND ("MAC Queue: Stored! Current count: " << m_multicastQueue.size());
        return; 
    }
    // ---------------- [연구용 멀티캐스트 직통 로직] ----------------
    // if (params.rnti >= 1000) 
    // {
    //     // 1. 순수 데이터 복사
    //     Ptr<Packet> p = params.pdu->Copy(); 
    //     uint32_t payloadSize = p->GetSize();

    //     // 2. MAC 헤더 및 서브헤더 생성 (알맹이 포장)
    //     MmWaveMacPduHeader macHeader;
    //     // [핵심] 서브헤더에 데이터 크기를 명시해야 UE가 읽을 수 있습니다.
    //     MacSubheader subHeader (params.lcid, payloadSize);
    //     macHeader.AddSubheader (subHeader);

    //     // 3. 패킷 맨 앞에 헤더 부착
    //     p->AddHeader (macHeader); 

    //     // 4. [매우 중요] 시간 태그 설정
    //     // MAC의 현재 카운터가 아니라, 스케줄러가 전송을 허용한 그 시점의 태그를 찾아야 합니다.
    //     // 일단 현재 로직에서는 m_frameNum 등을 쓰되, 
    //     // 만약 계속 2바이트가 뜬다면 태그 설정 없이 쏴보는 테스트가 필요합니다.
    //     MmWaveMacPduTag pduTag;
    //     pduTag.SetSfn (SfnSf (m_frameNum, m_sfNum, m_slotNum)); 
    //     p->AddPacketTag (pduTag);

    //     LteRadioBearerTag radioTag;
    //     radioTag.SetRnti (params.rnti);
    //     radioTag.SetLcid (params.lcid);
    //     p->AddPacketTag (radioTag);

    //     // 5. 발사
    //     m_phySapProvider->SendMacPdu (p);
        
    //     NS_LOG_UNCOND ("DEBUG: eNB MAC 발송 완료! 크기(헤더포함): " << p->GetSize());
    //     return; 
    // }

    // ---------------- [기존 ns-3 표준 유니캐스트 로직] ----------------
    // // 원래 코드의 Map 기반 전송 로직입니다. (삭제하지 않음)
    // uint32_t tbMapKey = ((params.rnti & 0xFFFF) << 8) | (params.harqProcessId & 0xFF);
    // auto it = m_macPduMap.find(tbMapKey);

    // if (it == m_macPduMap.end()) 
    // {
    //     NS_FATAL_ERROR("No MAC PDU storage element found for RNTI " << params.rnti);
    // } 
    // else 
    // {
    //     if (!it->second.m_pdu) 
    //     { 
    //         it->second.m_pdu = params.pdu; 
    //     }
    //     else 
    //     { 
    //         it->second.m_pdu->AddAtEnd(params.pdu); 
    //     }
        
    //     MacSubheader subheader(params.lcid, params.pdu->GetSize());
    //     it->second.m_macHeader.AddSubheader(subheader);
    //     it->second.m_numRlcPdu++;
    // }
}

void
MmWaveEnbMac::DoSchedConfigIndication(MmWaveMacSchedSapUser::SchedConfigIndParameters ind)
{
    TraceSchedInfo(ind);
    m_phySapProvider->SetSlotAllocInfo(ind.m_slotAllocInfo);
    LteMacSapUser::TxOpportunityParameters txOpParams;
    uint32_t subHdrSize = 4; 
    
    // [중요] 이번 슬롯에서 실제 데이터 전송이 일어났는지 추적하는 플래그
    bool multicastSentInThisSlot = false;

    for (unsigned iTti = 0; iTti < ind.m_slotAllocInfo.m_ttiAllocInfo.size(); iTti++)
    {
        TtiAllocInfo& ttiAllocInfo = ind.m_slotAllocInfo.m_ttiAllocInfo[iTti];
        if (ttiAllocInfo.m_ttiType != TtiAllocInfo::CTRL &&
            ttiAllocInfo.m_tddMode == TtiAllocInfo::DL_slotAllocInfo)
        {
            uint16_t rnti = ttiAllocInfo.m_dci.m_rnti;
            // std::cout << "fuck" << rnti <<std::endl;
            // ---------------- [A] 멀티캐스트 처리 로직 (1000 이상) ----------------
            if (rnti >= 1000)
{
    uint32_t currentBeamSector = g_activeSectors[g_activeIdx];
    uint16_t targetRnti = 1001 + currentBeamSector;

    if (rnti == targetRnti) 
{
    if (!m_multicastQueue.empty()) 
    {
        Ptr<Packet> p = m_multicastQueue.front()->Copy(); 
        
        // 1. 헤더 입히기
        MmWaveMacPduHeader macHeader;
        macHeader.AddSubheader (MacSubheader (3, p->GetSize())); 
        p->AddHeader (macHeader);

        // 2. [필수!!!] 타이밍 태그가 없으면 UE MAC에 로그가 안 뜹니다.
        MmWaveMacPduTag timingTag (ind.m_sfnSf); 
        p->AddPacketTag (timingTag);
        
        // 3. RNTI 태그
        LteRadioBearerTag radioTag;
        radioTag.SetRnti (rnti); 
        p->AddPacketTag (radioTag);

        m_phySapProvider->SendMacPdu (p);
    }
}
    continue; 
}

            // ---------------- [B] 일반 유니캐스트 처리 로직 ----------------
            auto rntiIt = m_rlcAttached.find(rnti);
            if (rntiIt != m_rlcAttached.end())
            {
                DciInfoElementTdma& dciElem = ttiAllocInfo.m_dci;
                uint8_t tbUid = dciElem.m_harqProcess;

                if (dciElem.m_ndi == 1) // 신규 데이터인 경우
                {
                    std::vector<RlcPduInfo>& rlcPduInfo = ttiAllocInfo.m_rlcPduInfo;
                    uint32_t tbMapKey = ((rnti & 0xFFFF) << 8) | (tbUid & 0xFF);
                    
                    // MAC PDU 맵에 전송 정보 저장 (RLC 응답 대기용)
                    MacPduInfo macPduInfo(ind.m_sfnSf, ttiAllocInfo.m_dci.m_tbSize, rlcPduInfo.size(), dciElem);
                    m_macPduMap.insert(std::make_pair(tbMapKey, macPduInfo));

                    // RLC 레이어에 데이터 요청
                    for (unsigned int ipdu = 0; ipdu < rlcPduInfo.size(); ipdu++) {
                        auto lcidIt = rntiIt->second.find(rlcPduInfo[ipdu].m_lcid);
                        if (lcidIt != rntiIt->second.end()) {
                            txOpParams.bytes = rlcPduInfo[ipdu].m_size - subHdrSize;
                            txOpParams.rnti = rnti;
                            txOpParams.lcid = rlcPduInfo[ipdu].m_lcid;
                            txOpParams.harqId = tbUid;
                            txOpParams.componentCarrierId = m_componentCarrierId;
                            (*lcidIt).second->NotifyTxOpportunity(txOpParams);
                        }
                    }

                    // 수집된 데이터를 PHY로 전송
                    auto pduMapIt = m_macPduMap.find(tbMapKey);
                    if (pduMapIt != m_macPduMap.end() && pduMapIt->second.m_pdu) {
                        Ptr<Packet> upkt = pduMapIt->second.m_pdu;
                        upkt->AddHeader(pduMapIt->second.m_macHeader);
                        
                        // [에러 해결] 유니캐스트 패킷에도 반드시 타이밍 태그를 부착해야 크래시가 안 납니다.
                        upkt->AddPacketTag(MmWaveMacPduTag(ind.m_sfnSf)); 
                        upkt->AddPacketTag(LteRadioBearerTag(rnti, pduMapIt->second.m_size, 0));
                        
                        m_phySapProvider->SendMacPdu(upkt);
                    }
                    m_macPduMap.erase(tbMapKey);
                }
            }
        }
    }

    // [중요] 모든 TTI 루프가 끝난 뒤, 이번 슬롯에서 실제 전송이 일어났다면 큐에서 패킷 제거
    if (multicastSentInThisSlot && !m_multicastQueue.empty()) 
    {
        m_multicastQueue.pop(); 
        NS_LOG_UNCOND ("MAC Queue POP: 배송 완료 후 패킷 제거. 남은 패킷: " << m_multicastQueue.size());
    }
}


uint8_t
MmWaveEnbMac::AllocateTbUid(void)
{
    return m_tbUid++;
}

void
MmWaveEnbMac::TraceSchedInfo(MmWaveMacSchedSapUser::SchedConfigIndParameters ind)
{
    MmWaveSchedTraceInfo mmWaveSchedInfo;
    mmWaveSchedInfo.m_indParam = ind;
    mmWaveSchedInfo.m_ccId = m_componentCarrierId;

    m_schedEnbInfo(mmWaveSchedInfo);
}

// ////////////////////////////////////////////
// CMAC SAP
// ////////////////////////////////////////////

void
MmWaveEnbMac::DoConfigureMac(uint8_t ulBandwidth, uint8_t dlBandwidth)
{
    NS_LOG_FUNCTION(this << " ulBandwidth=" << (uint16_t)ulBandwidth
                         << " dlBandwidth=" << (uint16_t)dlBandwidth);
    MmWaveMacCschedSapProvider::CschedCellConfigReqParameters params;
    // Configure the subset of parameters used by FfMacScheduler
    params.m_ulBandwidth = ulBandwidth;
    params.m_dlBandwidth = dlBandwidth;
    // m_macChTtiDelay = m_phySapProvider->GetMacChTtiDelay ();  // Gets set by MmWavePhyMacCommon
    m_macCschedSapProvider->CschedCellConfigReq(params);
}

void
MmWaveEnbMac::DoAddUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << " DoAddUe rnti=" << rnti);
    std::map<uint8_t, LteMacSapUser*> empty;
    std::pair<std::map<uint16_t, std::map<uint8_t, LteMacSapUser*>>::iterator, bool> ret =
        m_rlcAttached.insert(std::pair<uint16_t, std::map<uint8_t, LteMacSapUser*>>(rnti, empty));
    NS_ASSERT_MSG(ret.second, "element already present, RNTI already existed");
    // m_associatedUe.push_back (rnti);

    MmWaveMacCschedSapProvider::CschedUeConfigReqParameters params;
    params.m_rnti = rnti;
    params.m_transmissionMode =
        0; // set to default value (SISO) for avoiding random initialization (valgrind error)
    m_macCschedSapProvider->CschedUeConfigReq(params);

    // Create DL transmission HARQ buffers
    MmWaveDlHarqProcessesBuffer_t buf;
    uint16_t harqNum = m_phyMacConfig->GetNumHarqProcess();
    buf.resize(harqNum);
    for (uint8_t i = 0; i < harqNum; i++)
    {
        Ptr<PacketBurst> pb = CreateObject<PacketBurst>();
        buf.at(i).m_pktBurst = pb;
    }
    m_miDlHarqProcessesPackets.insert(
        std::pair<uint16_t, MmWaveDlHarqProcessesBuffer_t>(rnti, buf));
}

void
MmWaveEnbMac::DoRemoveUe(uint16_t rnti)
{
    NS_LOG_FUNCTION(this << " rnti=" << rnti);
    MmWaveMacCschedSapProvider::CschedUeReleaseReqParameters params;
    params.m_rnti = rnti;
    m_macCschedSapProvider->CschedUeReleaseReq(params);
    m_miDlHarqProcessesPackets.erase(rnti);
    // for(std::vector<UlHarqInfo>::iterator iter = m_ulHarqInfoReceived.begin(); iter !=
    // m_ulHarqInfoReceived.end(); ++iter)
    // {
    //    if(iter->m_rnti == rnti)
    //    {
    //            iter = m_ulHarqInfoReceived.erase(iter);
    //    }
    // }
    m_rlcAttached.erase(rnti);
}

void
MmWaveEnbMac::DoAddLc(LteEnbCmacSapProvider::LcInfo lcinfo, LteMacSapUser* msu)
{
    NS_LOG_FUNCTION(this);
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Add LC for lcid " << lcinfo.lcId);

    std::map<LteFlowId_t, LteMacSapUser*>::iterator it;

    LteFlowId_t flow(lcinfo.rnti, lcinfo.lcId);

    std::map<uint16_t, std::map<uint8_t, LteMacSapUser*>>::iterator rntiIt =
        m_rlcAttached.find(lcinfo.rnti);
    NS_ASSERT_MSG(rntiIt != m_rlcAttached.end(), "RNTI not found");
    std::map<uint8_t, LteMacSapUser*>::iterator lcidIt = rntiIt->second.find(lcinfo.lcId);
    if (lcidIt == rntiIt->second.end())
    {
        rntiIt->second.insert(std::pair<uint8_t, LteMacSapUser*>(lcinfo.lcId, msu));
    }
    else
    {
        NS_LOG_INFO("LC already exists");
    }

    // CCCH (LCID 0) is pre-configured
    // see FF LTE MAC Scheduler
    // Interface Specification v1.11,
    // 4.3.4 logicalChannelConfigListElement
    if (true) //(lcinfo.lcId != 0)
    {
        struct MmWaveMacCschedSapProvider::CschedLcConfigReqParameters params;
        params.m_rnti = lcinfo.rnti;
        params.m_reconfigureFlag = false;

        struct LogicalChannelConfigListElement_s lccle;
        lccle.m_logicalChannelIdentity = lcinfo.lcId;
        lccle.m_logicalChannelGroup = lcinfo.lcGroup;
        lccle.m_direction = LogicalChannelConfigListElement_s::DIR_BOTH;
        lccle.m_qosBearerType = lcinfo.isGbr ? LogicalChannelConfigListElement_s::QBT_GBR
                                             : LogicalChannelConfigListElement_s::QBT_NON_GBR;
        lccle.m_qci = lcinfo.qci;
        lccle.m_eRabMaximulBitrateUl = lcinfo.mbrUl;
        lccle.m_eRabMaximulBitrateDl = lcinfo.mbrDl;
        lccle.m_eRabGuaranteedBitrateUl = lcinfo.gbrUl;
        lccle.m_eRabGuaranteedBitrateDl = lcinfo.gbrDl;
        params.m_logicalChannelConfigList.push_back(lccle);

        m_macCschedSapProvider->CschedLcConfigReq(params);
    }
}

void
MmWaveEnbMac::DoReconfigureLc(LteEnbCmacSapProvider::LcInfo lcinfo)
{
    NS_FATAL_ERROR("not implemented");
}

void
MmWaveEnbMac::DoReleaseLc(uint16_t rnti, uint8_t lcid)
{
    // Find user based on rnti and then erase lcid stored against the same
    NS_LOG_INFO("ReleaseLc");
    std::map<uint16_t, std::map<uint8_t, LteMacSapUser*>>::iterator rntiIt =
        m_rlcAttached.find(rnti);
    rntiIt->second.erase(lcid);

    struct MmWaveMacCschedSapProvider::CschedLcReleaseReqParameters params;
    params.m_rnti = rnti;
    params.m_logicalChannelIdentity.push_back(lcid);
    m_macCschedSapProvider->CschedLcReleaseReq(params);
}

void
MmWaveEnbMac::UeUpdateConfigurationReq(LteEnbCmacSapProvider::UeConfig params)
{
    NS_LOG_FUNCTION(this);
    // propagates to scheduler
    MmWaveMacCschedSapProvider::CschedUeConfigReqParameters req;
    req.m_rnti = params.m_rnti;
    req.m_transmissionMode = params.m_transmissionMode;
    req.m_reconfigureFlag = true;
    m_macCschedSapProvider->CschedUeConfigReq(req);
}

LteEnbCmacSapProvider::RachConfig
MmWaveEnbMac::DoGetRachConfig()
{
    struct LteEnbCmacSapProvider::RachConfig rc;
    rc.numberOfRaPreambles = m_numberOfRaPreambles;
    rc.preambleTransMax = m_preambleTransMax;
    rc.raResponseWindowSize = m_raResponseWindowSize;
    return rc;
}

LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue
MmWaveEnbMac::DoAllocateNcRaPreamble(uint16_t rnti)
{
    bool found = false;
    uint8_t preambleId;
    for (preambleId = m_numberOfRaPreambles; preambleId < 64; ++preambleId)
    {
        std::map<uint8_t, NcRaPreambleInfo>::iterator it =
            m_allocatedNcRaPreambleMap.find(preambleId);
        if ((it == m_allocatedNcRaPreambleMap.end()) || (it->second.expiryTime < Simulator::Now()))
        {
            found = true;
            NcRaPreambleInfo preambleInfo;
            uint32_t expiryIntervalMs =
                (uint32_t)m_preambleTransMax * ((uint32_t)m_raResponseWindowSize + 5);

            preambleInfo.expiryTime = Simulator::Now() + MilliSeconds(expiryIntervalMs);
            preambleInfo.rnti = rnti;
            NS_LOG_INFO("allocated preamble for NC based RA: preamble "
                        << (uint16_t)preambleId << ", RNTI " << preambleInfo.rnti << ", exiryTime "
                        << preambleInfo.expiryTime);
            m_allocatedNcRaPreambleMap[preambleId] =
                preambleInfo; // create if not exist, update otherwise
            break;
        }
    }
    LteEnbCmacSapProvider::AllocateNcRaPreambleReturnValue ret;
    if (found)
    {
        ret.valid = true;
        ret.raPreambleId = preambleId;
        ret.raPrachMaskIndex = 0;
    }
    else
    {
        ret.valid = false;
        ret.raPreambleId = 0;
        ret.raPrachMaskIndex = 0;
    }
    return ret;
}

// ////////////////////////////////////////////
// CSCHED SAP
// ////////////////////////////////////////////

void
MmWaveEnbMac::DoCschedCellConfigCnf(MmWaveMacCschedSapUser::CschedCellConfigCnfParameters params)
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbMac::DoCschedUeConfigCnf(MmWaveMacCschedSapUser::CschedUeConfigCnfParameters params)
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbMac::DoCschedLcConfigCnf(MmWaveMacCschedSapUser::CschedLcConfigCnfParameters params)
{
    NS_LOG_FUNCTION(this);
    // Call the CSCHED primitive
    // m_cschedSap->LcConfigCompleted();
}

void
MmWaveEnbMac::DoCschedLcReleaseCnf(MmWaveMacCschedSapUser::CschedLcReleaseCnfParameters params)
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbMac::DoCschedUeReleaseCnf(MmWaveMacCschedSapUser::CschedUeReleaseCnfParameters params)
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbMac::DoCschedUeConfigUpdateInd(
    MmWaveMacCschedSapUser::CschedUeConfigUpdateIndParameters params)
{
    NS_LOG_FUNCTION(this);
    // propagates to RRC
    LteEnbCmacSapUser::UeConfig ueConfigUpdate;
    ueConfigUpdate.m_rnti = params.m_rnti;
    ueConfigUpdate.m_transmissionMode = params.m_transmissionMode;
    m_cmacSapUser->RrcConfigurationUpdateInd(ueConfigUpdate);
}

void
MmWaveEnbMac::DoCschedCellConfigUpdateInd(
    MmWaveMacCschedSapUser::CschedCellConfigUpdateIndParameters params)
{
    NS_LOG_FUNCTION(this);
}

} // namespace mmwave

} // namespace ns3
