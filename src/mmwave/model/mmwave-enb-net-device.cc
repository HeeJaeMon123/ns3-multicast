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
 *       Modified by: Tommaso Zugno <tommasozugno@gmail.com>
 *                               Integration of Carrier Aggregation
 */

#include "mmwave-enb-net-device.h"

#include "mmwave-net-device.h"
#include "mmwave-ue-net-device.h"
#include "ns3/mmwave-mac-pdu-header.h" 
#include "ns3/eps-bearer-tag.h"

#include <ns3/abort.h>
#include <ns3/callback.h>
#include <ns3/enum.h>
#include <ns3/ipv4-l3-protocol.h>
#include <ns3/ipv6-l3-protocol.h>
#include <ns3/llc-snap-header.h>
#include <ns3/log.h>
#include <ns3/lte-enb-component-carrier-manager.h>
#include <ns3/lte-enb-rrc.h>
#include <ns3/mmwave-component-carrier-enb.h>
#include <ns3/node.h>
#include <ns3/packet-burst.h>
#include <ns3/packet.h>
#include <ns3/pointer.h>
#include <ns3/simulator.h>
#include <ns3/trace-source-accessor.h>
#include <ns3/uinteger.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MmWaveEnbNetDevice");

namespace mmwave
{

NS_OBJECT_ENSURE_REGISTERED(MmWaveEnbNetDevice);

TypeId
MmWaveEnbNetDevice::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::MmWaveEnbNetDevice")
            .SetParent<MmWaveNetDevice>()
            .AddConstructor<MmWaveEnbNetDevice>()
            .AddAttribute("LteEnbComponentCarrierManager",
                          "The ComponentCarrierManager associated to this EnbNetDevice",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_componentCarrierManager),
                          MakePointerChecker<LteEnbComponentCarrierManager>())
            .AddAttribute("LteEnbRrc",
                          "The RRC layer associated with the ENB",
                          PointerValue(),
                          MakePointerAccessor(&MmWaveEnbNetDevice::m_rrc),
                          MakePointerChecker<LteEnbRrc>())
            .AddAttribute("CellId",
                          "Cell Identifier",
                          UintegerValue(0),
                          MakeUintegerAccessor(&MmWaveEnbNetDevice::m_cellId),
                          MakeUintegerChecker<uint16_t>());
    return tid;
}

MmWaveEnbNetDevice::MmWaveEnbNetDevice()
    //: m_cellId(0),
    // m_Bandwidth (72),
    // m_Earfcn(1),
    : m_componentCarrierManager(0),
      m_isConfigured(false)
{
    NS_LOG_FUNCTION(this);
}

MmWaveEnbNetDevice::~MmWaveEnbNetDevice()
{
    NS_LOG_FUNCTION(this);
}

void
MmWaveEnbNetDevice::DoInitialize(void)
{
    NS_LOG_FUNCTION(this);
    m_isConstructed = true;
    UpdateConfig();
    for (auto it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
    {
        it->second->Initialize();
    }
    m_rrc->Initialize();
    m_componentCarrierManager->Initialize();
}

void
MmWaveEnbNetDevice::DoDispose()
{
    NS_LOG_FUNCTION(this);

    m_rrc->Dispose();
    m_rrc = 0;

    m_componentCarrierManager->Dispose();
    m_componentCarrierManager = 0;
    // MmWaveComponentCarrierEnb::DoDispose() will call DoDispose
    // of its PHY, MAC, FFR and scheduler instance
    for (uint32_t i = 0; i < m_ccMap.size(); i++)
    {
        m_ccMap.at(i)->Dispose();
        m_ccMap.at(i) = 0;
    }

    MmWaveNetDevice::DoDispose();
}

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy(void) const
{
    NS_LOG_FUNCTION(this);
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(0))->GetPhy();
}

Ptr<MmWaveEnbPhy>
MmWaveEnbNetDevice::GetPhy(uint8_t index)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(index))->GetPhy();
}

uint16_t
MmWaveEnbNetDevice::GetCellId() const
{
    NS_LOG_FUNCTION(this);
    return m_cellId;
}

bool
MmWaveEnbNetDevice::HasCellId(uint16_t cellId) const
{
    for (auto& it : m_ccMap)
    {
        if (DynamicCast<MmWaveComponentCarrierEnb>(it.second)->GetCellId() == cellId)
        {
            return true;
        }
    }
    return false;
}

uint8_t
MmWaveEnbNetDevice::GetBandwidth() const
{
    NS_LOG_FUNCTION(this);
    return m_Bandwidth;
}

void
MmWaveEnbNetDevice::SetBandwidth(uint8_t bw)
{
    NS_LOG_FUNCTION(this << bw);
    m_Bandwidth = bw;
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac(void)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(0))->GetMac();
}

Ptr<MmWaveEnbMac>
MmWaveEnbNetDevice::GetMac(uint8_t index)
{
    return DynamicCast<MmWaveComponentCarrierEnb>(m_ccMap.at(index))->GetMac();
}

void
MmWaveEnbNetDevice::SetRrc(Ptr<LteEnbRrc> rrc)
{
    m_rrc = rrc;
}

Ptr<LteEnbRrc>
MmWaveEnbNetDevice::GetRrc(void)
{
    return m_rrc;
}


bool
MmWaveEnbNetDevice::DoSend(Ptr<Packet> packet, const Address& dest, uint16_t protocolNumber)
{
    NS_LOG_FUNCTION(this << packet << dest << protocolNumber);

    // [필수] IPv4(0x0800) 패킷일 때만 IP 헤더를 분석합니다.
    if (protocolNumber == 0x0800) 
    {
        Ipv4Header ipHeader;
        packet->PeekHeader(ipHeader); 

        // 멀티캐스트 주소(224.0.1.1 등)인 경우 연구용 직통 경로 실행
        if (ipHeader.GetDestination().IsMulticast())
        {
            NS_LOG_UNCOND("[T: " << Simulator::Now().GetSeconds() << "s] eNB NetDevice: 멀티캐스트 감지 -> MAC 직송");

            Ptr<MmWaveEnbMac> enbMac = this->GetMac();
            if (enbMac) 
            {
                // QoS 태그 부착 (RNTI 1001, BID 3)
                EpsBearerTag bearerTag (1001, 3); 
                packet->AddPacketTag (bearerTag);

                // MAC 전송 파라미터 설정
                LteMacSapProvider::TransmitPduParameters params;
                params.pdu = packet; 
                params.rnti = 1001;
                params.lcid = 3;
                params.harqProcessId = 0;
                params.layer = 0; 
                params.componentCarrierId = 0; 

                // MAC으로 직접 던지고 함수 종료 (RRC 우회)
                enbMac->DoTransmitPdu(params);
                return true; 
            }
        }
    }

    // --- 원래 ns-3 표준 로직 (삭제 금지) ---
    // 멀티캐스트가 아니거나 위 조건에 해당하지 않는 모든 패킷은 기존처럼 RRC가 처리합니다.
    NS_ABORT_MSG_IF(protocolNumber != Ipv4L3Protocol::PROT_NUMBER &&
                    protocolNumber != Ipv6L3Protocol::PROT_NUMBER,
                    "unsupported protocol " << protocolNumber);
    
    return m_rrc->SendData(packet);
}
void
MmWaveEnbNetDevice::UpdateConfig(void)
{
    NS_LOG_FUNCTION(this);

    if (m_isConstructed)
    {
        if (!m_isConfigured)
        {
            NS_LOG_LOGIC(this << " Configure cell " << m_cellId);
            // we have to make sure that this function is called only once
            // m_rrc->ConfigureCell (m_Bandwidth, m_Bandwidth, m_Earfcn, m_Earfcn, m_cellId);
            NS_ASSERT(!m_ccMap.empty());

            // create the MmWaveComponentCarrierConf map used for the RRC setup
            std::map<uint8_t, LteEnbRrc::MmWaveComponentCarrierConf> ccConfMap;
            for (auto it = m_ccMap.begin(); it != m_ccMap.end(); ++it)
            {
                Ptr<MmWaveComponentCarrierEnb> ccEnb =
                    DynamicCast<MmWaveComponentCarrierEnb>(it->second);
                LteEnbRrc::MmWaveComponentCarrierConf ccConf;
                ccConf.m_ccId = ccEnb->GetConfigurationParameters()->GetCcId();
                ccConf.m_cellId = ccEnb->GetCellId();
                ccConf.m_bandwidth = ccEnb->GetBandwidthInRb();

                ccConfMap[it->first] = ccConf;
            }

            m_rrc->ConfigureCell(ccConfMap);
            m_isConfigured = true;
        }

        // m_rrc->SetCsgId (m_csgId, m_csgIndication);
    }
    else
    {
        /*
         * Lower layers are not ready yet, so do nothing now and expect
         * ``DoInitialize`` to re-invoke this function.
         */
    }
}

void
MmWaveEnbNetDevice::SetCcMap(std::map<uint8_t, Ptr<MmWaveComponentCarrier>> ccm)
{
    NS_ASSERT_MSG(!m_isConfigured, "attempt to set CC map after configuration");
    m_ccMap = ccm;
}

} // namespace mmwave
} // namespace ns3
