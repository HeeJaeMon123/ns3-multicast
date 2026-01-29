/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef PAPER_CONE_ANTENNA_MODEL_H
#define PAPER_CONE_ANTENNA_MODEL_H

#include "ns3/antenna-model.h"
#include "ns3/angles.h"

namespace ns3 {

class PaperConeAntennaModel : public AntennaModel
{
public:
  static TypeId GetTypeId ();
  PaperConeAntennaModel () = default;          // override 제거
  ~PaperConeAntennaModel () override = default;

  double GetGainDb (Angles a) override;

private:
  double IntegrateArrayFactor (double nTh, double pTh, uint32_t steps) const;

  // Attributes
  uint32_t m_N {64};
  double   m_beta {0.0};
  double   m_k3db {2.782};
  bool     m_usePowerPattern {true};
  bool     m_useManualCone {false};
  double   m_manualHpbwDeg {10.0};
  double   m_manualGainDb {20.0};
  double   m_boresightAzimuthRad {0.0};
};

} // namespace ns3

#endif // PAPER_CONE_ANTENNA_MODEL_H
