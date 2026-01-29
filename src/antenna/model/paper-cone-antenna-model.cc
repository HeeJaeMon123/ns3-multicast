/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "paper-cone-antenna-model.h"

#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/log.h"

#include <cmath>
#include <algorithm>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("PaperConeAntennaModel");
NS_OBJECT_ENSURE_REGISTERED (PaperConeAntennaModel);

TypeId
PaperConeAntennaModel::GetTypeId ()
{
  static TypeId tid =
    TypeId ("ns3::PaperConeAntennaModel")
      .SetParent<AntennaModel> ()
      .SetGroupName ("Antenna")
      .AddConstructor<PaperConeAntennaModel> ()
      .AddAttribute ("N",
                     "Number Of Antenna Elements In Horizontal Plane.",
                     UintegerValue (64),
                     MakeUintegerAccessor (&PaperConeAntennaModel::m_N),
                     MakeUintegerChecker<uint32_t> (1))
      .AddAttribute ("Beta",
                     "Phase Excitation Difference (dimensionless).",
                     DoubleValue (0.0),
                     MakeDoubleAccessor (&PaperConeAntennaModel::m_beta),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("K3db",
                     "Paper Constant For 3-dB Points (e.g., 2.782 or 2.58).",
                     DoubleValue (2.782),
                     MakeDoubleAccessor (&PaperConeAntennaModel::m_k3db),
                     MakeDoubleChecker<double> (0.0))
      .AddAttribute ("UsePowerPattern",
                     "If true, use paper average-gain cone computed by integral.",
                     BooleanValue (true),
                     MakeBooleanAccessor (&PaperConeAntennaModel::m_usePowerPattern),
                     MakeBooleanChecker ())
      .AddAttribute ("UseManualCone",
                     "If true, override by ManualHpbwDeg/ManualGainDb.",
                     BooleanValue (false),
                     MakeBooleanAccessor (&PaperConeAntennaModel::m_useManualCone),
                     MakeBooleanChecker ())
      .AddAttribute ("ManualHpbwDeg",
                     "Manual Half-Power BeamWidth (deg) when UseManualCone=true.",
                     DoubleValue (10.0),
                     MakeDoubleAccessor (&PaperConeAntennaModel::m_manualHpbwDeg),
                     MakeDoubleChecker<double> (0.0))
      .AddAttribute ("ManualGainDb",
                     "Manual Mainlobe Gain (dB) when UseManualCone=true.",
                     DoubleValue (20.0),
                     MakeDoubleAccessor (&PaperConeAntennaModel::m_manualGainDb),
                     MakeDoubleChecker<double> ())
      .AddAttribute ("BoresightAzimuthRad",
                     "Boresight Azimuth (rad), steering in [-pi, pi).",
                     DoubleValue (0.0),
                     MakeDoubleAccessor (&PaperConeAntennaModel::m_boresightAzimuthRad),
                     MakeDoubleChecker<double> (-M_PI, M_PI));
  return tid;
}

double
PaperConeAntennaModel::IntegrateArrayFactor (double nTh, double pTh, uint32_t steps) const
{
  const double a = nTh;
  const double b = pTh;
  const double h = (b - a) / static_cast<double> (steps);

  auto fun = [this] (double t) -> double {
    const double c = std::cos (t);
    const double denomArg = (M_PI * c) / 2.0;
    const double numerArg = (static_cast<double> (m_N) * M_PI * c) / 2.0;

    const double denom = std::sin (denomArg);
    const double numer = std::sin (numerArg);

    if (std::abs (denom) < 1e-12)
      {
        return static_cast<double> (m_N);
      }
    return numer / denom;
  };

  double sum = 0.0;
  for (uint32_t i = 0; i <= steps; ++i)
    {
      const double t = a + h * static_cast<double> (i);
      const double w = (i == 0 || i == steps) ? 0.5 : 1.0;
      sum += w * fun (t);
    }
  return sum * h;
}

double
PaperConeAntennaModel::GetGainDb (Angles a)
{
  const double az = a.GetAzimuth ();
  const double dAz = std::abs (ns3::WrapToPi (az - m_boresightAzimuthRad));

  // We map offset to paper theta around pi/2
  const double t = (M_PI / 2.0) + dAz;

  if (m_useManualCone)
    {
      const double half = (m_manualHpbwDeg * M_PI / 180.0) / 2.0;
      if (dAz <= half)
        {
          return m_manualGainDb;
        }
      return -300.0;
    }

  if (!m_usePowerPattern)
    {
      return 0.0;
    }

  const double x = m_k3db / (static_cast<double> (m_N) * M_PI);
  const double argP = std::clamp (-m_beta + x, -1.0, 1.0);
  const double argN = std::clamp (-m_beta - x, -1.0, 1.0);

  const double pTh = std::acos (argP);
  const double nTh = std::acos (argN);
  const double lo = std::min (nTh, pTh);
  const double hi = std::max (nTh, pTh);

  if (t >= lo && t <= hi)
    {
      const double g = (hi - lo);
      if (g <= 1e-12)
        {
          return 0.0;
        }
      const double q = IntegrateArrayFactor (lo, hi, 4000);
      const double linearGain = q / g;

      if (linearGain <= 1e-15)
        {
          return -300.0;
        }
      return 10.0 * std::log10 (linearGain);
    }

  return -300.0;
}

} // namespace ns3
