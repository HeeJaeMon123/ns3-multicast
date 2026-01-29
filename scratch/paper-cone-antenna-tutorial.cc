#include "ns3/core-module.h"
#include "ns3/antenna-module.h"
#include "ns3/paper-cone-antenna-model.h"

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

using namespace ns3;

static inline double Deg2Rad (double deg) { return deg * M_PI / 180.0; }
static inline double Rad2Deg (double rad) { return rad * 180.0 / M_PI; }

int
main (int argc, char *argv[])
{
  uint32_t N = 64;
  double beta = 0.0;
  double k3db = 2.782;
  bool usePowerPattern = true;
  bool useManualCone = false;
  double manualHpbwDeg = 10.0;
  double manualGainDb = 20.0;
  double boresightAzDeg = 0.0;

  // scan options
  int maxOffsetDeg = 50;
  double stepDeg = 1.0;

  CommandLine cmd;
  cmd.AddValue ("N", "Number Of Elements", N);
  cmd.AddValue ("beta", "Beta", beta);
  cmd.AddValue ("k3db", "K3db", k3db);
  cmd.AddValue ("usePowerPattern", "Use Power Pattern", usePowerPattern);
  cmd.AddValue ("useManualCone", "Use Manual Cone", useManualCone);
  cmd.AddValue ("manualHpbwDeg", "Manual HPBW (deg)", manualHpbwDeg);
  cmd.AddValue ("manualGainDb", "Manual Gain (dB)", manualGainDb);
  cmd.AddValue ("boresightAzDeg", "Boresight Azimuth (deg)", boresightAzDeg);
  cmd.AddValue ("maxOffsetDeg", "Max offset scan (deg)", maxOffsetDeg);
  cmd.AddValue ("stepDeg", "Offset scan step (deg)", stepDeg);
  cmd.Parse (argc, argv);

  Ptr<PaperConeAntennaModel> ant = CreateObject<PaperConeAntennaModel> ();
  ant->SetAttribute ("N", UintegerValue (N));
  ant->SetAttribute ("Beta", DoubleValue (beta));
  ant->SetAttribute ("K3db", DoubleValue (k3db));
  ant->SetAttribute ("UsePowerPattern", BooleanValue (usePowerPattern));
  ant->SetAttribute ("UseManualCone", BooleanValue (useManualCone));
  ant->SetAttribute ("ManualHpbwDeg", DoubleValue (manualHpbwDeg));
  ant->SetAttribute ("ManualGainDb", DoubleValue (manualGainDb));
  ant->SetAttribute ("BoresightAzimuthRad", DoubleValue (Deg2Rad (boresightAzDeg)));

  // --- HPBW Debug Dump (print all intermediate values) ---
  const double Nd = static_cast<double> (N);
  const double x = k3db / (Nd * M_PI);

  auto clamp = [] (double v) { return std::max (-1.0, std::min (1.0, v)); };

  const double argP = clamp (-beta + x);
  const double argN = clamp (-beta - x);

  const double pTh = std::acos (argP);
  const double nTh = std::acos (argN);
  const double lo = std::min (pTh, nTh);
  const double hi = std::max (pTh, nTh);

  const double centerRad = M_PI / 2.0;
  const double dLo = std::abs (centerRad - lo);
  const double dHi = std::abs (hi - centerRad);

  const double halfHpbwRad = std::max (dLo, dHi);
  const double hpbwRad = 2.0 * halfHpbwRad;

  const double halfHpbwDeg = Rad2Deg (halfHpbwRad);
  const double hpbwDeg = Rad2Deg (hpbwRad);

  // "3-dB boundary offsets" in the azimuth-offset view
  const double left3dBOffsetDeg  = Rad2Deg (dLo);
  const double right3dBOffsetDeg = Rad2Deg (dHi);

  // often-used rough approximation (if you want to compare)
  const double approxHpbwDeg = 102.0 / Nd;

  std::cout << std::fixed << std::setprecision (6);
  std::cout << "---- HPBW Debug Dump ----\n";
  std::cout << "N=" << N << "\n";
  std::cout << "beta=" << beta << "\n";
  std::cout << "k3db=" << k3db << "\n";
  std::cout << "x=k3db/(N*pi)=" << x << "\n";
  std::cout << "argP=-beta+x=" << argP << "\n";
  std::cout << "argN=-beta-x=" << argN << "\n";
  std::cout << "pTh(rad)=" << pTh << ", pTh(deg)=" << Rad2Deg (pTh) << "\n";
  std::cout << "nTh(rad)=" << nTh << ", nTh(deg)=" << Rad2Deg (nTh) << "\n";
  std::cout << "lo(rad)=" << lo << ", lo(deg)=" << Rad2Deg (lo) << "\n";
  std::cout << "hi(rad)=" << hi << ", hi(deg)=" << Rad2Deg (hi) << "\n";
  std::cout << "center(pi/2)(deg)=90.000000\n";
  std::cout << "dLo=|center-lo|(deg)=" << left3dBOffsetDeg << "\n";
  std::cout << "dHi=|hi-center|(deg)=" << right3dBOffsetDeg << "\n";
  std::cout << "HalfHPBWdeg=" << halfHpbwDeg << "\n";
  std::cout << "HPBWdeg=" << hpbwDeg << "\n";
  std::cout << "Left3dBOffsetDeg=" << left3dBOffsetDeg << "\n";
  std::cout << "Right3dBOffsetDeg=" << right3dBOffsetDeg << "\n";
  std::cout << "ApproxHPBWdeg(102/N)=" << approxHpbwDeg << "\n";
  std::cout << "-------------------------\n";

  // --- Gain scan around boresight (offset view) ---
  std::cout << "OffsetDeg,GainDb\n";

  const int nSteps = static_cast<int>(std::floor(maxOffsetDeg / stepDeg + 1e-9));
  for (int i = -nSteps; i <= nSteps; ++i)
  {
    const double offDeg = i * stepDeg;
    const double az = Deg2Rad(offDeg);           // 음수도 그대로 넣음
    Angles a(az, M_PI / 2.0);
    const double g = ant->GetGainDb(a);
    std::cout << std::fixed << std::setprecision(4) << offDeg << "," << g << "\n";
  }


  return 0;
}
