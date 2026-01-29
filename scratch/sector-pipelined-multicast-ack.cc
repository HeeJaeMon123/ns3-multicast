#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/antenna-module.h"
#include "ns3/paper-cone-antenna-model.h"

#include <vector>
#include <random>
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace ns3;

static inline double Deg2Rad (double deg) { return deg * M_PI / 180.0; }
static inline double Rad2Deg (double rad) { return rad * 180.0 / M_PI; }



// Free-space path loss (dB) using 32.4 + 20log10(fc_MHz) + 20log10(d_km)
static double FsPlDb (double fcHz, double dMeters)
{
  const double fcMHz = fcHz / 1e6;
  const double dKm = std::max (dMeters / 1000.0, 1e-9);
  return 32.4 + 20.0 * std::log10 (fcMHz) + 20.0 * std::log10 (dKm);
}

struct UeInfo
{
  Ptr<Node> node;
  Vector pos;
  double az;        // azimuth from BS
  uint32_t sector;  // assigned sector
};

struct Stats
{
  uint64_t dataTx = 0;
  uint64_t dataRx = 0;
  uint64_t ackTx = 0;
  uint64_t ackRx = 0;
  double   ackDelaySum = 0.0;
  uint64_t ackDelayCnt = 0;
};

class SectorPipelineSim
{
public:
  SectorPipelineSim () = default;

  void Configure (uint32_t numUe,
                  uint32_t numSectors,
                  double radius,
                  double dataMs,
                  double ackGapMs,
                  uint32_t steps,
                  uint32_t seed);

  void Run ();

private:
  void BuildTopology ();
  void AssignSectors ();
  void StepSendData (uint32_t stepIdx);
  void ReceiveAckForSector (uint32_t sectorIdx, Time dataEndTime);

  bool DownlinkOk (const UeInfo &ue, double boresightAz);
  bool UplinkOk (const UeInfo &ue, double boresightAz);

private:
  uint32_t m_numUe {60};
  uint32_t m_numSectors {6};
  double   m_radius {100.0};
  Time     m_dataT {MilliSeconds (1.0)};
  Time     m_ackGapT {MilliSeconds (0.2)};
  uint32_t m_steps {60};
  uint32_t m_seed {1};

  // Link budget parameters
  double m_fcHz {28e9};
  double m_bsTxPowerDbm {30.0};
  double m_ueTxPowerDbm {20.0};
  double m_rxThreshDbm {-90.0}; // simplistic threshold

  Ptr<Node> m_bs;
  Ptr<PaperConeAntennaModel> m_bsAnt;

  std::vector<UeInfo> m_ues;
  std::vector<Stats> m_stats; // per sector

  // Aggregator: pick one UE per sector (closest to sector center az)
  std::vector<int> m_aggUeIndex;
};

void
SectorPipelineSim::Configure (uint32_t numUe,
                              uint32_t numSectors,
                              double radius,
                              double dataMs,
                              double ackGapMs,
                              uint32_t steps,
                              uint32_t seed)
{
  m_numUe = numUe;
  m_numSectors = std::max (numSectors, 1u);
  m_radius = radius;
  m_dataT = MilliSeconds (dataMs);
  m_ackGapT = MilliSeconds (ackGapMs);
  m_steps = steps;
  m_seed = seed;
}

void
SectorPipelineSim::BuildTopology ()
{
  m_bs = CreateObject<Node> ();

  m_bsAnt = CreateObject<PaperConeAntennaModel> ();
  m_bsAnt->SetAttribute ("N", UintegerValue (64));
  m_bsAnt->SetAttribute ("Beta", DoubleValue (0.0));
  m_bsAnt->SetAttribute ("K3db", DoubleValue (2.782));
  m_bsAnt->SetAttribute ("UsePowerPattern", BooleanValue (true));
  m_bsAnt->SetAttribute ("UseManualCone", BooleanValue (false));
  m_bsAnt->SetAttribute ("BoresightAzimuthRad", DoubleValue (0.0));

  m_ues.clear ();
  m_ues.reserve (m_numUe);

  std::mt19937 rng (m_seed);
  std::uniform_real_distribution<double> uni01 (0.0, 1.0);
  std::uniform_real_distribution<double> uniAng (-M_PI, M_PI);

  for (uint32_t i = 0; i < m_numUe; ++i)
    {
      Ptr<Node> ue = CreateObject<Node> ();
      // Uniform in disk: r = R*sqrt(u)
      const double r = m_radius * std::sqrt (uni01 (rng));
      const double ang = uniAng (rng);
      const double x = r * std::cos (ang);
      const double y = r * std::sin (ang);

      UeInfo info;
      info.node = ue;
      info.pos = Vector (x, y, 0.0);
      info.az = ns3::WrapToPi (std::atan2 (y, x));
      info.sector = 0;
      m_ues.push_back (info);
    }

  m_stats.assign (m_numSectors, Stats{});
  m_aggUeIndex.assign (m_numSectors, -1);
}

void
SectorPipelineSim::AssignSectors ()
{
  const double sectorW = 2.0 * M_PI / static_cast<double> (m_numSectors);

  // Assign each UE to sector by its azimuth
  for (auto &ue : m_ues)
    {
      double a = ue.az;
      // Map [-pi,pi) -> [0,2pi)
      const double ap = (a < 0) ? (a + 2.0 * M_PI) : a;
      uint32_t s = static_cast<uint32_t> (std::floor (ap / sectorW));
      if (s >= m_numSectors) s = m_numSectors - 1;
      ue.sector = s;
    }

  // Pick aggregator per sector: closest to sector center az
  for (uint32_t s = 0; s < m_numSectors; ++s)
    {
      const double center = (static_cast<double> (s) + 0.5) * sectorW; // [0,2pi)
      double best = 1e9;
      int bestIdx = -1;

      for (uint32_t i = 0; i < m_ues.size (); ++i)
        {
          if (m_ues[i].sector != s) continue;
          const double ap = (m_ues[i].az < 0) ? (m_ues[i].az + 2.0 * M_PI) : m_ues[i].az;
          const double d = std::abs (ap - center);
          if (d < best)
            {
              best = d;
              bestIdx = static_cast<int> (i);
            }
        }

      // If sector empty, leave -1
      m_aggUeIndex[s] = bestIdx;
    }
}

bool
SectorPipelineSim::DownlinkOk (const UeInfo &ue, double boresightAz)
{
  // direction from BS to UE: azimuth = ue.az
  Angles a (ue.az, M_PI / 2.0);
  m_bsAnt->SetAttribute ("BoresightAzimuthRad", DoubleValue (boresightAz));
  const double gTx = m_bsAnt->GetGainDb (a);
  const double gRx = 0.0; // UE omni

  const double d = std::sqrt (ue.pos.x * ue.pos.x + ue.pos.y * ue.pos.y);
  const double pl = FsPlDb (m_fcHz, d);

  const double pr = m_bsTxPowerDbm + gTx + gRx - pl;
  return pr >= m_rxThreshDbm;
}

bool
SectorPipelineSim::UplinkOk (const UeInfo &ue, double boresightAz)
{
  // direction from BS to UE used for BS receive gain
  Angles a (ue.az, M_PI / 2.0);
  m_bsAnt->SetAttribute ("BoresightAzimuthRad", DoubleValue (boresightAz));
  const double gRxBs = m_bsAnt->GetGainDb (a);
  const double gTxUe = 0.0; // UE omni

  const double d = std::sqrt (ue.pos.x * ue.pos.x + ue.pos.y * ue.pos.y);
  const double pl = FsPlDb (m_fcHz, d);

  const double pr = m_ueTxPowerDbm + gTxUe + gRxBs - pl;
  return pr >= m_rxThreshDbm;
}

void
SectorPipelineSim::ReceiveAckForSector (uint32_t sectorIdx, Time dataEndTime)
{
  // Beam returns to this sector, aggregator transmits one aggregated ACK
  if (sectorIdx >= m_numSectors) return;

  Stats &st = m_stats[sectorIdx];
  st.ackTx += 1;

  const int aggIdx = m_aggUeIndex[sectorIdx];
  if (aggIdx < 0)
    {
      return; // no UE => no ACK
    }

  // Sector center boresight
  const double sectorW = 2.0 * M_PI / static_cast<double> (m_numSectors);
  const double center = (static_cast<double> (sectorIdx) + 0.5) * sectorW;
  const double boresightAz = ns3::WrapToPi (center);


  const bool ok = UplinkOk (m_ues[static_cast<uint32_t> (aggIdx)], boresightAz);
  if (ok)
    {
      st.ackRx += 1;
      const double delay = (Simulator::Now () - dataEndTime).GetSeconds ();
      st.ackDelaySum += delay;
      st.ackDelayCnt += 1;
    }
}

void
SectorPipelineSim::StepSendData (uint32_t stepIdx)
{
  const uint32_t s = stepIdx % m_numSectors;
  Stats &st = m_stats[s];

  // Sector center boresight
  const double sectorW = 2.0 * M_PI / static_cast<double> (m_numSectors);
  const double center = (static_cast<double> (s) + 0.5) * sectorW;
  const double boresightAz = ns3::WrapToPi (center);


  // "Send data" to all UEs in this sector
  const Time tStart = Simulator::Now ();
  const Time tEnd = tStart + m_dataT;

  uint64_t tx = 0, rx = 0;
  for (const auto &ue : m_ues)
    {
      if (ue.sector != s) continue;
      tx += 1;
      if (DownlinkOk (ue, boresightAz))
        {
          rx += 1;
        }
    }

  st.dataTx += tx;
  st.dataRx += rx;

  // Pipeline: while next sector is served, ACK is aggregated.
  // Return time: after next data duration + ackGap
  const Time tReturn = tEnd + m_dataT + m_ackGapT;

  Simulator::Schedule (tReturn - Simulator::Now (),
                       &SectorPipelineSim::ReceiveAckForSector,
                       this,
                       s,
                       tEnd);
}

void
SectorPipelineSim::Run ()
{
  BuildTopology ();
  AssignSectors ();

  const Time stepPeriod = m_dataT; // data launch each dataMs
  for (uint32_t k = 0; k < m_steps; ++k)
    {
      Simulator::Schedule (stepPeriod * k, &SectorPipelineSim::StepSendData, this, k);
    }

  // Stop after last return ACK
  const Time stopT = stepPeriod * m_steps + m_dataT + m_dataT + m_ackGapT + MilliSeconds (1.0);
  Simulator::Stop (stopT);
  Simulator::Run ();
  Simulator::Destroy ();

  // Print results
  std::cout << "Sector,DataTx,DataRx,FLR,AckTx,AckRx,AckSuccRate,MeanAckDelaySec\n";
  for (uint32_t s = 0; s < m_numSectors; ++s)
    {
      const auto &st = m_stats[s];
      const double flr = (st.dataTx == 0) ? 0.0 : 1.0 - (static_cast<double> (st.dataRx) / static_cast<double> (st.dataTx));
      const double ackSucc = (st.ackTx == 0) ? 0.0 : (static_cast<double> (st.ackRx) / static_cast<double> (st.ackTx));
      const double meanAckD = (st.ackDelayCnt == 0) ? 0.0 : (st.ackDelaySum / static_cast<double> (st.ackDelayCnt));

      std::cout << s << ","
                << st.dataTx << ","
                << st.dataRx << ","
                << std::fixed << std::setprecision (6) << flr << ","
                << st.ackTx << ","
                << st.ackRx << ","
                << std::fixed << std::setprecision (6) << ackSucc << ","
                << std::fixed << std::setprecision (6) << meanAckD
                << "\n";
    }
}

int
main (int argc, char *argv[])
{
  uint32_t numUe = 60;
  uint32_t numSectors = 6;
  double radius = 100.0;
  double dataMs = 1.0;
  double ackGapMs = 0.2;
  uint32_t steps = 60;
  uint32_t seed = 1;

  CommandLine cmd;
  cmd.AddValue ("numUe", "Number Of UEs", numUe);
  cmd.AddValue ("numSectors", "Number Of Sectors", numSectors);
  cmd.AddValue ("radius", "UE Placement Radius (m)", radius);
  cmd.AddValue ("dataMs", "Data Transmission Duration Per Sector (ms)", dataMs);
  cmd.AddValue ("ackGapMs", "Extra Gap Before Receiving Aggregated Ack (ms)", ackGapMs);
  cmd.AddValue ("steps", "Number Of Data Steps", steps);
  cmd.AddValue ("seed", "Random Seed", seed);
  cmd.Parse (argc, argv);

  SectorPipelineSim sim;
  sim.Configure (numUe, numSectors, radius, dataMs, ackGapMs, steps, seed);
  sim.Run ();

  return 0;
}
