/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014-2018,  Regents of the University of California,
 *                           Arizona Board of Regents,
 *                           Colorado State University,
 *                           University Pierre & Marie Curie, Sorbonne University,
 *                           Washington University in St. Louis,
 *                           Beijing Institute of Technology,
 *                           The University of Memphis.
 *
 * This file is part of NFD (Named Data Networking Forwarding Daemon).
 * See AUTHORS.md for complete list of NFD authors and contributors.
 *
 * NFD is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * NFD is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * NFD, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NFD_DAEMON_FW_VANET_MEASUREMENTS_HPP
#define NFD_DAEMON_FW_VANET_MEASUREMENTS_HPP

#include "core/rtt-estimator.hpp"
#include "fw/strategy-info.hpp"
#include "table/measurements-accessor.hpp"

namespace nfd {
namespace fw {
namespace vanet {

class RttStats
{
public:
  typedef time::duration<double, boost::micro> Rtt;

  RttStats();

  void
  addRttMeasurement(RttEstimator::Duration& durationRtt);

  void
  recordTimeout()
  {
    m_rtt = RTT_TIMEOUT;
  }

  Rtt
  getRtt() const
  {
    return m_rtt;
  }

  Rtt
  getSrtt() const
  {
    return m_srtt;
  }

  RttEstimator::Duration
  computeRto() const
  {
    return m_rttEstimator.computeRto();
  }

private:
  static Rtt
  computeSrtt(Rtt previousSrtt, Rtt currentRtt);

public:
  static const Rtt RTT_TIMEOUT;
  static const Rtt RTT_NO_MEASUREMENT;

private:
  Rtt m_srtt;
  Rtt m_rtt;
  RttEstimator m_rttEstimator;

  static const double ALPHA;
};

class FaceInfo {
public:
  class Error : public std::runtime_error
  {
  public:
    explicit
    Error(const std::string& what)
      : std::runtime_error(what)
    {
    }
  };

  FaceInfo();

  // ~FaceInfo();

  double
  getScore() const
  {
    return m_score;
  }

  void
  updateScore() {
    m_score = m_noOfData/ m_noOfInterest;
  }

  void
  setDestination(double lat, double longi) {
    m_destination = std::make_pair(lat, longi);
  }

  std::pair<double, double>
  getDestination() const
  {
    return m_destination;
  }

  void
  incrementInterest() {
    m_noOfInterest++;
    updateScore();
  }

  void
  incrementData() {
    m_noOfData++;
    updateScore();
  }

  void
  reduceScore() {
    m_score = m_score/3; // reduce by third, it is a configurable number
  }

private:
  Name m_name;
  double m_score;
  std::pair<double, double> m_destination; // lat, long. todo: make it a list of destination

  int m_noOfInterest;
  int m_noOfData;

  scheduler::EventId m_timeoutEventId;
  bool m_isTimeoutScheduled;
};

typedef std::unordered_map<FaceId, FaceInfo> FaceInfoTable;

class NamespaceInfo : public StrategyInfo
{
public:
  NamespaceInfo();

  static constexpr int
  getTypeId()
  {
    return 1080;
  }

  FaceInfo&
  getOrCreateFaceInfo(const fib::Entry& fibEntry, FaceId faceId);

  FaceInfo*
  getFaceInfo(const fib::Entry& fibEntry, FaceId faceId);

  void
  expireFaceInfo(FaceId faceId);

  void
  extendFaceInfoLifetime(FaceInfo& info, FaceId faceId);

  FaceInfo*
  get(FaceId faceId)
  {
    if (m_fit.find(faceId) != m_fit.end()) {
      return &m_fit.at(faceId);
    }
    else {
      return nullptr;
    }
  }

  FaceInfoTable::iterator
  find(FaceId faceId)
  {
    return m_fit.find(faceId);
  }

  FaceInfoTable::iterator
  end()
  {
    return m_fit.end();
  }

  const FaceInfoTable::iterator
  insert(FaceId faceId)
  {
    const auto& pair = m_fit.insert(std::make_pair(faceId, FaceInfo()));
    return pair.first;
  }

private:
  FaceInfoTable m_fit;
};

class VanetMeasurements : noncopyable
{
public:
  explicit
  VanetMeasurements(MeasurementsAccessor& measurements);

  FaceInfo*
  getFaceInfo(const fib::Entry& fibEntry, const Interest& interest, FaceId faceId);

  FaceInfo&
  getOrCreateFaceInfo(const fib::Entry& fibEntry, const Interest& interest, FaceId faceId);

  NamespaceInfo*
  getNamespaceInfo(const Name& prefix);

  NamespaceInfo&
  getOrCreateNamespaceInfo(const fib::Entry& fibEntry, const Interest& interest);

private:
  MeasurementsAccessor& m_measurements;
};

} // namespace vanet
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_VANET_MEASUREMENTS_HPP
