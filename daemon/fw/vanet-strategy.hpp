/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2014-2016,  Regents of the University of California,
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

#ifndef NFD_DAEMON_FW_VANET_ROUTE_STRATEGY_HPP
#define NFD_DAEMON_FW_VANET_ROUTE_STRATEGY_HPP

#include "strategy.hpp"
#include "vanet-measurements.hpp"

#include <ndn-cxx/lp/plocation.hpp>
#include <ndn-cxx/lp/dlocation.hpp>
#include <ndn-cxx/util/scheduler.hpp>
#include <ndn-cxx/mgmt/nfd/controller.hpp>
#include <ndn-cxx/face.hpp>

#include <cmath>
#include <math.h>

#include <boost/asio.hpp>

namespace nfd {
namespace fw {
namespace vanet {

class VanetStrategy : public Strategy
{
public:
  VanetStrategy(Forwarder& forwarder, const Name& name);

  void
  afterReceiveInterest(const Face& inFace, const Interest& interest,
                       const shared_ptr<pit::Entry>& pitEntry) override;

  void
  afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                   const Face& inFace, const Data& data) override;

  void
  afterReceiveNack(const Face& inFace, const lp::Nack& nack,
                   const shared_ptr<pit::Entry>& pitEntry) override;

  static const Name&
  getStrategyName();

private:
  // This function converts decimal degrees to radians
  double deg2rad(double deg) {
    return (deg * M_PI / 180);
  }

  //  This function converts radians to decimal degrees
  double rad2deg(double rad) {
    return (rad * 180 / M_PI);
  }

  double
  calculateDistance(double lat1d, double lon1d,
                    double lat2d, double lon2d);

  std::pair<double, double>
  getMyLocation() {
    return {0, 0}; // latitude, longitude
  }


  double
  calculateTimer(double lat1d, double lon1d,
                 double lat2d, double lon2d);

  void
  forwardInterest(const Interest& interest,
                  const shared_ptr<pit::Entry>& pitEntry,
                  FaceInfo* info,
                  Face& outFace) {
    // send the interest
    this->sendInterest(pitEntry, outFace, interest);

    // remove the interest from the pool, because it has been sent.
    m_scheduledInterstPool.erase(interest.getName());

    // update the score of this prefix
    info->incrementInterest();
  }

private:
  const double earthRadiusKm = 6371;

  VanetMeasurements m_measurements;

  boost::asio::io_service m_ioService;

  ndn::Scheduler m_scheduler;
  ndn::Face m_face;
  ndn::KeyChain m_keyChain;
  ndn::nfd::Controller m_controller;
  std::unordered_map<ndn::Name, ndn::EventId> m_scheduledInterstPool;
};

} // namespace vanet
} // namespace fw
} // namespace nfd

#endif // NFD_DAEMON_FW_VANET_ROUTE_STRATEGY_HPP