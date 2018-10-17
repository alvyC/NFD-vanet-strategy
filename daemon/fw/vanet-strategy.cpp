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

#include "vanet-strategy.hpp"

#include <ndn-cxx/lp/empty-value.hpp>
#include <ndn-cxx/lp/tags.hpp>

#include "core/logger.hpp"
#include "algorithm.hpp"
#include "../../TraciApi/TraCIAPI.h"

namespace nfd {
namespace fw {
namespace vanet {

NFD_REGISTER_STRATEGY(VanetStrategy);

NFD_LOG_INIT(VanetStrategy);

VanetStrategy::VanetStrategy(Forwarder& forwarder, const Name& name)
  : Strategy(forwarder)
  , m_measurements(getMeasurements())
  , m_scheduler(m_ioService)
  , m_controller(m_face, m_keyChain)
{
  ParsedInstanceName parsed = parseInstanceName(name);
  if (!parsed.parameters.empty()) {
    BOOST_THROW_EXCEPTION(std::invalid_argument("AccessStrategy does not accept parameters"));
  }
  if (parsed.version && *parsed.version != getStrategyName()[-1].toVersion()) {
    BOOST_THROW_EXCEPTION(std::invalid_argument(
      "AccessStrategy does not support version " + to_string(*parsed.version)));
  }
  this->setInstanceName(makeInstanceName(name, getStrategyName()));
  TraCIAPI client;
}

void
VanetStrategy::afterReceiveInterest(const Face& inFace, const Interest& interest,
                                    const shared_ptr<pit::Entry>& pitEntry)
{
  NamespaceInfo* namespaceInfo = m_measurements.getNamespaceInfo(pitEntry->getName());

  if (namespaceInfo == nullptr) {
    NFD_LOG_TRACE("Could not find measurements entry for " << pitEntry->getName());
    return;
  }

  // if we received this interest before, then no need to send it again
  if (m_scheduledInterstPool.find(interest.getName()) != m_scheduledInterstPool.end()) {
    return;
  }

  // get list of next hops for this fib entry
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();

  // select a face which will be used for forwarding
  face::FaceId choosenFaceId;
  for (const fib::NextHop& hop : fibEntry.getNextHops()) {
    Face& hopFace = hop.getFace();

    if (hopFace.getLinkType() == ndn::nfd::LINK_TYPE_AD_HOC) {
      choosenFaceId = hopFace.getId();
      break;
    }
  }

  // get the score for the corresponding prefix
  FaceInfo* info = m_measurements.getFaceInfo(fibEntry, interest, choosenFaceId);
  double prefixScore = info->getScore();

  // extract tag from interest
  shared_ptr<lp::DLocationTag> dLocationTag = interest.getTag<lp::DLocationTag>();
  shared_ptr<lp::PLocationTag> pLocationTag = interest.getTag<lp::PLocationTag>();

  // get previous location
  lp::PLocation pl = pLocationTag->get();

  // get own's location
  std::pair<double, double> myLocation = getMyLocation();

  uint32_t duration;
  float alpha = 0.5; float beta = 0.5;

  // if there is a dlocationtag
  if (dLocationTag != nullptr) {
    lp::DLocation dl = dLocationTag->get();

    double distanceFromPrev = calculateDistance(pl.getLatitude(), pl.getLongitude(),
                                                dl.getLatitude(), dl.getLongitude());

    double distanceFromMe = calculateDistance(myLocation.first, myLocation.second,
                                              dl.getLatitude(), dl.getLongitude());

    std::pair<double, double> dest = info->getDestination();
    if (dest.first < 0 or dest.second < 0) { // this vehicle does not have the dest info for this prefix
      info->setDestination(dl.getLatitude(), dl.getLongitude());
    }

    duration = std::ceil(alpha * (1 - distanceFromMe/std::max(distanceFromMe, distanceFromPrev))
                         + beta * prefixScore);
  }
  // else, if there is no dlocation
  else {
    // look up location-to-prefix table
    std::pair<double, double> dest = info->getDestination();
    // if a matching location is found for the prefix
    if (dest.first > 0 or dest.second > 0) {
      // schedule interest sending based on distance from destination and centrality score, if found
      double distanceFromMe = calculateDistance(myLocation.first, myLocation.second,
                                                dest.first, dest.second);
      double distanceFromPrev = calculateDistance(pl.getLatitude(), pl.getLongitude(),
                                                  dest.first, dest.second);

      // the dest info was not present in the interest packet, so tag it with the info, so the later nodes can use it
      lp::DLocation dl(dest.first, dest.second);
      interest.setTag(make_shared<lp::DLocationTag>(dl));

      duration = std::ceil(alpha * (1 - distanceFromMe/std::max(distanceFromMe, distanceFromPrev))
                           + beta * prefixScore);

    }
    // else, only use prefix score
    else {
      duration = std::ceil(beta * prefixScore);
    }
  }

  time::milliseconds waitingTime(duration);

  if (nexthops.size() == 0) {
      for (auto outFace = this->getFaceTable().begin(); outFace != this->getFaceTable().end(); ++outFace) {
        if (wouldViolateScope(inFace, interest, *outFace)) {
          continue;
        }

        ndn::EventId eventId = m_scheduler.scheduleEvent(waitingTime,
                                [=]{VanetStrategy::forwardInterest(interest, pitEntry,
                                                                   info, *outFace);});

        // insert the event id in the scheduled interest pool
        m_scheduledInterstPool[interest.getName()] = eventId;
      }
    }
}

void
VanetStrategy::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                                const Face& inFace, const Data& data)
{
  // get the dlocationtag from the data packet
  shared_ptr<lp::DLocationTag> dLocationTag = data.getTag<lp::DLocationTag>();
  lp::DLocation dl = dLocationTag->get();

  // get the pit corresponding fibEntry and name tree entry
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);


  FaceInfo* info = m_measurements.getFaceInfo(fibEntry, pitEntry->getInterest(), inFace.getId());

  // set the destination location of name tree entry
  info->setDestination(dl.getLatitude(), dl.getLongitude());


  // register the prefix
  ndn::nfd::ControlParameters faceParameters;
  faceParameters
   .setName(data.getName())
   .setFaceId(inFace.getId())
   // .setFlags(flags)
   // .setCost(faceCost)
   // .setExpirationPeriod(timeout)
   // .setOrigin(ndn::nfd::ROUTE_ORIGIN_NLSR)
   ;
  m_controller.start<ndn::nfd::RibRegisterCommand>(faceParameters,
                                               [this] (const ndn::nfd::ControlParameters&) {
                                                NFD_LOG_DEBUG("Registration Successful");
                                               },
                                               [this] (const ndn::nfd::ControlResponse&) {
                                                NFD_LOG_DEBUG("Registration Unsuccessful");
                                               });

  // send data
  this->sendDataToAll(pitEntry, inFace, data);

  // update the score of the prefix
  info->incrementData();
  info->updateScore();
}

const Name&
VanetStrategy::getStrategyName() {
  static Name strategyName("/localhost/nfd/strategy/vanet/%FD%03");
  return strategyName;
}

void
VanetStrategy::afterReceiveNack(const Face& inFace, const lp::Nack& nack,
                                const shared_ptr<pit::Entry>& pitEntry)
{

}

double
VanetStrategy::calculateDistance(double lat1d, double lon1d,
                                 double lat2d, double lon2d)
{
  double lat1r, lon1r, lat2r, lon2r, u, v;

  lat1r = deg2rad(lat1d);
  lon1r = deg2rad(lon1d);
  lat2r = deg2rad(lat2d);
  lon2r = deg2rad(lon2d);

  u = sin((lat2r - lat1r)/2);
  v = sin((lon2r - lon1r)/2);

  return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
}

double
VanetStrategy::calculateTimer(double lat1d, double lon1d,
                              double lat2d, double lon2d)
{
  return 0.0;
}

} // vanet
} // fw
} // nfd
