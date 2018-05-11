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

#include <ndn-cxx/lp/plocation.hpp>
#include <ndn-cxx/lp/dlocation.hpp>
#include <ndn-cxx/lp/empty-value.hpp>
#include <ndn-cxx/lp/tags.hpp>

namespace nfd {
namespace fw {

VanetStrategy::VanetStrategy(Forwarder& forwarder)
  : Strategy(forwarder)
{
}

void
VanetStrategy::afterReceiveInterest(const Face& inFace, const Interest& interest,
                                    const shared_ptr<pit::Entry>& pitEntry)
{
  const fib::Entry& fibEntry = this->lookupFib(*pitEntry);
  const fib::NextHopList& nexthops = fibEntry.getNextHops();

  shared_ptr<lp::DLocationTag> dLocationTag = interest.getTag<lp::DLocationTag>();
  shared_ptr<lp::PLocationTag> pLocationTag = interest.getTag<lp::PLocationTag>();

  // if there is a dlocationtag
    // if distance from my location is closer than the distance from my previous location
        // if centrality score for this prefix is found
          // schedule interest sending based on distance from destination and centrality score,
        // else schedule interest sending based on distance
    // else
  // else, if there is no dlocationtad
    // look up location-to-prefix table
    // if a matching location is found for the prefix
      // schedule interest sending based on distance from destination and centrality score, if found
    // else
      // if centrality score for this prefix is found
        // schedule interest sending based on centrality score
      // else, there is no centrality score or location information
        // Todo: what to do in this scenario
}

void
VanetStrategy::afterReceiveData(const shared_ptr<pit::Entry>& pitEntry,
                                const Face& inFace, const Data& data)
{
  // get the dlocationtag from the data packet
  shared_ptr<lp::PLocationTag> pLocationTag = data.getTag<lp::PLocationTag>();

  // update the prefix-to-location table

  // update prefix-to-centralityScore table: increase the score of the prefix
}

void
VanetStrategy::afterReceiveNack(const Face& inFace, const lp::Nack& nack,
                                const shared_ptr<pit::Entry>& pitEntry)
{

}


} // fw

} // nfd