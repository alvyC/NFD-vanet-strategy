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

#include "vanet-measurements.hpp"

namespace nfd {
namespace fw {
namespace vanet {

FaceInfo::FaceInfo()
  : m_noOfInterest(0)
  , m_noOfData(0)
  , m_score(0)
  , m_destination{-1.0, -1.0}
  {
  }

NamespaceInfo::NamespaceInfo() {

}

FaceInfo&
NamespaceInfo::getOrCreateFaceInfo(const fib::Entry& fibEntry, FaceId faceId) {
  FaceInfoTable::iterator it = m_fit.find(faceId);

  FaceInfo* info = nullptr;

  if (it == m_fit.end()) {
    const auto& pair = m_fit.insert(std::make_pair(faceId, FaceInfo()));
    info = &pair.first->second;

    extendFaceInfoLifetime(*info, faceId);
  }
  else {
    info = &it->second;
  }

  return *info;
}

FaceInfo*
NamespaceInfo::getFaceInfo(const fib::Entry& fibEntry, FaceId faceId)
{
  return nullptr;
}

void
NamespaceInfo::expireFaceInfo(FaceId faceId)
{

}

void
NamespaceInfo::extendFaceInfoLifetime(FaceInfo& info, FaceId faceId)
{

}

VanetMeasurements::VanetMeasurements(MeasurementsAccessor& measurements)
  : m_measurements(measurements)
{
}

FaceInfo*
VanetMeasurements::getFaceInfo(const fib::Entry& fibEntry, const Interest& interest,
                               FaceId faceId) {
  return nullptr;
}

FaceInfo&
VanetMeasurements::getOrCreateFaceInfo(const fib::Entry& fibEntry, const Interest& interest,
                                       FaceId faceId) {
  NamespaceInfo& info = getOrCreateNamespaceInfo(fibEntry, interest);
  return info.getOrCreateFaceInfo(fibEntry, faceId);
}

NamespaceInfo*
VanetMeasurements::getNamespaceInfo(const Name& prefix) {
  return nullptr;
}

NamespaceInfo&
VanetMeasurements::getOrCreateNamespaceInfo(const fib::Entry& fibEntry,
                                            const Interest& interest) {
  measurements::Entry* me = m_measurements.get(fibEntry);

  // If the FIB entry is not under the strategy's namespace, find a part of the prefix
  // that falls under the strategy's namespace
  for (size_t prefixLen = fibEntry.getPrefix().size() + 1;
       me == nullptr && prefixLen <= interest.getName().size(); ++prefixLen) {
    me = m_measurements.get(interest.getName().getPrefix(prefixLen));
  }

  // Either the FIB entry or the Interest's name must be under this strategy's namespace
  BOOST_ASSERT(me != nullptr);

  // Set or update entry lifetime
  //extendLifetime(*me);

  NamespaceInfo* info = me->insertStrategyInfo<NamespaceInfo>().first;
  BOOST_ASSERT(info != nullptr);
  return *info;
}

} // vanet
} // fw
} // nfd