/**
 * \file track_matrix.cpp
 *
 *  Created on: Mar 24, 2018
 *      \author: jsola
 */

#include "track_matrix.h"

namespace wolf
{

TrackMatrix::TrackMatrix()
{
    //
}

TrackMatrix::~TrackMatrix()
{
    //
}

TrackType TrackMatrix::track(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id);
    else
        return TrackType();
}

SnapshotType TrackMatrix::snapshot(CaptureBasePtr _capture)
{
    if (snapshots_.count(_capture->id()) > 0)
        return snapshots_.at(_capture->id());
    else
        return SnapshotType();
}

void TrackMatrix::add(CaptureBasePtr _cap, FeatureBasePtr _ftr)
{
    tracks_[_ftr->trackId()].emplace(_cap->getTimeStamp(), _ftr); // will create new track    if _track_id is not present
    snapshots_[_cap->id()].emplace(_ftr->trackId(), _ftr);        // will create new snapshot if _cap_id   is not present
}

void TrackMatrix::remove(size_t _track_id)
{
    // Remove from all Snapshots
    for (auto pair_time_ftr : tracks_.at(_track_id))
        for (auto pair_cap_snapshot : snapshots_)
            pair_cap_snapshot.second.erase(pair_time_ftr.second->trackId());
    // Remove track
    tracks_.erase(_track_id);
}

void TrackMatrix::remove(CaptureBasePtr _cap)
{
    // remove features in all tracks
    for (auto pair_time_ftr : snapshots_.at(_cap->id()))
        for (auto pair_trkid_track : tracks_)
            pair_trkid_track.second.erase(_cap->getTimeStamp());
    // remove snapshot
    snapshots_.erase(_cap->id());
}

void TrackMatrix::remove(FeatureBasePtr _ftr)
{
    if (_ftr)
        if (auto cap = _ftr->getCapturePtr())
        {
            tracks_   .at(_ftr->trackId()).erase(cap->getTimeStamp());
            snapshots_.at(cap->id())      .erase(_ftr->trackId());
        }
}

size_t TrackMatrix::numTracks()
{
    return tracks_.size();
}

size_t TrackMatrix::trackSize(size_t _track_id)
{
    return track(_track_id).size();
}

FeatureBasePtr TrackMatrix::firstFeature(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id).begin()->second;
    else
        return nullptr;
}

FeatureBasePtr TrackMatrix::lastFeature(size_t _track_id)
{
    if (tracks_.count(_track_id) > 0)
        return tracks_.at(_track_id).rbegin()->second;
    else
        return nullptr;
}

vector<FeatureBasePtr> TrackMatrix::trackAsVector(size_t _track_id)
{
    vector<FeatureBasePtr> vec(trackSize(_track_id));
    size_t i = 0;
    for (auto pair_time_ftr : tracks_.at(_track_id))
    {
        vec[i] = pair_time_ftr.second;
        i++;
    }
    return vec;
}

FeatureBasePtr TrackMatrix::feature(size_t _track_id, size_t _position)
{
    if (tracks_.count(_track_id) > 0 && _position <= trackSize(_track_id))
        return trackAsVector(_track_id).at(_position);
    else
        return nullptr;
}

CaptureBasePtr TrackMatrix::firstCapture(size_t _track_id)
{
    return firstFeature(_track_id)->getCapturePtr();
}

}
