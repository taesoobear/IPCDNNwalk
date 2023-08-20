#ifndef _VERSION_H_
#define _VERSION_H_
#pragma once

static const int MOT_RECENT_VERSION=6;

static const char* MOT_VERSION_STRING[]={"v0", "v1", "v2", "mot_version3", "mot_version4", "mot_version5"};

//MOT_RECENT_VERSION 2


//MOT_RECENT_VERSION 3 -> 2007.7.24
// posture에 translation joint기능을 추가하면서 버젼업.

//MOT_RECENT_VERSION 4 -> 2008.9.22
// motionloader에 m_aVoca2RotJointIndex를 m_aVoca2TreeIndex로 바꾸면서 버젼업.


class Posture;
void savePose(Posture& pose, const char* fn);
void loadPose(Posture& pose, const char* fn);
#endif
