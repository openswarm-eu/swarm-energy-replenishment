/**
 * @file <utility/robot_position.h>
 *
 * @author Genki Miyauchi <g.miyauchi@sheffield.ac.uk>
 * 
 * Defines the color to be used by each team.
 */

#ifndef ROBOT_POSITION_H
#define ROBOT_POSITION_H

/* Definition of the CVector2 datatype */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/quaternion.h>

#include <unordered_map>

using namespace argos;

struct RobotPosition {
    CVector2 position;
    CQuaternion orientation;
};

#endif