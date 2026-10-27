#ifndef DYNAMIC_CAMERA_H
#define DYNAMIC_CAMERA_H

#include <iostream>
#include <limits>

#include "camera.h"

constexpr float INF = std::numeric_limits<float>::infinity();

// AABB in 1D
struct Interval {
    float min;
    float max;

    constexpr bool contains(const Interval& other) const {
        return (min <= other.min) && (max >= other.max);
    }

    constexpr bool contains(float x) const { return (min <= x) && (x <= max); }

    // signedDistance returns a positive smallest distance of x from the interval boundary
    // if it is not contained, else it returns a smallest (in magnitude) negative distance
    // from the interval boundary.
    constexpr float signedDistance(float x) const {
        if (contains(x)) {
            // 0 or negative
            return std::max(x - max, min - x);
        }
        // positive
        return std::max(x - max, min - x);
    }
};

struct DynamicCamera {
    void update(Camera& camera,                           // the camera to update
                const sf::Vector2f& trackedPosWorld,      // world coords of player to track
                const sf::Vector2f& trackedVelocityWorld  // world velocity of player to track
    ) {
        // TODO: (HINTS/outline)
        // collect statistics on the player position and velocity in (normalized, if you like)
        // window coordinates. Compute lookAhead distance in world coords, adjusting for
        // if the player is moving left or right.
        camera.setCameraCenter(cameraUnadjustedCenter);

        sf::Vector2f playerWindow = camera.worldToWindowPercent(trackedPosWorld);

        float lookAheadDir = 0.f;

        if (trackedVelocityWorld.x > 0.f) {
            lookAheadDir = 1.f;
        } else if (trackedVelocityWorld.x < 0.f) {
            lookAheadDir= -1.f;
        }

        float targetLookahead = camera.windowPercentToWorldDist({lookAhead * lookAheadDir, 0.f}).x;

        // Now update lookAhead state using an EMA over its previous state and the lookAhead world
        // vector. Then determine if the player is in the dead zone, soft rejection zone, or hard
        // rejection zone. For each, perform EMA on the delta of the camera displacement (ignoring
        // lookahead) appropriately. Next,
        float cameraLatPercentage = 0.f; // amount camera should move laterally as %

        if (playerWindow.x < deadZoneX.min) {
            if (playerWindow.x < softZoneX.min) {
                cameraLatPercentage = playerWindow.x - softZoneX.min;
            } else {
                cameraLatPercentage = alpha * (playerWindow.x - deadZoneX.min);
            }
        } else if (playerWindow.x > deadZoneX.max) {
            if (playerWindow.x > softZoneX.max) {
                cameraLatPercentage = playerWindow.x - softZoneX.max;
            } else {
                cameraLatPercentage = alpha * (playerWindow.x - deadZoneX.max);
            }
        }

        sf::Vector2f cameraDeltaWorld = camera.windowPercentToWorldDist({cameraLatPercentage, 0.f});

        cameraUnadjustedCenter += cameraDeltaWorld;

        lookaheadState += alpha * (targetLookahead - lookaheadState);

        // set the camera center by offsetting the cameraUnadjustedCenter, combining lookAhead.
        camera.setCameraCenter(cameraUnadjustedCenter + sf::Vector2f{lookaheadState, 0.f});
        // Finally update cameraUnadjustedCenter, but without lookahead.

    }

    float alpha;  // alpha to use for damping of camera position, *as well as* for smoothing of
                  // lookahead offset.
    Interval deadZoneX;  // percentage interval along x-axis to use for dead zone in [0, 1]
    // percentage interval along x-axis to use for soft rejection zone in [0, 1]
    Interval softZoneX = {-INF, INF};
    // (everything beyond softZone is considered hard zone).
    float lookAhead;  // screen-percentage lookahead
    mutable sf::Vector2f
        cameraUnadjustedCenter;      // the camera world center from last frame, ignoring lookahead
    mutable float lookaheadState{};  // the EMA of the accumulated lookahead offset in world coords
};

#endif // DYNAMIC_CAMERA_H
