#include "Trajectory.h"
#include <Arduino.h>
#include <math.h>

Point3D calculateTrajectory(
    const Point3D& home,
    float phase,
    float stride,
    float stepHeight,
    float motionX,
    float motionY,
    float rotation,
    LegID leg
) {
    phase = constrain(phase, 0.0f, 1.0f);

    Point3D target = home;

    // ==================================================
    // 1. LÉPÉS - ELŐRE / HÁTRA / OLDALRA
    // ==================================================

    float moveX = 0.0f;
    float moveY = 0.0f;

    if (phase < 0.5f) {

        // Swing / emelés
        float t = phase / 0.5f;

        moveX =
            motionX *
            stride *
            (t - 0.5f);

        moveY =
            motionY *
            stride *
            (t - 0.5f);

        target.z =
            home.z +
            stepHeight *
            sin(t * MY_PI);

    } else {

        // Stance / talajon lévő szakasz
        float t = (phase - 0.5f) / 0.5f;

        moveX =
            motionX *
            stride *
            (0.5f - t);

        moveY =
            motionY *
            stride *
            (0.5f - t);

        target.z = home.z;
    }

    target.x += moveX;
    target.y += moveY;


    // ==================================================
    // 2. HELYBEN FORDULÁS
    // ==================================================

    if (rotation != 0.0f) {

        /*
         * A rotation nem egy fix pozícióeltolás.
         *
         * A lábnak a test középpontja körül kell
         * "előre-hátra" mozognia egy íven.
         */

        float turnPhase;

        if (phase < 0.5f) {

            // Swing:
            // egyik irányból a másikba
            turnPhase =
                (phase / 0.5f) - 0.5f;

        } else {

            // Stance:
            // visszatérés
            turnPhase =
                0.5f -
                ((phase - 0.5f) / 0.5f);
        }

        /*
         * A rotation fokban értendő.
         *
         * Például:
         * rotation = 5°
         *
         * Ez egy ±5°-os lábmozgást jelent
         * a test középpontja körül.
         */

        float angle =
            rotation *
            turnPhase *
            MY_PI /
            180.0f;

        float cosA = cos(angle);
        float sinA = sin(angle);

        // Az aktuális célpont forgatása
        // a robot testének középpontja körül.

        float rotatedX =
            target.x * cosA -
            target.y * sinA;

        float rotatedY =
            target.x * sinA +
            target.y * cosA;

        target.x = rotatedX;
        target.y = rotatedY;
    }

    return target;
}