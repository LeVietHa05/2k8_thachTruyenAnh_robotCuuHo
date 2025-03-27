#ifndef STEP_H
#define STEP_H

typedef struct Step
{
    byte id;
    float distance;
    int type;
    double targetLat;
    double targetLon;
    double startLat;
    double startLon;
} Step;

#endif // STEP_H