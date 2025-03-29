#ifndef STEP_H
#define STEP_H


enum StepType
{
    FORWARD = 0,
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    BACKWARD = 3,
    STOP = 4,
};

typedef struct Step
{
    byte id;
    float distance;
    StepType type;
    double targetLat;
    double targetLon;
    double startLat;
    double startLon;
} Step;

#endif // STEP_H