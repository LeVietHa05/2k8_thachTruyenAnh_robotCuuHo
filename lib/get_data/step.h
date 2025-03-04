#ifndef STEP_H
#define STEP_H

typedef struct Step
{
    float distance;
    float duration;
    int type;
    String instruction;
    float targetLat;
    float targetLon;
} Step;

#endif // STEP_H