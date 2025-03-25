#ifndef STEP_H
#define STEP_H

typedef struct Step
{
    float distance;
    int type;
    float targetLat;
    float targetLon;
    float startLat;
    float startLon;
} Step;

#endif // STEP_H