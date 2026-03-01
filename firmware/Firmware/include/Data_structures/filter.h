//
// Created by matthias on 5/4/25.
//

#ifndef FILTER_H
#define FILTER_H

inline float lowPassFilter(float newValue, float previousValue, float alpha)
{
    return alpha * newValue + (1.0f - alpha) * previousValue;
}

#endif //FILTER_H