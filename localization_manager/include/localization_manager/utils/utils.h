#ifndef _LOCALIZATION_MANAGER_H_UTILS_
#define _LOCALIZATION_MANAGER_H_UTILS_
float getProbGivenNormalDistribution(float x, float mean, float sigma){
    float z = (x-mean)/(1.414213*sigma);
    float res = exp(-(z*z));
    float scalingFactor = 1/sqrt(2*M_PI*sigma*sigma);
    res *= scalingFactor;
    return res;
}
#endif