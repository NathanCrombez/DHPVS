#ifndef LOCALIZATION_STEREOVISUALSERVOING_H
#define LOCALIZATION_STEREOVISUALSERVOING_H

////Visp
#include <visp/vpImage.h>



typedef struct{
    int height, width;
    float au, av;
    float u0, v0;
    float xi;
}cameraParameters;

typedef struct{
    float x, y;
    float dIdu, dIdv;
    float I;
    float Z;
}luminanceFeature;

void buildLuminanceFeature(vpImage<unsigned char> I, cameraParameters C, std::vector<luminanceFeature> &s, bool computeDerivatives=true);
void computeLuminanceInteractionMatrix(std::vector<luminanceFeature> s, vpMatrix &L, vpHomogeneousMatrix oMi=vpHomogeneousMatrix(0,0,0,0,0,0));
void computeLuminanceInteractionMatrixOmni(std::vector<luminanceFeature> s, vpMatrix &L, cameraParameters C, vpHomogeneousMatrix oMi=vpHomogeneousMatrix(0,0,0,0,0,0));

void computeErrorVector(std::vector<luminanceFeature> s, std::vector<luminanceFeature> sd, vpColVector &e);



#endif //LOCALIZATION_STEREOVISUALSERVOING_H
