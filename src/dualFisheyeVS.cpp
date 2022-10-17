#include "../include/dualFisheyeVS/dualFisheyeVS.h"

#include <visp/vpImageIo.h>
#include <visp3/core/vpImageFilter.h>

void buildLuminanceFeature(vpImage<unsigned char> I, cameraParameters C, std::vector<luminanceFeature> &s_, bool computeDerivatives){
    luminanceFeature s;
    unsigned char *ptrI = I.bitmap;
    s_.clear();

    for(int v=0;v<C.height;v++){
        for(int u=0;u<C.width;u++,ptrI++){
            if(u>2 && u<C.width-3 && v>2 && v<C.height-3 ){
                s.x = (1.*u-C.u0)/C.au;                                             //TODO : precalculate
                s.y = (1.*v-C.v0)/C.av;
                s.I = (*ptrI);
                s.Z = 5.0;
                if(computeDerivatives) {
                    s.dIdu = C.au * vpImageFilter::derivativeFilterX(I, v, u);      // TODO: derivatives adapted to fisheye image
                    s.dIdv = C.av * vpImageFilter::derivativeFilterY(I, v, u);
                }
                s_.push_back(s);
            }
        }
    }
}

void computeLuminanceInteractionMatrixOmni(std::vector<luminanceFeature> s, vpMatrix &L, cameraParameters C, vpHomogeneousMatrix oMi){
    L.resize(s.size(),6);
    vpRowVector l(6);
    vpMatrix V(6,6,0.0);
    V.insert(oMi.getRotationMatrix(),0,0);
    V.insert(oMi.getRotationMatrix(),3,3);
    V.insert(oMi.getTranslationVector().skew()*oMi.getRotationMatrix(),0,3);

    for(size_t i = 0; i < s.size(); i++){
        float xi = C.xi;
        float dIdu = -s[i].dIdu;
        float dIdv = -s[i].dIdv;
        float x = (float)(s[i].x);
        float y = (float)(s[i].y);
        float srac = 1.0+(1.0-xi*xi)*(x*x + y*y);
        float fact = (xi + sqrt(srac)) / (x*x + y*y + 1.0);
        float Zs = fact - xi;
        float Z = s[i].Z;
        float rho = Z/Zs;

        float Ls[6][2];
        Ls[0][0] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*y*y/rho) ;
        Ls[1][0] = xi*x*y/rho;
        Ls[2][0] = x * ((rho+xi*Z) / (rho*(Z + xi*rho)));
        Ls[3][0] = x*y;
        Ls[4][0] = -( x*x + (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) );
        Ls[5][0] = y;
        Ls[0][1] = xi*x*y/rho;
        Ls[1][1] = -( (rho*Z + xi*Z*Z)/(rho*pow(Z + xi*rho,2)) + xi*x*x/rho );
        Ls[2][1] = y * ( (rho+xi*Z)/(rho*(Z + xi*rho)) );
        Ls[3][1] = ( (Z*Z + Z*xi*rho)/(pow(Z + xi*rho,2)) ) + y*y;
        Ls[4][1] = -x*y;
        Ls[5][1] = -x;


        l[0] = (dIdu * Ls[0][0]) + (dIdv * Ls[0][1]);
        l[1] = (dIdu * Ls[1][0]) + (dIdv * Ls[1][1]);
        l[2] = (dIdu * Ls[2][0]) + (dIdv * Ls[2][1]);
        l[3] = (dIdu * Ls[3][0]) + (dIdv * Ls[3][1]);
        l[4] = (dIdu * Ls[4][0]) + (dIdv * Ls[4][1]);
        l[5] = (dIdu * Ls[5][0]) + (dIdv * Ls[5][1]);
        L.insert(l*V,i,0);
    }
}


void computeLuminanceInteractionMatrix(std::vector<luminanceFeature> s, vpMatrix &L, vpHomogeneousMatrix oMi){
    L.resize(s.size(),6);

    vpRowVector l(6);
    vpMatrix V(6,6,0.0);
    V.insert(oMi.getRotationMatrix(),0,0);
    V.insert(oMi.getRotationMatrix(),3,3);
    V.insert(oMi.getTranslationVector().skew()*oMi.getRotationMatrix(),0,3);

    for(size_t i = 0; i < s.size(); i++){
        l[0] = s[i].dIdu  * (1.0/s[i].Z) ;
        l[1] = s[i].dIdv  * (1.0/s[i].Z) ;
        l[2] = -((float)(s[i].x) * s[i].dIdu + (float)(s[i].y * s[i].dIdv)) * (1.0/s[i].Z) ;
        l[3] = -s[i].dIdu * (float)(s[i].x) * (float)(s[i].y) - (1.0 + (float)(s[i].y) * (float)(s[i].y)) * s[i].dIdv;
        l[4] = (1.0 + (float)(s[i].x) * (float)(s[i].x)) * s[i].dIdu + s[i].dIdv * (float)(s[i].x) * (float)(s[i].y);
        l[5] = s[i].dIdv * (float)(s[i].x) - s[i].dIdu * (float)(s[i].y);
        L.insert(l*V,i,0);
    }
}

void computeErrorVector(std::vector<luminanceFeature> s, std::vector<luminanceFeature> sd, vpColVector &e){
    e.resize(s.size());
    for(size_t i = 0; i < s.size(); i++)
        e[i] = s[i].I - sd[i].I;
}


