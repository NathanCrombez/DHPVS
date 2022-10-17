#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageFilter.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPixelMeterConversion.h>

#include "../include/localization/ncFeatureLuminance.h"

/*!
  \file ncFeatureLuminance.cpp
  \brief Class that defines the image luminance visual feature

  For more details see \cite Collewet08c.
*/

/*!
  Initialize the memory space requested for ncFeatureLuminance visual feature.
*/
void ncFeatureLuminance::init()
{
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default value Z (1 meters)
  Z = 1;

  firstTimeIn = 0;

  nbr = nbc = 0;

  dof.resize(6);
  dof = {1,1,1,1,1,1};
}

void ncFeatureLuminance::init(unsigned int _nbr, unsigned int _nbc, double _Z)
{
  init();

  nbr = _nbr;
  nbc = _nbc;

  if ((nbr < 2 * bord) || (nbc < 2 * bord)) {
    throw vpException(vpException::dimensionError, "border is too important compared to number of row or column.");
  }

  // number of feature = nb column x nb lines in the images
  dim_s = (nbr - 2 * bord) * (nbc - 2 * bord);

  s.resize(dim_s);

  if (pixInfo != NULL)
    delete[] pixInfo;

  pixInfo = new vpLuminance[dim_s];

  Z = _Z;

    dof.resize(6);
    dof = {1,1,1,1,1,1};
}

/*!
  Default constructor that build a visual feature.
*/
ncFeatureLuminance::ncFeatureLuminance() : Z(1), nbr(0), nbc(0), bord(10), pixInfo(NULL), firstTimeIn(0), cam()
{
  nbParameters = 1;
  dim_s = 0;
  flags = NULL;

  init();
}

/*!
 Copy constructor.
 */
ncFeatureLuminance::ncFeatureLuminance(const ncFeatureLuminance &f)
  : vpBasicFeature(f), Z(1), nbr(0), nbc(0), bord(10), pixInfo(NULL), firstTimeIn(0), cam()
{
  *this = f;
}

/*!
 Copy operator.
 */
ncFeatureLuminance &ncFeatureLuminance::operator=(const ncFeatureLuminance &f)
{
  Z = f.Z;
  nbr = f.nbr;
  nbc = f.nbc;
  bord = f.bord;
  firstTimeIn = f.firstTimeIn;
  cam = f.cam;
  if (pixInfo)
    delete[] pixInfo;
  pixInfo = new vpLuminance[dim_s];
  for (unsigned int i = 0; i < dim_s; i++)
    pixInfo[i] = f.pixInfo[i];
  return (*this);
}

/*!
  Destructor that free allocated memory.
*/
ncFeatureLuminance::~ncFeatureLuminance()
{
  if (pixInfo != NULL)
    delete[] pixInfo;
}

/*!
  Set the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \param Z_ : \f$ Z \f$ value to set.
*/
void ncFeatureLuminance::set_Z(const double Z_)
{
  this->Z = Z_;
  flags[0] = true;
}

/*!
  Set the value of \f$ dof={tx,ty,...} \f$ which represents the used degrees of freedom.

  \param tx: \f$dof tx \f$ is used or not.
  \param ty: ...
*/
void ncFeatureLuminance::set_dof(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz){
    dof={tx,ty,tz,rx,ry,rz};
}


/*!
  Get the number the used degrees of freedom.

   \return Number of dof
*/
int ncFeatureLuminance::get_Ndof(){
    int n=0;
    for(int i=0;i<dof.size();i++){
        if(dof[i]){
            n++;
        }
    }
    return n;
}


/*!
  Get the value of \f$ Z \f$ which represents the depth in the 3D camera
  frame.

  \return The value of \f$ Z \f$.
*/
double ncFeatureLuminance::get_Z() const { return Z; }

void ncFeatureLuminance::setCameraParameters(CameraParameters &_cam) {
    camParam = _cam;
    cam.initPersProjWithoutDistortion(camParam.au, camParam.av, camParam.u0, camParam.v0);
}

/*!

  Build a luminance feature directly from the image
*/

void ncFeatureLuminance::buildFrom(vpImage<unsigned char> &I, vpImage<unsigned char> mask)
{
  unsigned int l = 0;
  double Ix, Iy;

  double px = cam.get_px();
  double py = cam.get_py();

  if (firstTimeIn == 0) {
    firstTimeIn = 1;
    l = 0;
    for (unsigned int i = bord; i < nbr - bord; i++) {
      for (unsigned int j = bord; j < nbc - bord; j++) {
            double x = 0, y = 0;
            vpPixelMeterConversion::convertPoint(cam, j, i, x, y);
            pixInfo[l].x = x;
            pixInfo[l].y = y;
            pixInfo[l].Z = Z;

        l++;
      }
    }
  }

  l = 0;
  for (unsigned int i = bord; i < nbr - bord; i++) {
    for (unsigned int j = bord; j < nbc - bord; j++) {
        if(mask[i][j]<250) {
            Ix = px * vpImageFilter::derivativeFilterX(I, i, j);
            Iy = py * vpImageFilter::derivativeFilterY(I, i, j);

            pixInfo[l].I = I[i][j];
            s[l] = I[i][j];
            pixInfo[l].Ix = Ix;
            pixInfo[l].Iy = Iy;

        }else{
            pixInfo[l].I = I[i][j];
            s[l] = I[i][j];
            pixInfo[l].Ix = 0;
            pixInfo[l].Iy = 0;
        }

      l++;
    }
  }
}

void ncFeatureLuminance::buildFrom(vpImage<unsigned char> &I)
{
    vpImage<float> If;
    vpImageConvert::convert(I, If);
    unsigned int l = 0;
    double Ix, Iy;

    double px = cam.get_px();
    double py = cam.get_py();

    ////ZERO MEAN NORMALIZATION
    float Ifmean=0;
    for (unsigned int i = 0; i < nbr; i++) {
        for (unsigned int j = 0; j < nbc; j++) {
            Ifmean+=If[i][j];
        }
    }
    Ifmean/=nbr*nbc;

    float Ifmeannorm=0;
    for (unsigned int i = 0; i < nbr; i++) {
        for (unsigned int j = 0; j < nbc; j++) {
            If[i][j]-=Ifmean;
            Ifmeannorm+=fabs(If[i][j]);
        }
    }
    Ifmeannorm/=nbr*nbc;
    for (unsigned int i = 0; i < nbr; i++) {
        for (unsigned int j = 0; j < nbc; j++) {
            If[i][j]/=Ifmeannorm;
        }
    }
    /////END OF ZERO MEAN NORMALIZATION



    ////TO REMOVE THE NORMALIZATION CHANGE If by I
    if (firstTimeIn == 0) {
        firstTimeIn = 1;
        l = 0;
        for (unsigned int i = bord; i < nbr - bord; i++) {
            for (unsigned int j = bord; j < nbc - bord; j++) {
                double x = 0, y = 0;
                vpPixelMeterConversion::convertPoint(cam, j, i, x, y);
                pixInfo[l].x = x;
                pixInfo[l].y = y;
                pixInfo[l].Z = Z;
                l++;
            }
        }
    }

    l = 0;
    for (unsigned int i = bord; i < nbr - bord; i++) {
        for (unsigned int j = bord; j < nbc - bord; j++) {
                Ix = px * vpImageFilter::derivativeFilterX(I, i, j);
                Iy = py * vpImageFilter::derivativeFilterY(I, i, j);
                pixInfo[l].I = I[i][j];
                s[l] = I[i][j];
                pixInfo[l].Ix = Ix;
                pixInfo[l].Iy = Iy;

            l++;
        }
    }

}

void ncFeatureLuminance::buildFrom(vpImage<float> &I)
{
    unsigned int l = 0;
    double Ix, Iy;

    double px = cam.get_px();
    double py = cam.get_py();

    if (firstTimeIn == 0) {
        firstTimeIn = 1;
        l = 0;
        for (unsigned int i = bord; i < nbr - bord; i++) {
            for (unsigned int j = bord; j < nbc - bord; j++) {
                double x = 0, y = 0;
                vpPixelMeterConversion::convertPoint(cam, j, i, x, y);
                pixInfo[l].x = x;
                pixInfo[l].y = y;
                pixInfo[l].Z = Z;
                l++;
            }
        }
    }

    l = 0;
    for (unsigned int i = bord; i < nbr - bord; i++) {
        for (unsigned int j = bord; j < nbc - bord; j++) {
            Ix = px * vpImageFilter::derivativeFilterX(I, i, j);
            Iy = py * vpImageFilter::derivativeFilterY(I, i, j);
            pixInfo[l].I = I[i][j];
            s[l] = I[i][j];
            pixInfo[l].Ix = Ix;
            pixInfo[l].Iy = Iy;

            l++;
        }
    }

}

/*!

  Compute and return the interaction matrix \f$ L_I \f$. The computation is
  made thanks to the values of the luminance features \f$ I \f$
*/
void ncFeatureLuminance::interaction(vpMatrix &L)
{
  L.resize(dim_s, get_Ndof());

  for (unsigned int m = 0; m < L.getRows(); m++) {
    double Ix = pixInfo[m].Ix;
    double Iy = pixInfo[m].Iy;

    double x = pixInfo[m].x;
    double y = pixInfo[m].y;
    double Zinv = 1.0 / pixInfo[m].Z;

    int i=0, j=0;

    if(dof[i++]) {
        L[m][j++] = Ix * Zinv;
    }
    if(dof[i++]) {
        L[m][j++] = Iy * Zinv;
    }
    if(dof[i++]) {
      L[m][j++] = -(x * Ix + y * Iy) * Zinv;
    }
    if(dof[i++]) {
      L[m][j++] = -Ix * x * y - (1 + y * y) * Iy;
    }
    if(dof[i++]) {
      L[m][j++] = (1 + x * x) * Ix + Iy * x * y;
    }
    if(dof[i++]) {
      L[m][j++] = Iy * x - Ix * y;
    }

  }
}


void ncFeatureLuminance::interactionOmni(vpMatrix &L) {
    L.resize(dim_s, get_Ndof());

    for (unsigned int m = 0; m < L.getRows(); m++) {
        double xi = camParam.xi;
        double dIdx = -pixInfo[m].Ix;
        double dIdy = -pixInfo[m].Iy;
        double x = pixInfo[m].x;
        double y = pixInfo[m].y;

        double srac = 1.0+(1.0-xi*xi)*(x*x + y*y);
        double fact = (xi + sqrt(srac)) / (x*x + y*y + 1.0);
        double Zs = fact - xi;
        double rho = Z/Zs;

        double Ls[6][2];
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

        int i=0, j=0;


        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[0][0]) + (dIdy * Ls[0][1]);
        }
        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[1][0]) + (dIdy * Ls[1][1]);
        }
        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[2][0]) + (dIdy * Ls[2][1]);
        }
        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[3][0]) + (dIdy * Ls[3][1]);
        }
        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[4][0]) + (dIdy * Ls[4][1]);
        }
        if(dof[i++]) {
            L[m][j++] = (dIdx * Ls[5][0]) + (dIdy * Ls[5][1]);
        }

    }
}

/*!
  Compute and return the interaction matrix \f$ L_I \f$. The computation is
  made thanks to the values of the luminance features \f$ I \f$
*/
vpMatrix ncFeatureLuminance::interaction(const unsigned int /* select */)
{
  /* static */ vpMatrix L; // warning C4640: 'L' : construction of local
                           // static object is not thread-safe
  interaction(L);
  return L;
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param e : Error between the current and the desired features.

*/
void ncFeatureLuminance::error(const vpBasicFeature &s_star, vpColVector &e)
{
    e.resize(dim_s);

    for (unsigned int i = 0; i < dim_s; i++) {
        e[i] = s[i] - s_star[i];
    }
}

/*!
  Compute the error \f$ (I-I^*)\f$ between the current and the desired

  \param s_star : Desired visual feature.
  \param select : Not used.

*/
vpColVector ncFeatureLuminance::error(const vpBasicFeature &s_star, const unsigned int /* select */)
{
  /* static */ vpColVector e; // warning C4640: 'e' : construction of local
                              // static object is not thread-safe


  error(s_star, e);

  return e;
}

/*!

  Not implemented.

 */
void ncFeatureLuminance::print(const unsigned int /* select */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void ncFeatureLuminance::display(const vpCameraParameters & /* cam */, const vpImage<unsigned char> & /* I */,
                                 const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!

  Not implemented.

 */
void ncFeatureLuminance::display(const vpCameraParameters & /* cam */, const vpImage<vpRGBa> & /* I */,
                                 const vpColor & /* color */, unsigned int /* thickness */) const
{
  static int firsttime = 0;

  if (firsttime == 0) {
    firsttime = 1;
    vpERROR_TRACE("not implemented");
    // Do not throw and error since it is not subject
    // to produce a failure
  }
}

/*!
  Create an object with the same type.

  \code
  vpBasicFeature *s_star;
  ncFeatureLuminance s;
  s_star = s.duplicate(); // s_star is now a ncFeatureLuminance
  \endcode

*/
ncFeatureLuminance *ncFeatureLuminance::duplicate() const
{
  ncFeatureLuminance *feature = new ncFeatureLuminance;
  return feature;
}


