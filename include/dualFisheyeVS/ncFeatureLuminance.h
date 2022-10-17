#ifndef ncFeatureLuminance_h
#define ncFeatureLuminance_h

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
class VISP_EXPORT vpLuminance
{
public:
  double x, y;   // point coordinates (in meter)
  double I;      // pixel intensity
  double Ix, Iy; // pixel gradient
  double Z;      // pixel depth
};
#endif


typedef struct CameraParameters{
    int width, height;
    float au, av;
    float u0, v0;
    float xi;
}CameraParameters;

class VISP_EXPORT ncFeatureLuminance : public vpBasicFeature{
protected:
  //! FeaturePoint depth (required to compute the interaction matrix)
  //! default Z = 1m
  double Z;

  //! Number of rows.
  unsigned int nbr;
  //! Number of column.
  unsigned int nbc;
  //! Border size.
  unsigned int bord;

  std::vector<bool> dof;

  //! Store the image (as a vector with intensity and gradient I, Ix, Iy)
  vpLuminance *pixInfo;
  int firstTimeIn;

public:
  ncFeatureLuminance();
  ncFeatureLuminance(const ncFeatureLuminance &f);
  //! Destructor.
  virtual ~ncFeatureLuminance();

  void buildFrom(vpImage<unsigned char> &I);
    void buildFrom(vpImage<float> &I);
    void buildFrom(vpImage<unsigned char> &I, vpImage<unsigned char> mask);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  ncFeatureLuminance *duplicate() const;

  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);
  void error(const vpBasicFeature &s_star, vpColVector &e);
  //! Compute the error between a visual features and zero
  vpColVector error(const unsigned int select = FEATURE_ALL);


  void init();
  void init(unsigned int _nbr, unsigned int _nbc, double _Z);
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);
  void interaction(vpMatrix &L);
  void interactionOmni(vpMatrix &L);


    ncFeatureLuminance &operator=(const ncFeatureLuminance &f);

  void print(const unsigned int select = FEATURE_ALL) const;

  void setCameraParameters(CameraParameters &_cam);
  void set_Z(const double Z);
  void set_dof(bool tx, bool ty, bool tz, bool rx, bool ry, bool rz);

  int get_Ndof();
  double get_Z() const;




public:
  vpCameraParameters cam;
  CameraParameters camParam;
};

#endif

