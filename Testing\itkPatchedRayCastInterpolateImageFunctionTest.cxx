#if defined(_MSC_VER)
#pragma warning ( disable : 4786 )
#endif

#include <iostream>

#include <itkImage.h>
#include <itkTranslationTransform.h>
#include <itkLinearInterpolateImageFunction.h>

#include "itkPatchedRayCastInterpolateImageFunction.h"

/*
 * This program tests the PatchedRayCastInterpolateImageFunction class.
 * Multiple projections are tested on a synthetic image, checking if the
 * resulting rays' values are within the accepted tolerance.
 *
 */
int main( int argc, char * argv[] )
{
  std::cout << "Testing PatchedRayCastInterpolateImageFunction:\n";

  bool testOk = true;

  typedef float PixelType;
  const unsigned int ImageDimension = 3;

  typedef itk::Image< PixelType, ImageDimension > ImageType;

  typedef ImageType::IndexType    IndexType;
  typedef ImageType::PointType    PointType;
  typedef ImageType::SpacingType  SpacingType;
  typedef ImageType::SizeType     SizeType;
  typedef ImageType::RegionType   RegionType;
  
  /* Allocate a simple test image, a 1x1x1mm cube made of 40x40x40 voxels.
     Each voxel has a value of 2.0, so if we ask the interpolator to integrate
     along each axis with a threshold of 1.0 we should obtain a value equal to
     the length of each of the cube's edges, i.e 1.0.*/
  ImageType::Pointer image = ImageType::New();
  IndexType start;
  start.Fill(0);
  SizeType size;
  size[0] = 40;
  size[1] = 40;
  size[2] = 40;

  RegionType region;
  region.SetIndex(start);
  region.SetSize(size);
  image->SetRegions(region);
  image->Allocate();

  PointType origin;
  origin.Fill(0.0);

  SpacingType spacing;
  spacing.Fill(0.025);

  /* Set origin and spacing of physical coordinates */
  image->SetOrigin(origin);
  image->SetSpacing(spacing);

  /* Initialize the image contents */
  IndexType index;
  for (unsigned int slice = 0; slice < size[2]; slice++) 
    {
    index[2] = slice;
    for (unsigned int row = 0; row < size[1]; row++) 
      {
      index[1] = row;
      for (unsigned int col = 0; col < size[0]; col++) 
        {
        index[0] = col;
        PixelType value = (PixelType)(2.0);
        image->SetPixel(index,value);
        }
      }
    }

  typedef itk::PatchedRayCastInterpolateImageFunction<
                    ImageType,double> RayCastInterpolatorType;

  /* Create and initialize the interpolator */
  RayCastInterpolatorType::Pointer interp = RayCastInterpolatorType::New();
  interp->SetInputImage(image);
  interp->Print( std::cout );


  /* Create the transform */
  typedef itk::TranslationTransform<
                  double,ImageDimension>  TransformType;

  TransformType::Pointer transform = TransformType::New();
  interp->SetTransform( transform );

  /* Create the auxiliary interpolator */
  typedef itk::LinearInterpolateImageFunction< 
                  ImageType, double > InterpolatorType;

  InterpolatorType::Pointer auxInterpolator = InterpolatorType::New();
  
  interp->SetInterpolator( auxInterpolator );


  /* Exercise the SetThreshold() method */
  interp->SetThreshold( 1.0 );

  /* Evaluate the function integrating across the cube's faces. 
  In theory, all integrals should be equal to 1.0. However, results
  should be equal to 0.975. */
  double tol = 0.25;

  double integral;
  PointType query;
  PointType focus;

  focus[0] = 100.0;
  focus[1] =  0.45;
  focus[2] =  0.45;
  interp->SetFocalPoint( focus );
  query[0] =  -10.0;
  query[1] =   0.45;
  query[2] =   0.45;
  integral = interp->Evaluate(query);
  std::cout << "Integral along x = " << integral << std::endl;
  if( abs(integral - 1.0) > tol )
    {
    std::cerr << "ERROR: Integral along has an incorrect value" << std::endl;
    testOk = false;
    }

  focus[0] =  0.0;
  focus[1] = 100.0;
  focus[2] =  0.45;
  interp->SetFocalPoint( focus );
  query[0] =   0.45;
  query[1] = -10.0;
  query[2] =   0.45;
  integral = interp->Evaluate(query);
  std::cout << "Integral along y = " << integral << std::endl;
  if( abs(integral - 1.0) > tol )
    {
    std::cerr << "ERROR: Integral along has an incorrect value" << std::endl;
    testOk = false;
    }

  focus[0] =  0.45;
  focus[1] =  0.45;
  focus[2] = 100.0;
  interp->SetFocalPoint( focus );
  query[0] =   0.45;
  query[1] =   0.45;
  query[2] = -10.00;
  integral = interp->Evaluate(query);
  std::cout << "Integral along z = " << integral << std::endl;
  if( abs(integral - 1.0) > tol )
    {
    std::cerr << "ERROR: Integral along has an incorrect value" << std::endl;
    testOk = false;
    }

  if( testOk == true )
    return EXIT_SUCCESS;
  else
    return EXIT_FAILURE;
}
