
#ifdef _MSC_VER
#pragma warning ( disable : 4786 )
#endif

#include <itkImageMaskSpatialObject.h>
#include <itkNumericTraits.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkTranslationTransform.h>

#include "itkMeanSquaresMultiImageToImageMetric.h"
#include "itkMultiImageToImageRegistrationMethod.h"
#include "itkPatchedRayCastInterpolateImageFunction.h"

#include <vector>


/** 
 *  This program test one instantiation of the 
 * itk::MultiImageRegistrationMethod class.
 * 
 *  This file tests initialization errors.
 *
 */ 
int main(int argc, char* argv[] )
{
  const unsigned int Dimension = 3;
  const unsigned int FImgTotal = 2;

  typedef itk::Image<float,Dimension> FixedImageType;
  typedef itk::Image<char,Dimension>  MovingImageType;

  typedef itk::MultiImageToImageRegistrationMethod< 
                                    FixedImageType, 
                                    MovingImageType > RegistrationType;
  RegistrationType::Pointer registration = RegistrationType::New();


//----------------------------------------------------------------------------
// Create the fixed images
//----------------------------------------------------------------------------
  typedef itk::ImageMaskSpatialObject<Dimension>  FixedMaskType;
  typedef itk::Image<unsigned char,Dimension>     FixedMaskImageType;

  FixedImageType::SizeType    size;
  size.Fill( 4 );  // the size of image have to be at least 4 in each dimension to
                   // compute gradient image inside the metric.
  FixedImageType::RegionType  region( size );

  for( unsigned int f = 0; f<FImgTotal; f++ )
    {
    FixedImageType::Pointer fixedImage = FixedImageType::New();

    fixedImage->SetRegions( region );
    fixedImage->Allocate();
    fixedImage->FillBuffer( 3.0 );

    FixedMaskImageType::Pointer fixedMaskImage = FixedMaskImageType::New();
    fixedMaskImage->SetRegions( region );
    fixedMaskImage->Allocate();
    fixedMaskImage->FillBuffer( 255 );

    FixedMaskType::Pointer fixedMask = FixedMaskType::New();
    fixedMask->SetImage( fixedMaskImage );

    registration->AddFixedImage( fixedImage );
    registration->AddFixedImageMask( fixedMask );
    }


//----------------------------------------------------------------------------
// Create the moving image
//----------------------------------------------------------------------------
  MovingImageType::Pointer  movingImage       = MovingImageType::New();  

  movingImage->SetRegions( region );
  movingImage->Allocate();
  movingImage->FillBuffer( 4 );

  registration->SetMovingImage( movingImage );
  

//----------------------------------------------------------------------------
// Create the transform
//----------------------------------------------------------------------------
  typedef itk::TranslationTransform< double, Dimension > TransformType;

  TransformType::Pointer    transform         = TransformType::New();

  registration->SetTransform( transform );


//----------------------------------------------------------------------------
// Create the multi metric
//----------------------------------------------------------------------------
  typedef itk::MeanSquaresMultiImageToImageMetric< 
                                    FixedImageType, 
                                    MovingImageType >    MultiMetricType;

  MultiMetricType::Pointer multiMetric = MultiMetricType::New();

  registration->SetMultiMetric( multiMetric );


//----------------------------------------------------------------------------
// Create the optimizer
//----------------------------------------------------------------------------
  typedef itk::RegularStepGradientDescentOptimizer OptimizerType;

  OptimizerType::Pointer optimizer = OptimizerType::New();

  registration->SetOptimizer( optimizer );
  

//----------------------------------------------------------------------------
// Create the interpolators
//----------------------------------------------------------------------------
  typedef itk::PatchedRayCastInterpolateImageFunction< 
                                    MovingImageType,
                                    double          >   InterpolatorType;
  
  for( unsigned int f = 0; f<FImgTotal; f++ )
    {
    InterpolatorType::Pointer interpolator = InterpolatorType::New();
    registration->AddInterpolator( interpolator );
    }


//----------------------------------------------------------------------------
// Set the initial parameters
//----------------------------------------------------------------------------
  typedef RegistrationType::ParametersType            ParametersType;
  ParametersType initialParameters( transform->GetNumberOfParameters() );
  initialParameters.Fill( itk::NumericTraits<ParametersType::ValueType>::Zero );
  registration->SetInitialTransformParameters( initialParameters );


//----------------------------------------------------------------------------
// Exercise miscellaneous methods
//----------------------------------------------------------------------------
  registration->Initialize();

  registration->Print( std::cout );

  std::cout << "MovingImage: " << registration->GetMovingImage() << std::endl;
  std::cout << "FixedMultiImage[0]: " << registration->GetFixedMultiImage()[0] << std::endl;
  std::cout << "FixedMultiImage[1]: " << registration->GetFixedMultiImage()[1] << std::endl;
  std::cout << "Transform: " << registration->GetTransform() << std::endl;
  std::cout << "MultiMetric: " << registration->GetMultiMetric() << std::endl;
  std::cout << "Optimizer: " << registration->GetOptimizer() << std::endl;
  std::cout << "MultiInterpolator[0]: " << registration->GetMultiInterpolator()[0] << std::endl;
  std::cout << "MultiInterpolator[1]: " << registration->GetMultiInterpolator()[1] << std::endl;
  std::cout << "FixedMultiImageRegion[0]: " << registration->GetFixedMultiImageRegion()[0] << std::endl;
  std::cout << "FixedMultiImageRegion[1]: " << registration->GetFixedMultiImageRegion()[1] << std::endl;
  std::cout << "FixedMultiImageMask[0]: " << registration->GetFixedMultiImageMask()[0] << std::endl;
  std::cout << "FixedMultiImageMask[1]: " << registration->GetFixedMultiImageMask()[1] << std::endl;
  std::cout << "InitialParameters: " << registration->GetInitialTransformParameters() << std::endl;


//----------------------------------------------------------------------------
// Test out initialization errors
//----------------------------------------------------------------------------
  bool pass;

#define TEST_INITIALIZATION( ComponentName, goodComponent ) \
  registration->Set##ComponentName( goodComponent ); \
  try \
    { \
    registration->Initialize(); \
    } \
  catch( itk::ExceptionObject& err ) \
    { \
    std::cout << err.GetDescription() << std::endl; \
    std::cout << "Test failed." << std::endl; \
    return EXIT_FAILURE; \
    } \
  \

#define TEST_INITIALIZATION_ERROR( ComponentName, badComponent, goodComponent ) \
  registration->Set##ComponentName( badComponent ); \
  try \
    { \
    pass = false; \
    registration->Update(); \
    } \
  catch( itk::ExceptionObject& err ) \
    { \
    std::cout << "Caught expected ExceptionObject" << std::endl; \
    std::cout << err << std::endl; \
    pass = true; \
    } \
  registration->Set##ComponentName( goodComponent ); \
  \
  if( !pass ) \
    { \
    std::cout << "Test failed." << std::endl; \
    return EXIT_FAILURE; \
    } \
  \


  typedef RegistrationType::FixedMultiImageType       FixedMultiImageType;
  typedef RegistrationType::MultiInterpolatorType     MultiInterpolatorType;
  typedef RegistrationType::FixedMultiImageRegionType FixedMultiImageRegionType;
  typedef RegistrationType::FixedMultiImageMaskType   FixedMultiImageMaskType;

    
  TEST_INITIALIZATION_ERROR( MovingImage, NULL, movingImage );
  TEST_INITIALIZATION_ERROR( Transform, NULL, transform );
  TEST_INITIALIZATION_ERROR( MultiMetric, NULL, multiMetric );
  TEST_INITIALIZATION_ERROR( Optimizer, NULL, optimizer );
  
  ParametersType badParameters;
  TEST_INITIALIZATION_ERROR( InitialTransformParameters, badParameters, initialParameters );
  
  FixedMultiImageType fixedMultiImage = registration->GetFixedMultiImage();
  FixedMultiImageType badFixedMultiImage;
  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  badFixedMultiImage.push_back( fixedMultiImage[0] );
  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  
  MultiInterpolatorType multiInterpolator = registration->GetMultiInterpolator();
  MultiInterpolatorType badMultiInterpolator;
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );
  badMultiInterpolator.push_back( multiInterpolator[0] );
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );

// Registration should not fail with an empty fixedMultiImageRegion. In that
// case the images' buffered regions will be used instead.
  FixedMultiImageRegionType fixedMultiImageRegion = registration->GetFixedMultiImageRegion();
  FixedMultiImageRegionType badFixedMultiImageRegion;
  badFixedMultiImageRegion.push_back( fixedMultiImageRegion[0] );
  TEST_INITIALIZATION_ERROR( FixedMultiImageRegion, badFixedMultiImageRegion, fixedMultiImageRegion );


  FixedMultiImageMaskType goodFixedMultiImageMask = registration->GetFixedMultiImageMask();
// Registration should work when no masks are defined
  FixedMultiImageMaskType tempFixedMultiImageMask;
  TEST_INITIALIZATION( FixedMultiImageMask, tempFixedMultiImageMask );
// ...but not when the number of masks is not equal to the number of fixed images
  FixedMultiImageMaskType badFixedMultiImageMask;
  badFixedMultiImageMask.push_back( goodFixedMultiImageMask[0] ); 
  TEST_INITIALIZATION_ERROR( FixedMultiImageMask, badFixedMultiImageMask, goodFixedMultiImageMask );
// Null elements are allowed in the image masks
  tempFixedMultiImageMask.push_back( goodFixedMultiImageMask[0] );
  tempFixedMultiImageMask.push_back( NULL );
  TEST_INITIALIZATION( FixedMultiImageMask, tempFixedMultiImageMask );

  registration->SetFixedMultiImageMask( goodFixedMultiImageMask );

  

//----------------------------------------------------------------------------
// End of test
//----------------------------------------------------------------------------
  std::cout << "Test passed." << std::endl;
  return EXIT_SUCCESS;
}
