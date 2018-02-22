
#ifdef _MSC_VER
#pragma warning ( disable : 4786 )
#endif

#include <itkInterpolateImageFunction.h>
#include <itkMultiResolutionPyramidImageFilter.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkTranslationTransform.h>

#include "itkMeanSquaresMultiImageToImageMetric.h"
#include "itkMultiResolutionMultiImageToImageRegistrationMethod.h"
#include "itkPatchedRayCastInterpolateImageFunction.h"

#include <vector>


/** 
 *  This program tests one instantiation of the 
 * itk::MultiResolutionMultiImageRegistrationMethod class.
 * 
 *  This file tests initialization errors.
 * 
 */ 
int main(int argc, char* argv[] )
{
  const unsigned int FImgTotal = 2;

  bool pass;

  const unsigned int dimension = 3;

  // Fixed Image Type
  typedef itk::Image<float,dimension>                    FixedImageType;
  typedef std::vector<FixedImageType::ConstPointer>      FixedMultiImageType;

  // Moving Image Type
  typedef itk::Image<char,dimension>                     MovingImageType;

  // Transform Type
  typedef itk::TranslationTransform< double, dimension > TransformType;

  // Optimizer Type
  typedef itk::RegularStepGradientDescentOptimizer              OptimizerType;

  // Metric Type
  typedef itk::MeanSquaresMultiImageToImageMetric< 
                                    FixedImageType, 
                                    MovingImageType >    MultiMetricType;

  // Interpolation technique
  typedef itk::InterpolateImageFunction<
                                    MovingImageType,
                                    double>             BaseInterpolatorType;
  typedef itk::PatchedRayCastInterpolateImageFunction< 
                                    MovingImageType,
                                    double          >   InterpolatorType;
  typedef std::vector<BaseInterpolatorType::Pointer>    MultiInterpolatorType;

  
  // Fixed Image Pyramid Type
  typedef itk::MultiResolutionPyramidImageFilter<
                                    FixedImageType,
                                    FixedImageType  >  FixedImagePyramidType;
  typedef std::vector<FixedImagePyramidType::Pointer>  FixedMultiImagePyramidType;

  // Fixed image region Type
  typedef FixedImageType::RegionType        FixedImageRegionType;
  typedef std::vector<FixedImageRegionType> FixedMultiImageRegionType;

  // Moving Image Pyramid Type
  typedef itk::MultiResolutionPyramidImageFilter<
                                    MovingImageType,
                                    MovingImageType  >   MovingImagePyramidType;

  // Registration Method
  typedef itk::MultiResolutionMultiImageToImageRegistrationMethod< 
                                    FixedImageType, 
                                    MovingImageType >    RegistrationType;


  MultiMetricType::Pointer        multiMetric       = MultiMetricType::New();
  TransformType::Pointer          transform         = TransformType::New();
  OptimizerType::Pointer          optimizer         = OptimizerType::New();
  FixedMultiImageType             fixedMultiImage;
  MovingImageType::Pointer        movingImage       = MovingImageType::New();  
  MultiInterpolatorType           multiInterpolator;
  RegistrationType::Pointer       registration      = RegistrationType::New();
  FixedMultiImagePyramidType      fixedMultiImagePyramid;
  MovingImagePyramidType::Pointer movingImagePyramid = MovingImagePyramidType::New();
  FixedMultiImageRegionType       fixedMultiImageRegion;

  FixedImageType::SizeType    size;
  size.Fill( 8 );  // the size of image have to be at least 4 in each dimension to
                   // compute gradient image inside the metric.
  FixedImageType::RegionType  region( size );

//Create the fixed images
  for( unsigned int f = 0; f<FImgTotal; f++ )
    {
    FixedImageType::Pointer fixedImage = FixedImageType::New();

    fixedImage->SetRegions( region );
    fixedImage->Allocate();
    fixedImage->FillBuffer( 3.0 );

    registration->AddFixedImage( fixedImage );
    registration->AddFixedImageRegion( fixedImage->GetBufferedRegion() );

    FixedImagePyramidType::Pointer fixedImagePyramid = FixedImagePyramidType::New();
    registration->AddFixedImagePyramid( fixedImagePyramid );

    InterpolatorType::Pointer interpolator = InterpolatorType::New();
    registration->AddInterpolator( interpolator );
    }

  // Create the moving image
  movingImage->SetRegions( region );
  movingImage->Allocate();
  movingImage->FillBuffer( 4 );

  registration->SetMovingImage( movingImage );
  registration->SetMovingImagePyramid( movingImagePyramid );

  registration->SetMultiMetric( multiMetric );
  registration->SetOptimizer( optimizer );
  registration->SetTransform( transform );

  registration->SetNumberOfLevels( 2 );

  registration->Print( std::cout );

  // Exercise Get methods
  std::cout << "multiMetric: " << registration->GetMultiMetric() << std::endl;
  std::cout << "optimizer: " << registration->GetOptimizer() << std::endl;
  std::cout << "transform: " << registration->GetTransform() << std::endl;
  std::cout << "fixedMultiImage[0]: " << registration->GetFixedMultiImage()[0] << std::endl;
  std::cout << "fixedMultiImage[1]: " << registration->GetFixedMultiImage()[1] << std::endl;
  std::cout << "moving image: " << registration->GetMovingImage() << std::endl;
  std::cout << "multiInterpolator[0]: " << registration->GetMultiInterpolator()[0] << std::endl;
  std::cout << "multiInterpolator[1]: " << registration->GetMultiInterpolator()[1] << std::endl;

  std::cout << "initial parameters: ";
  std::cout << registration->GetInitialTransformParameters() << std::endl;

  typedef RegistrationType::ParametersType ParametersType;
  ParametersType initialParameters( transform->GetNumberOfParameters() );
  initialParameters.Fill(0);

  ParametersType badParameters( 2 );
  badParameters.Fill( 5 );

  fixedMultiImage = registration->GetFixedMultiImage();
  multiInterpolator = registration->GetMultiInterpolator();
  FixedMultiImageType   badFixedMultiImage;
  MultiInterpolatorType badMultiInterpolator;

  registration->SetInitialTransformParameters( initialParameters );

  std::cout << registration;
  /****************************************************
   * Test out initialization errors
   ****************************************************/

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
    } 

  TEST_INITIALIZATION_ERROR( InitialTransformParameters, badParameters, initialParameters );
  TEST_INITIALIZATION_ERROR( MultiMetric, NULL, multiMetric );
  TEST_INITIALIZATION_ERROR( Optimizer, NULL, optimizer );
  TEST_INITIALIZATION_ERROR( Transform, NULL, transform );
  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  TEST_INITIALIZATION_ERROR( MovingImage, NULL, movingImage );
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );

  std::cout << "Test passed." << std::endl;
  return EXIT_SUCCESS;


}
