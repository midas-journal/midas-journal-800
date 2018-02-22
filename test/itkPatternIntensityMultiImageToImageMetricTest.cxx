#if defined(_MSC_VER)
#pragma warning ( disable : 4786 )
#endif
#include <itkImage.h>
#include <itkImageFileWriter.h>
#include <itkImageRegionIterator.h>
#include <itkTranslationTransform.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkGaussianImageSource.h>
#include <itkResampleImageFilter.h>

#include "itkPatchedRayCastInterpolateImageFunction.h"
#include "itkPatternIntensityMultiImageToImageMetric.h"

#include <iostream>


/**
 * Tests the PatternIntensityMultiImageMetric with a synthetic image of a
 * Gaussian, centered at the origin. Fixed images are projections along the x
 * axis (focal distance = 200 mm, distance from origin to detector = 40 mm)
 * and along the y axis (same parameters as the first one ).
 *
 * A transalation transform is applied to the moving image, displacing it
 * along the y axis and measuring the metric's value at regular intervals.
 *
 */

int main( int argc, char* argv[] )
{

  const unsigned int ImageDimension = 3;
  typedef float                   PixelType;

  typedef double                   CoordinateRepresentationType;

  typedef itk::Image<PixelType,ImageDimension>        MovingImageType;
  typedef itk::Image<PixelType,ImageDimension>        FixedImageType;
  typedef std::vector<FixedImageType::ConstPointer>   FixedMultiImageType;


//------------------------------------------------------------
// Create the Moving image
//------------------------------------------------------------

// Declare Gaussian Sources
  typedef itk::GaussianImageSource< MovingImageType >  MovingImageSourceType;
  typedef MovingImageSourceType::Pointer               MovingImageSourcePointer;

  MovingImageType::SizeValueType movingImageSize[]  = {  41,  41, 41 };
  double movingImageSpacing[]  = { 1.0f, 1.0f, 1.0f };
  double movingImageOrigin[] = { -20.0f, -20.0f, -20.0f };

  MovingImageSourceType::ArrayType movingImageMean;
  movingImageMean[0] = 0.0f;
  movingImageMean[1] = 0.0f;
  movingImageMean[2] = 0.0f;

  MovingImageSourceType::ArrayType movingImageSigma;
  movingImageSigma[0] = 8.0f;
  movingImageSigma[1] = 8.0f;
  movingImageSigma[2] = 8.0f;

  MovingImageSourceType::Pointer movingImageSource = MovingImageSourceType::New();

  movingImageSource->SetSize(    movingImageSize    );
  movingImageSource->SetOrigin(  movingImageOrigin  );
  movingImageSource->SetSpacing( movingImageSpacing );
  movingImageSource->SetNormalized( false );
  movingImageSource->SetScale( 250.0f );
  movingImageSource->SetMean( movingImageMean );
  movingImageSource->SetSigma( movingImageSigma );

  movingImageSource->Update(); // Force the filter to run

  MovingImageType::Pointer movingImage = movingImageSource->GetOutput();

  //typedef itk::ImageFileWriter<MovingImageType> MovingImageWriterType;
  //MovingImageWriterType::Pointer movingImageWriter = MovingImageWriterType::New();
  //movingImageWriter->SetFileName( "movingImage.mha" );
  //movingImageWriter->SetInput( movingImageSource->GetOutput() );
  //try {
  //  movingImageWriter->Update();
  //  }
  //catch( itk::ExceptionObject &e )
  //  {
  //  std::cerr << e.GetDescription() << std::endl;
  //  return EXIT_FAILURE;
  //  }

//-----------------------------------------------------------
// Set up a Transform
//-----------------------------------------------------------

  typedef itk::TranslationTransform<
                        CoordinateRepresentationType,
                        ImageDimension >         TransformType;

  TransformType::Pointer transform = TransformType::New();
  transform->SetIdentity();

  TransformType::Pointer identity = TransformType::New();
  identity->SetIdentity();

//-----------------------------------------------------------
// Set up  the Metric
//-----------------------------------------------------------
  typedef itk::PatternIntensityMultiImageToImageMetric<
    FixedImageType, MovingImageType > MultiMetricType;

  typedef MultiMetricType::TransformType    TransformBaseType;
  typedef TransformBaseType::ParametersType ParametersType;
  typedef TransformBaseType::JacobianType   JacobianType;

  MultiMetricType::Pointer multiMetric = MultiMetricType::New();

//-----------------------------------------------------------
// Create the Fixed images
//-----------------------------------------------------------
  FixedMultiImageType fixedMultiImage;

  typedef FixedImageType::RegionType        FixedImageRegionType;
  typedef std::vector<FixedImageRegionType> FixedMultiImageRegionType;
  FixedMultiImageRegionType fixedMultiImageRegion;

  typedef itk::PatchedRayCastInterpolateImageFunction<MovingImageType,
    CoordinateRepresentationType> RayCasterType;
  typedef MultiMetricType::InterpolatorType       InterpolatorType;
  typedef MultiMetricType::MultiInterpolatorType  MultiInterpolatorType;
  MultiInterpolatorType multiInterpolator;

  typedef itk::ResampleImageFilter<MovingImageType,FixedImageType> ResamplerType;
  typedef std::vector<ResamplerType::Pointer> MultiResamplerType;
  MultiResamplerType multiResampler;


// Projection along x, 0 degrees
  RayCasterType::Pointer interpolator = RayCasterType::New();
  interpolator->SetInputImage( movingImage );
  interpolator->SetTransform( transform );
  multiInterpolator.push_back( (RayCasterType*)interpolator );

  multiResampler.push_back( ResamplerType::New() );

  ResamplerType::SizeType fixedImageSize;
  fixedImageSize[0] = 101;
  fixedImageSize[1] = 101;
  fixedImageSize[2] = 1;

  ResamplerType::SpacingType fixedImageSpacing( ImageDimension );
  fixedImageSpacing[0] = 0.67f;
  fixedImageSpacing[1] = 0.67f;
  fixedImageSpacing[2] = 0.67f;

  multiResampler[0]->SetInput( movingImage );
  multiResampler[0]->SetInterpolator( multiInterpolator[0] );
  multiResampler[0]->SetTransform( identity );
  multiResampler[0]->SetSize( fixedImageSize );
  multiResampler[0]->SetOutputSpacing( fixedImageSpacing );
  multiResampler[0]->SetDefaultPixelValue( 0.0f );

  RayCasterType::InputPointType fixedImageFocalPoint;
  fixedImageFocalPoint[0] = 200.0f;
  fixedImageFocalPoint[1] = 0.0f;
  fixedImageFocalPoint[2] = 0.0f;

  ResamplerType::OriginPointType fixedImageOrigin;
  fixedImageOrigin[0] = -40.0f;
  fixedImageOrigin[1] = -33.5f;
  fixedImageOrigin[2] = -33.5f;

  ResamplerType::DirectionType fixedImageDirection;
  fixedImageDirection[0][0] = 0.0f;
  fixedImageDirection[1][0] = 1.0f;
  fixedImageDirection[2][0] = 0.0f;
  fixedImageDirection[0][1] = 0.0f;
  fixedImageDirection[1][1] = 0.0f;
  fixedImageDirection[2][1] = 1.0f;
  fixedImageDirection[0][2] = 1.0f;
  fixedImageDirection[1][2] = 0.0f;
  fixedImageDirection[2][2] = 0.0f;

  dynamic_cast<RayCasterType*>(multiInterpolator[0].GetPointer())->SetFocalPoint( fixedImageFocalPoint );
  multiResampler[0]->SetOutputOrigin( fixedImageOrigin );
  multiResampler[0]->SetOutputDirection( fixedImageDirection );

  try {
    multiResampler[0]->Update();
    }
  catch( itk::ExceptionObject &e )
    {
    std::cerr << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }

  fixedMultiImage.push_back( multiResampler[0]->GetOutput() );
  fixedMultiImageRegion.push_back( fixedMultiImage[0]->GetLargestPossibleRegion() );


// Projection along y, 90 degrees
  interpolator = RayCasterType::New();
  interpolator->SetInputImage( movingImage );
  interpolator->SetTransform( transform );
  multiInterpolator.push_back( (RayCasterType*)interpolator );

  multiResampler.push_back( ResamplerType::New() );

  multiResampler[1]->SetInput( movingImage );
  multiResampler[1]->SetInterpolator( multiInterpolator[1] );
  multiResampler[1]->SetTransform( identity );
  multiResampler[1]->SetSize( fixedImageSize );
  multiResampler[1]->SetOutputSpacing( fixedImageSpacing );
  multiResampler[1]->SetDefaultPixelValue( 0.0f );

  fixedImageFocalPoint[0] =   0.0f;
  fixedImageFocalPoint[1] = 200.0f;
  fixedImageFocalPoint[2] =   0.0f;

  fixedImageOrigin[0] =  33.5f;
  fixedImageOrigin[1] = -40.0f;
  fixedImageOrigin[2] = -33.5f;

  fixedImageDirection[0][0] = -1.0f;
  fixedImageDirection[1][0] = 0.0f;
  fixedImageDirection[2][0] = 0.0f;
  fixedImageDirection[0][1] = 0.0f;
  fixedImageDirection[1][1] = 0.0f;
  fixedImageDirection[2][1] = 1.0f;
  fixedImageDirection[0][2] = 0.0f;
  fixedImageDirection[1][2] = 1.0f;
  fixedImageDirection[2][2] = 0.0f;

  dynamic_cast<RayCasterType*>(multiInterpolator[1].GetPointer())->SetFocalPoint( fixedImageFocalPoint );
  multiResampler[1]->SetOutputOrigin( fixedImageOrigin );
  multiResampler[1]->SetOutputDirection( fixedImageDirection );

  try {
    multiResampler[1]->Update();
    }
  catch( itk::ExceptionObject &e )
    {
    std::cerr << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }

  fixedMultiImage.push_back( multiResampler[1]->GetOutput() );
  fixedMultiImageRegion.push_back( fixedMultiImage[1]->GetLargestPossibleRegion() );


  //typedef itk::ImageFileWriter<FixedImageType> FixedImageWriterType;
  //FixedImageWriterType::Pointer fixedImageWriter = FixedImageWriterType::New();

  //fixedImageWriter->SetInput( fixedMultiImage[0] );
  //fixedImageWriter->SetFileName( "projection00.mha" );
  //try {
  //  fixedImageWriter->Update();
  //  }
  //catch( itk::ExceptionObject &e )
  //  {
  //  std::cerr << e.GetDescription() << std::endl;
  //  return EXIT_FAILURE;
  //  }

  //fixedImageWriter->SetInput( fixedMultiImage[1] );
  //fixedImageWriter->SetFileName( "projection90.mha" );
  //try {
  //  fixedImageWriter->Update();
  //  }
  //catch( itk::ExceptionObject &e )
  //  {
  //  std::cerr << e.GetDescription() << std::endl;
  //  return EXIT_FAILURE;
  //  }


//-----------------------------------------------------------
// Plug the Images into the metric
//-----------------------------------------------------------
  multiMetric->SetMovingImage( movingImage );
  multiMetric->SetFixedMultiImage( fixedMultiImage );
  multiMetric->SetFixedMultiImageRegion( fixedMultiImageRegion );
  multiMetric->SetMultiInterpolator( multiInterpolator );
  multiMetric->SetTransform( transform );
  multiMetric->SetSigma( 100.0f );


//------------------------------------------------------------
// This call is mandatory before start querying the Metric
// This method do all the necesary connections between the
// internal components: Interpolator, Transform and Images
//------------------------------------------------------------
  try {
    multiMetric->Initialize();
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << "MultiMetric initialization failed" << std::endl;
    std::cout << "Reason " << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }


//---------------------------------------------------------
// Print out metric values
// for parameters[1] = {-5,5}  (arbitrary choice...)
//---------------------------------------------------------

  MultiMetricType::MeasureType     measure;
  MultiMetricType::DerivativeType  derivative;

  TransformType::ParametersType parameters(
    transform->GetNumberOfParameters() );
  parameters.Fill( itk::NumericTraits<ParametersType::ValueType>::Zero );

  std::cout << "param[1]   Metric    d(Metric)/d(param[1] " << std::endl;

  for( double trans = -5; trans <= 5; trans += 0.2  )
    {
    parameters[1] = trans;
    multiMetric->GetValueAndDerivative( parameters, measure, derivative );

    std::cout.width(5);
    std::cout.precision(5);
    std::cout << trans;
    std::cout.width(15);
    std::cout.precision(5);
    std::cout << measure;
    std::cout.width(15);
    std::cout.precision(5);
    std::cout << derivative[1];
    std::cout << std::endl;

    // exercise the other functions
    multiMetric->GetValue( parameters );
    multiMetric->GetDerivative( parameters, derivative );

    }


//-------------------------------------------------------
// exercise Print() method
//-------------------------------------------------------
  multiMetric->Print( std::cout );



//-------------------------------------------------------
// exercise misc member functions
//-------------------------------------------------------
  std::cout << "FixedMultiImage[0]: " << multiMetric->GetFixedMultiImage()[0] << std::endl;
  std::cout << "FixedMultiImage[1]: " << multiMetric->GetFixedMultiImage()[1] << std::endl;
  std::cout << "MovingImage: " << multiMetric->GetMovingImage() << std::endl;
  std::cout << "Transform: " << multiMetric->GetTransform() << std::endl;
  std::cout << "MultiInterpolator[0]: " << multiMetric->GetMultiInterpolator()[0] << std::endl;
  std::cout << "MultiInterpolator[1]: " << multiMetric->GetMultiInterpolator()[1] << std::endl;
  std::cout << "FixedMultiImageRegion[0]: " << multiMetric->GetFixedMultiImageRegion()[0] << std::endl;
  std::cout << "FixedMultiImageRegion[1]: " << multiMetric->GetFixedMultiImageRegion()[1] << std::endl;
  std::cout << "Derivative delta: " << multiMetric->GetDerivativeDelta() << std::endl;
  std::cout << "Radius: " << multiMetric->GetRadius() << std::endl;
  std::cout << "Sigmat: " << multiMetric->GetSigma() << std::endl;

  std::cout << "Check case when FixedMultiImage is empty" << std::endl;
  FixedMultiImageType badFixedMultiImage;
  multiMetric->SetFixedMultiImage( badFixedMultiImage );
  try
    {
    std::cout << "Value = " << multiMetric->GetValue( parameters );
    std::cout << "If you are reading this message the Metric " << std::endl;
    std::cout << "is NOT managing exceptions correctly    " << std::endl;
    return EXIT_FAILURE;
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << "Exception received (as expected) "    << std::endl;
    std::cout << "Description : " << e.GetDescription() << std::endl;
    std::cout << "Location    : " << e.GetLocation()    << std::endl;
    std::cout << "Test for exception throwing... PASSED ! " << std::endl;
    }

  try
    {
    measure = multiMetric->GetValue( parameters );
    //metric->GetValueAndDerivative( parameters, measure, derivative );
    std::cout << "Value = " << measure << std::endl;
    std::cout << "If you are reading this message the Metric " << std::endl;
    std::cout << "is NOT managing exceptions correctly    " << std::endl;
    return EXIT_FAILURE;
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << "Exception received (as expected) "    << std::endl;
    std::cout << "Description : " << e.GetDescription() << std::endl;
    std::cout << "Location    : " << e.GetLocation()    << std::endl;
    std::cout << "Test for exception throwing... PASSED ! "  << std::endl;
    }

//Put the correct fixed images back in place...
  multiMetric->SetFixedMultiImage( fixedMultiImage );

 bool pass;
#define TEST_INITIALIZATION_ERROR( ComponentName, badComponent, goodComponent ) \
  multiMetric->Set##ComponentName( badComponent ); \
  try \
    { \
    pass = false; \
    multiMetric->Initialize(); \
    } \
  catch( itk::ExceptionObject& err ) \
    { \
    std::cout << "Caught expected ExceptionObject" << std::endl; \
    std::cout << err << std::endl; \
    pass = true; \
    } \
  multiMetric->Set##ComponentName( goodComponent ); \
  \
  if( !pass ) \
    { \
    std::cout << "Test failed." << std::endl; \
    return EXIT_FAILURE; \
    }

  MultiInterpolatorType badMultiInterpolator;
  FixedMultiImageRegionType badFixedMultiImageRegion;

  TEST_INITIALIZATION_ERROR( Transform, NULL, transform );
  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  TEST_INITIALIZATION_ERROR( FixedMultiImageRegion, badFixedMultiImageRegion, fixedMultiImageRegion );
  TEST_INITIALIZATION_ERROR( MovingImage, NULL, movingImage );
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );
  TEST_INITIALIZATION_ERROR( DerivativeDelta, -0.001, 0.001 );
  TEST_INITIALIZATION_ERROR( Radius, -1, 3 );

// The multi-metric should also fail when given only one fixed image, one interpolator or one region
  badFixedMultiImage.push_back( fixedMultiImage[0] );
  badMultiInterpolator.push_back( multiInterpolator[0] );
  badFixedMultiImageRegion.push_back( fixedMultiImageRegion[0] );

  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );
  TEST_INITIALIZATION_ERROR( FixedMultiImageRegion, badFixedMultiImageRegion, fixedMultiImageRegion );

// ...and when there is a null fixed image or a null interpolator
  badFixedMultiImage.push_back( 0 );
  badMultiInterpolator.push_back( 0 );
  TEST_INITIALIZATION_ERROR( FixedMultiImage, badFixedMultiImage, fixedMultiImage );
  TEST_INITIALIZATION_ERROR( MultiInterpolator, badMultiInterpolator, multiInterpolator );

  std::cout << "Test passed. " << std::endl;
  return EXIT_SUCCESS;

}
