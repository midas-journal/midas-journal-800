
#ifdef _MSC_VER
#pragma warning ( disable : 4786 )
#endif

#include <itkCommand.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkMultiResolutionPyramidImageFilter.h>
#include <itkPowellOptimizer.h>
#include <itkTranslationTransform.h>

#include "itkNormalizedGradientCorrelationMultiImageToImageMetric.h"
#include "itkMultiResolutionMultiImageToImageRegistrationMethod.h"
#include "itkPatchedRayCastInterpolateImageFunction.h"

#include <vector>
#include <sstream>
#include <string>


/**
 * This program tests the MultiResolutionMultiImageRegistrationMethod class.
 * A moving image of lumbar vertebrae is registered to two projections taken
 * on the Antero-Posterior and Lateral orientations. Multiple registrations
 * are started with different starting points to test the registration
 * method's capture range.
 *
 */

//----------------------------------------------------------------------------
// Optimizer observer class
//----------------------------------------------------------------------------
class OptimizerObserver : public itk::Command
{
public:
typedef  OptimizerObserver     Self;
typedef  itk::Command             Superclass;
typedef  itk::SmartPointer<Self>  Pointer;
itkNewMacro( Self );

protected:
OptimizerObserver() {};

public:
typedef itk::PowellOptimizer  OptimizerType;
typedef OptimizerType*        OptimizerPointer;

void Execute(itk::Object *caller, const itk::EventObject & event)
{
  OptimizerPointer optimizer =
    dynamic_cast<OptimizerPointer>( caller );
  if( itk::IterationEvent().CheckEvent( & event ) )
    {
    std::cout << "Iteration " << optimizer->GetCurrentIteration()
      << "/" << optimizer->GetMaximumIteration() << " Position: " <<
      optimizer->GetCurrentPosition() << " Value: " <<
      optimizer->GetCurrentCost() << std::endl;
    }
  else if( itk::StartEvent().CheckEvent( & event ) )
    {
    std::cout << "Optimization started ..." << std::endl;
    }
  else if( itk::EndEvent().CheckEvent( & event ) )
    {
    std::cout << "Optimization ended ..." << std::endl;
    }
}


void Execute(const itk::Object* caller, const itk::EventObject & event)
{}

};


//----------------------------------------------------------------------------
// Main function of test
//----------------------------------------------------------------------------
int main(int argc, char* argv[] )
{
  if( argc < 2 )
    {
    std::cerr << "Usage: " << argv[0] << " [data directory]" << std::endl;
    return EXIT_FAILURE;
    }
  const std::string dataDirectory = argv[1];

//  const unsigned int FImgTotal = 2;

  const unsigned int Dimensions = 3;

  typedef itk::Image<short,Dimensions>   FixedImageType;
  typedef itk::Image<short,Dimensions>  MovingImageType;
  typedef itk::MultiResolutionMultiImageToImageRegistrationMethod<
    FixedImageType,MovingImageType >    RegistrationType;

  RegistrationType::Pointer registration = RegistrationType::New();


//----------------------------------------------------------------------------
// Load the moving image
//----------------------------------------------------------------------------
  typedef itk::ImageFileReader<MovingImageType> MovingImageReaderType;

  MovingImageReaderType::Pointer movingReader = MovingImageReaderType::New();
  std::string movingPath = dataDirectory;
  movingPath.append( "/moving.thr.mha" );
  movingReader->SetFileName( movingPath.c_str() );
  try
    {
    movingReader->Update();
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }

  registration->SetMovingImage( movingReader->GetOutput() );


//----------------------------------------------------------------------------
// Load the fixed images
//----------------------------------------------------------------------------
  typedef std::vector<FixedImageType::ConstPointer>  FixedMultiImageType;
  typedef itk::ImageFileReader<FixedImageType>       FixedImageReaderType;
  typedef std::vector<FixedImageReaderType::Pointer> FixedMultiImageReaderType;

  FixedMultiImageReaderType fixedMultiReader;

  fixedMultiReader.push_back( FixedImageReaderType::New() );
  std::string fixedAPPath = dataDirectory;
  fixedAPPath.append( "/fixedAP.mha" );
  fixedMultiReader[0]->SetFileName( fixedAPPath.c_str() );
  try
    {
    fixedMultiReader[0]->Update();
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }
  registration->AddFixedImage( fixedMultiReader[0]->GetOutput() );

  FixedImageType::RegionType fixedRegionAP;
  fixedRegionAP.SetIndex( 0, 150 );
  fixedRegionAP.SetIndex( 1, 200 );
  fixedRegionAP.SetIndex( 2, 0 );
  fixedRegionAP.SetSize( 0, 200 );
  fixedRegionAP.SetSize( 1, 120 );
  fixedRegionAP.SetSize( 2, 1 );
  registration->AddFixedImageRegion( fixedRegionAP );

  fixedMultiReader.push_back( FixedImageReaderType::New() );
  std::string fixedLATPath = dataDirectory;
  fixedLATPath.append( "/fixedLAT.mha" );
  fixedMultiReader[1]->SetFileName( fixedLATPath.c_str() );
  try
    {
    fixedMultiReader[1]->Update();
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }
  registration->AddFixedImage( fixedMultiReader[1]->GetOutput() );

  FixedImageType::RegionType fixedRegionLAT;
  fixedRegionLAT.SetIndex( 0, 150 );
  fixedRegionLAT.SetIndex( 1, 200 );
  fixedRegionLAT.SetIndex( 2, 0 );
  fixedRegionLAT.SetSize( 0, 200 );
  fixedRegionLAT.SetSize( 1, 120 );
  fixedRegionLAT.SetSize( 2, 1 );
  registration->AddFixedImageRegion( fixedRegionLAT );


//----------------------------------------------------------------------------
// Create the transform
//----------------------------------------------------------------------------
  typedef itk::TranslationTransform< double, Dimensions > TransformType;

  TransformType::Pointer transform = TransformType::New();
  registration->SetTransform( transform );


//----------------------------------------------------------------------------
// Create the multi metric
//----------------------------------------------------------------------------
  typedef itk::NormalizedGradientCorrelationMultiImageToImageMetric<
    FixedImageType, MovingImageType> MultiMetricType;

  MultiMetricType::Pointer multiMetric = MultiMetricType::New();
  registration->SetMultiMetric( multiMetric );


//----------------------------------------------------------------------------
// Create the optimizer
//----------------------------------------------------------------------------
  typedef itk::PowellOptimizer OptimizerType;

  OptimizerType::Pointer optimizer = OptimizerType::New();

  OptimizerType::ScalesType scales( transform->GetNumberOfParameters() );
  scales.Fill( itk::NumericTraits<OptimizerType::ScalesType::ValueType>::One );
  optimizer->SetScales( scales );

  optimizer->SetMaximumIteration( 100 );
  optimizer->SetMaximumLineIteration( 10 );
  optimizer->SetStepLength( 1.0 );
  optimizer->SetStepTolerance( 1e-2 );
  optimizer->SetValueTolerance( 1e-3 );
  optimizer->SetMaximize( true );
  optimizer->SetCatchGetValueException( true );
  optimizer->SetMetricWorstPossibleValue(
    -itk::NumericTraits<MultiMetricType::MeasureType>::infinity() );

  OptimizerObserver::Pointer observer = OptimizerObserver::New();
  optimizer->AddObserver( itk::StartEvent(), observer );
  optimizer->AddObserver( itk::IterationEvent(), observer );
  optimizer->AddObserver( itk::EndEvent(), observer );

  registration->SetOptimizer( optimizer );


//----------------------------------------------------------------------------
// Create the interpolators
//----------------------------------------------------------------------------
  typedef itk::PatchedRayCastInterpolateImageFunction<
    MovingImageType, double> InterpolatorType;
  typedef InterpolatorType::InputPointType FocalPointType;

  InterpolatorType::Pointer interpolatorAP = InterpolatorType::New();
  FocalPointType focalPointAP;
  focalPointAP[0] = 0.0;
  focalPointAP[1] = 1000.0;
  focalPointAP[2] = 0.0;
  interpolatorAP->SetFocalPoint( focalPointAP );
  interpolatorAP->SetTransform( transform );
  registration->AddInterpolator( interpolatorAP );

  InterpolatorType::Pointer interpolatorLAT = InterpolatorType::New();
  FocalPointType focalPointLAT;
  focalPointLAT[0] = -1000.0;
  focalPointLAT[1] = 0.0;
  focalPointLAT[2] = 0.0;
  interpolatorLAT->SetFocalPoint( focalPointLAT );
  interpolatorLAT->SetTransform( transform );
  registration->AddInterpolator( interpolatorLAT );


//----------------------------------------------------------------------------
// Create the pyramid filters
//----------------------------------------------------------------------------
  typedef itk::MultiResolutionPyramidImageFilter<
                                    MovingImageType,
                                    MovingImageType  > MovingImagePyramidType;
  typedef itk::MultiResolutionPyramidImageFilter<
                                    FixedImageType,
                                    FixedImageType  >  FixedImagePyramidType;

  MovingImagePyramidType::Pointer movingPyramidFilter =
    MovingImagePyramidType::New();
  registration->SetMovingImagePyramid( movingPyramidFilter );

  FixedImagePyramidType::Pointer fixedPyramidFilterAP =
    FixedImagePyramidType::New();
  registration->AddFixedImagePyramid( fixedPyramidFilterAP );

  FixedImagePyramidType::Pointer fixedPyramidFilterLAT =
    FixedImagePyramidType::New();
  registration->AddFixedImagePyramid( fixedPyramidFilterLAT );


//----------------------------------------------------------------------------
// Set the moving and fixed images' schedules
//----------------------------------------------------------------------------
  const unsigned int ResolutionLevels = 2;

  RegistrationType::ScheduleType fixedSchedule( ResolutionLevels,Dimensions );
  fixedSchedule[0][0] = 2;
  fixedSchedule[0][1] = 2;
  fixedSchedule[0][2] = 1;
  fixedSchedule[1][0] = 1;
  fixedSchedule[1][1] = 1;
  fixedSchedule[1][2] = 1;

  RegistrationType::ScheduleType movingSchedule( ResolutionLevels,Dimensions);
  movingSchedule[0][0] = 2;
  movingSchedule[0][1] = 2;
  movingSchedule[0][2] = 2;
  movingSchedule[1][0] = 1;
  movingSchedule[1][1] = 1;
  movingSchedule[1][2] = 1;

  registration->SetSchedules( fixedSchedule, movingSchedule );


//----------------------------------------------------------------------------
// Start the registrations
//----------------------------------------------------------------------------
  const unsigned int RegTotal = 4;
  double startX[RegTotal] = { 0.0,  0.3,  0.4, -1.0 };
  double startY[RegTotal] = { 0.0, -0.1, -0.2, -0.1 };
  double startZ[RegTotal] = { 0.0,  0.0,  0.4,  0.0 };

  unsigned int wrongRegistrations = 0;

  typedef RegistrationType::ParametersType ParametersType;
  ParametersType initialParameters( transform->GetNumberOfParameters() );

  for( unsigned int r=0; r<RegTotal; r++ )
    {
    std::cout << "Testing registration " << r << "/" << RegTotal << std::endl;

    initialParameters[0] = startX[r];
    initialParameters[1] = startY[r];
    initialParameters[2] = startZ[r];
    registration->SetInitialTransformParameters( initialParameters );

    std::cout << "Initial parameters: " << initialParameters << std::endl;
    try
      {
      registration->Update();
      }
    catch( itk::ExceptionObject & e )
      {
      std::cout << e.GetDescription() << std::endl;
      return EXIT_FAILURE;
      }

    ParametersType lastParameters = registration->GetLastTransformParameters();
    std::cout << "Last parameters: " << lastParameters << std::endl;
    //std::cout << "Stop condition: " <<
    //  optimizer->GetStopConditionDescription() << std::endl;

    if( vcl_sqrt( lastParameters[0]*lastParameters[0] +
                  lastParameters[1]*lastParameters[1] +
                  lastParameters[2]*lastParameters[2] ) > 2.0 )
      {
      std::cout << "Registration " << r << " converged to an wrong solution"
        << std::endl;
      wrongRegistrations++;
      return EXIT_FAILURE;
      }
    }

  if( wrongRegistrations > 0 )
    {
    std::cout << "There were " << wrongRegistrations <<
      " wrong registrations" << std::endl;
    std::cout << "Test failed" << std::endl;
    return EXIT_FAILURE;
    }



//----------------------------------------------------------------------------
// End of test
//----------------------------------------------------------------------------
  std::cout << "Test passed." << std::endl;
  return EXIT_SUCCESS;


}
