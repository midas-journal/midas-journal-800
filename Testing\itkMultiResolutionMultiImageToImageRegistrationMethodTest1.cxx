
#ifdef _MSC_VER
#pragma warning ( disable : 4786 )
#endif

#include <itkCommand.h>
#include <itkExhaustiveOptimizer.h>
#include <itkImage.h>
#include <itkImageFileReader.h>
#include <itkImageFileWriter.h>
#include <itkMultiResolutionPyramidImageFilter.h>
#include <itkTranslationTransform.h>

#include "itkNormalizedGradientCorrelationMultiImageToImageMetric.h"
#include "itkMultiResolutionMultiImageToImageRegistrationMethod.h"
#include "itkPatchedRayCastInterpolateImageFunction.h"

#include <vector>
#include <sstream>
#include <string>

/**
 * This program tests that the
 * MultiResolutionMultiImageRegistrationMethod's filters generate the correct
 * downsampled versions of the fixed and moving images on each resolution
 * level.
 *
 */


//----------------------------------------------------------------------------
// Registration observer class
//----------------------------------------------------------------------------
template <typename TFixedImage, typename TMovingImage>
class RegistrationObserver : public itk::Command
{
public:
typedef  RegistrationObserver     Self;
typedef  itk::Command             Superclass;
typedef  itk::SmartPointer<Self>  Pointer;
itkNewMacro( Self );

protected:
  RegistrationObserver() {};

private:
  std::string m_DataDirectory;

public:
typedef itk::MultiResolutionMultiImageToImageRegistrationMethod
  <TFixedImage,TMovingImage>    RegistrationType;
typedef RegistrationType*       RegistrationPointer;
typedef const RegistrationType* RegistrationConstPointer;

typedef typename RegistrationType::MovingImageType          MovingImageType;
typedef typename RegistrationType::MovingImageConstPointer  MovingImageConstPointer;
typedef typename RegistrationType::FixedImageType           FixedImageType;
typedef typename RegistrationType::FixedImageConstPointer   FixedImageConstPointer;
typedef itk::ImageFileReader<MovingImageType>               MovingImageReaderType;
typedef itk::ImageFileReader<FixedImageType>                FixedImageReaderType;
typedef itk::ImageFileWriter<MovingImageType>               MovingImageWriterType;
typedef typename MovingImageWriterType::Pointer             MovingImageWriterPointer;
typedef itk::ImageFileWriter<FixedImageType>                FixedImageWriterType;
typedef typename FixedImageWriterType::Pointer              FixedImageWriterPointer;

itkSetMacro(DataDirectory,std::string);


void Execute(itk::Object *caller, const itk::EventObject & event)
{
  RegistrationPointer registration =
    dynamic_cast<RegistrationPointer>( caller );
  if( itk::IterationEvent().CheckEvent( & event ) )
    {
    const unsigned long level = registration->GetCurrentLevel();

// Check the downsampled version of the moving image
    MovingImageConstPointer movingImage =
      registration->GetMovingImagePyramid()->GetOutput( level );

    std::stringstream movingStream;
    movingStream << m_DataDirectory << "/moving.level" << level << ".mha";
    MovingImageWriterPointer movingWriter =
      MovingImageWriterType::New();
    movingWriter->SetFileName( movingStream.str().c_str() );
    movingWriter->SetInput( movingImage );
    try
      {
      movingWriter->Update();
      }
    catch( itk::ExceptionObject & e )
      {
      throw;
      }


// Check the downsampled version of the fixed AP image
    FixedImageConstPointer fixedImageAP =
      registration->GetFixedMultiImagePyramid()[0]->GetOutput( level );
    std::stringstream fixedStreamAP;
    fixedStreamAP << m_DataDirectory << "/fixedAP.level" << level << ".mha";
    FixedImageWriterPointer fixedWriterAP = FixedImageWriterType::New();
    fixedWriterAP->SetFileName( fixedStreamAP.str().c_str() );
    fixedWriterAP->SetInput( fixedImageAP );
    try
      {
      fixedWriterAP->Update();
      }
    catch( itk::ExceptionObject & e )
      {
      throw;
      }

// Check the downsampled version of the fixed LAT image
    FixedImageConstPointer fixedImageLAT =
      registration->GetFixedMultiImagePyramid()[1]->GetOutput( level );
    std::stringstream fileNameStreamLAT;
    fileNameStreamLAT << m_DataDirectory << "/fixedLAT.level" << level << ".mha";
    FixedImageWriterPointer fixedWriterLAT = FixedImageWriterType::New();
    fixedWriterLAT->SetFileName( fileNameStreamLAT.str().c_str() );
    fixedWriterLAT->SetInput( fixedImageLAT );
    try
      {
      fixedWriterLAT->Update();
      }
    catch( itk::ExceptionObject & e )
      {
      throw;
      }

    } // if( itk::IterationEvent().CheckEvent( & event ) )
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

//  bool pass;

  const unsigned int Dimensions = 3;

  typedef itk::Image<short,Dimensions>   FixedImageType;
  typedef itk::Image<short,Dimensions>  MovingImageType;
  typedef itk::MultiResolutionMultiImageToImageRegistrationMethod<
    FixedImageType,MovingImageType >    RegistrationType;


  RegistrationType::Pointer registration = RegistrationType::New();

  typedef RegistrationObserver<FixedImageType,MovingImageType> ObserverType;
  ObserverType::Pointer observer = ObserverType::New();
  observer->SetDataDirectory( dataDirectory );
  registration->AddObserver( itk::IterationEvent(), observer );


//----------------------------------------------------------------------------
// Load the moving image
//----------------------------------------------------------------------------
  typedef itk::ImageFileReader<MovingImageType> MovingImageReaderType;

  MovingImageReaderType::Pointer movingReader = MovingImageReaderType::New();
  std::string  movingPath =  dataDirectory;
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
  typedef itk::ExhaustiveOptimizer OptimizerType;
  typedef OptimizerType::StepsType StepsType;

  OptimizerType::Pointer optimizer = OptimizerType::New();

  StepsType steps( transform->GetNumberOfParameters() );
  steps.Fill( itk::NumericTraits<StepsType::ValueType>::Zero );
  optimizer->SetNumberOfSteps( steps );

  optimizer->SetStepLength( 0.0 );

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
// Start the registration
//----------------------------------------------------------------------------
  typedef RegistrationType::ParametersType ParametersType;
  ParametersType initialParameters( transform->GetNumberOfParameters() );
  initialParameters.Fill(itk::NumericTraits<ParametersType::ValueType>::Zero);
  registration->SetInitialTransformParameters( initialParameters );

  try
    {
    registration->Update();
    }
  catch( itk::ExceptionObject & e )
    {
    std::cout << e.GetDescription() << std::endl;
    return EXIT_FAILURE;
    }


//----------------------------------------------------------------------------
// End of test
//----------------------------------------------------------------------------
  std::cout << "Test passed." << std::endl;
  return EXIT_SUCCESS;


}
