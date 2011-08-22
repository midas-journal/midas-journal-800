cmake_minimum_required(VERSION 2.6)

include(${CMAKE_SOURCE_DIR}/IJMacros.txt)

project(itkMultiImageRegistration)

#The following lines are required to use Dart
enable_testing()
include(Dart)

set(Required_Packages ITK)

FOREACH(Package ${Required_Packages})
  LOADPACKAGE(${Package})
ENDFOREACH(Package)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#Set any libraries that your project depends on.
#examples: ITKCommon, VTKRendering, etc
set(Libraries ITKAlgorithms ITKBasicFilters ITKCommon ITKIO)


#Add project's executables
add_executable(ImageCompare ImageCompare.cxx) 
target_link_libraries(ImageCompare ${Libraries})

add_executable(MultiImageSearch Examples/MultiImageSearch.cxx)
target_link_libraries(MultiImageSearch ${Libraries})

add_executable(MultiImageRegistration Examples/MultiImageRegistration.cxx)
target_link_libraries(MultiImageRegistration ${Libraries})

set(HeaderFiles
itkGradientDifferenceMultiImageToImageMetric.h
itkGradientDifferenceSingleImageToImageMetric.h
itkGradientDifferenceSingleImageToImageMetric.txx
itkMeanSquaresMultiImageToImageMetric.h
itkMultiImageRegistrationMacro.h
itkMultiImageToImageMetric.h
itkMultiImageToImageMetric.txx
itkMultiImageToImageRegistrationMethod.h
itkMultiImageToImageRegistrationMethod.txx
itkMultiResolutionMultiImageToImageRegistrationMethod.h
itkMultiResolutionMultiImageToImageRegistrationMethod.txx
itkNormalizedGradientCorrelationImageToImageMetric.h
itkNormalizedGradientCorrelationImageToImageMetric.txx
itkNormalizedGradientCorrelationMultiImageToImageMetric.h
itkPatchedRayCastInterpolateImageFunction.h
itkPatchedRayCastInterpolateImageFunction.txx
itkPatternIntensityImageToImageMetric.h
itkPatternIntensityImageToImageMetric.txx
itkPatternIntensityMultiImageToImageMetric.h
itkPatternIntensityMultiImageToImageMetric.txx
)
install(FILES ${HeaderFiles} DESTINATION include COMPONENT Development)

install(TARGETS MultiImageSearch DESTINATION bin COMPONENT Runtime)
install(TARGETS MultiImageRegistration DESTINATION bin COMPONENT Runtime)

if(${BUILD_TESTING} STREQUAL "ON")

#Add tests
set(DataDirectory ${CMAKE_CURRENT_SOURCE_DIR}/Data)

# Test for the PatchedRayCastInterpolateImageFunction class

add_executable(itkPatchedRayCastInterpolateImageFunctionTest Testing/itkPatchedRayCastInterpolateImageFunctionTest.cxx)
target_link_libraries(itkPatchedRayCastInterpolateImageFunctionTest ${Libraries})
add_test(RunPatchedRayCastInterpolateImageFunctionTest itkPatchedRayCastInterpolateImageFunctionTest)

# Tests for the MultiImageToImageMetric subclasses

add_executable(itkGradientDifferenceMultiImageToImageMetricTest Testing/itkGradientDifferenceMultiImageToImageMetricTest.cxx)
target_link_libraries(itkGradientDifferenceMultiImageToImageMetricTest ${Libraries})
add_test(RunGradientDifferenceMultiImageToImageMetricTest itkGradientDifferenceMultiImageToImageMetricTest)

add_executable(itkMeanSquaresMultiImageToImageMetricTest Testing/itkMeanSquaresMultiImageToImageMetricTest.cxx)
target_link_libraries(itkMeanSquaresMultiImageToImageMetricTest ${Libraries})
add_test(RunMeanSquaresMultiImageToImageMetricTest itkMeanSquaresMultiImageToImageMetricTest)

add_executable(itkNormalizedGradientCorrelationMultiImageToImageMetricTest Testing/itkNormalizedGradientCorrelationMultiImageToImageMetricTest.cxx)
target_link_libraries(itkNormalizedGradientCorrelationMultiImageToImageMetricTest ${Libraries})
add_test(RunNormalizedGradientCorrelationMultiImageToImageMetricTest itkNormalizedGradientCorrelationMultiImageToImageMetricTest)

add_executable(itkPatternIntensityMultiImageToImageMetricTest Testing/itkPatternIntensityMultiImageToImageMetricTest.cxx)
target_link_libraries(itkPatternIntensityMultiImageToImageMetricTest ${Libraries})
add_test(RunPatternIntensityMultiImageToImageMetricTest itkPatternIntensityMultiImageToImageMetricTest)


# Tests for the MultiImageToImageRegistrationMethod class

add_executable(itkMultiImageToImageRegistrationMethodTest0 Testing/itkMultiImageToImageRegistrationMethodTest0.cxx)
target_link_libraries(itkMultiImageToImageRegistrationMethodTest0 ${Libraries})
add_test(RunMultiImageToImageRegistrationMethodTest0 itkMultiImageToImageRegistrationMethodTest0)

add_executable(itkMultiImageToImageRegistrationMethodTest1 Testing/itkMultiImageToImageRegistrationMethodTest1.cxx)
target_link_libraries(itkMultiImageToImageRegistrationMethodTest1 ${Libraries})
add_test(RunMultiImageToImageRegistrationMethodTest1 itkMultiImageToImageRegistrationMethodTest1 ${DataDirectory})


# Tests for the MultiResolutionMultiImageToImageRegistrationMethod class

add_executable(itkMultiResolutionMultiImageToImageRegistrationMethodTest0 Testing/itkMultiResolutionMultiImageToImageRegistrationMethodTest0.cxx)
target_link_libraries(itkMultiResolutionMultiImageToImageRegistrationMethodTest0 ${Libraries})
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest0 itkMultiResolutionMultiImageToImageRegistrationMethodTest0)

add_executable(itkMultiResolutionMultiImageToImageRegistrationMethodTest1 Testing/itkMultiResolutionMultiImageToImageRegistrationMethodTest1.cxx)
target_link_libraries(itkMultiResolutionMultiImageToImageRegistrationMethodTest1 ${Libraries})
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest1 itkMultiResolutionMultiImageToImageRegistrationMethodTest1 ${DataDirectory})
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest1-mov ImageCompare ${DataDirectory}/moving.level0.mha   ${DataDirectory}/moving.level0.baseline.mha)
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest1-LAT ImageCompare ${DataDirectory}/fixedLAT.level0.mha ${DataDirectory}/fixedLAT.level0.baseline.mha)
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest1-AP  ImageCompare ${DataDirectory}/fixedAP.level0.mha  ${DataDirectory}/fixedAP.level0.baseline.mha)

add_executable(itkMultiResolutionMultiImageToImageRegistrationMethodTest2 Testing/itkMultiResolutionMultiImageToImageRegistrationMethodTest2.cxx)
target_link_libraries(itkMultiResolutionMultiImageToImageRegistrationMethodTest2 ${Libraries})
add_test(RunMultiResolutionMultiImageToImageRegistrationMethodTest2 itkMultiResolutionMultiImageToImageRegistrationMethodTest2 ${DataDirectory})

endif(${BUILD_TESTING} STREQUAL "ON")