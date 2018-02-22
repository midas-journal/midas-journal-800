itk_module_test()

set(BertelstenRegistrationTests
  itkMinimalStandardRandomVariateGeneratorTest.cxx
  itkNormalDistributionImageSourceTest.cxx
  )

CreateTestDriver(BertelstenRegistration "${BertelstenRegistration-Test_LIBRARIES}" "${BertelstenRegistrationTests}")

itk_add_test(NAME itkMinimalStandardRandomVariateGeneratorTest
  COMMAND BertelstenRegistrationTestDriver itkMinimalStandardRandomVariateGeneratorTest
  )

itk_add_test(NAME itkNormalDistributionImageSourceTest
  COMMAND BertelstenRegistrationTestDriver --without-threads
  --compare
    ${ITK_TEST_OUTPUT_DIR}/itkNormalDistributionImageSourceTestOutput.mha
    DATA{Baseline/itkNormalDistributionImageSourceTestOutput.mha}
  itkNormalDistributionImageSourceTest
    ${ITK_TEST_OUTPUT_DIR}/itkNormalDistributionImageSourceTestOutput.mha
  )
