SET(TEST_SOURCES test_sli_slam)

FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cc)
    TARGET_LINK_LIBRARIES(${test_src} ${THIRD_PARTY_LIBS} sli_slam)
    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)