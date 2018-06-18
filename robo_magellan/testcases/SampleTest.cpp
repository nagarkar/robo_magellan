#include "gtest/gtest.h"
#include "FileUnderTest.h"


namespace {
  class BarTest : public ::testing::Test {
  protected:
    Foo foo;
  };
  TEST_F(BarTest, Foo) {
    ASSERT_TRUE(foo.foo());
  }
}
