#include "gtest/gtest.h"
#include "util/CircularBuffer.h"


#define INV_SQRT_ABS_ERR .01f

namespace {
  class CircularBufferTest : public ::testing::Test {
  protected:
  };


  TEST_F(CircularBufferTest, put_get) {
	  CircularBuffer<int> buffer(10);
	  buffer.put(1);
	  ASSERT_EQ(buffer.currentSize(), 1);
	  ASSERT_EQ(buffer.get(), 1);
	  ASSERT_EQ(buffer.currentSize(), 0);
  }

  TEST_F(CircularBufferTest, put_to_capacity) {
	  CircularBuffer<int> buffer(2);
	  buffer.put(1);
	  buffer.put(2);
	  ASSERT_EQ(buffer.currentSize(), 2);
	  ASSERT_EQ(buffer.get(), 1);
	  ASSERT_EQ(buffer.get(), 2);
	  ASSERT_EQ(buffer.currentSize(), 0);
  }

  TEST_F(CircularBufferTest, wrap_around) {
	  CircularBuffer<int> buffer(2);
	  buffer.put(1);
	  buffer.put(2);
	  ASSERT_EQ(buffer.currentSize(), 2);
	  buffer.put(3);
	  ASSERT_EQ(buffer.currentSize(), 2);
	  ASSERT_EQ(buffer.get(), 2);
	  ASSERT_EQ(buffer.get(), 3);
	  ASSERT_EQ(buffer.currentSize(), 0);
  }

  TEST_F(CircularBufferTest, alternate_put_gets) {
  	  CircularBuffer<int> buffer(2);
  	  buffer.put(1);
  	  ASSERT_EQ(buffer.get(), 1);
  	  buffer.put(2);
  	  ASSERT_EQ(buffer.get(), 2);
  	  buffer.put(3);
  	  ASSERT_EQ(buffer.get(), 3);
  }

  TEST_F(CircularBufferTest, get_last_inserted) {
  	  CircularBuffer<int> buffer(3);
  	  buffer.put(1);
  	  ASSERT_EQ(buffer.getLastInserted(), 1);
  	  buffer.put(2);
  	  buffer.put(3);
  	  buffer.put(4);
  	  ASSERT_EQ(buffer.getLastInserted(), 4);
  	  ASSERT_EQ(buffer.currentSize(), 3);
  }
}
