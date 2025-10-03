// Copyright (c) FIRST and other WPILib contributors.
// Turbo Torque 7492

#include <gtest/gtest.h>

TEST(DummyTest, AlwaysPasses) {
  EXPECT_EQ(1, 1);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
