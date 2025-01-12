#include <bmb_math/Matrix.h>
#include <gtest/gtest.h>
#include <cstdint>
#include <string>

TEST(TestMatrix, testMatrix) {
  Matrix<uint8_t, 2, 3> mat{};
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 3; j++) ASSERT_EQ(mat[i][j], 0);

  Matrix<uint8_t, 3, 4> mat2 = Matrix<uint8_t, 3, 4>::identity();
  Matrix<uint8_t, 2, 4> product = mat * mat2;
  Matrix<uint8_t, 2, 4> z = Matrix<uint8_t, 2, 4>::zeros();
  for (size_t i = 0; i < 2; i++)
    for (size_t j = 0; j < 4; j++) ASSERT_EQ(product[i][j], z[i][j]);

  Matrix<int, 1, 2> d_mat = Matrix<int, 1, 2>::identity();
  Matrix<int, 1, 2> s_mat =
      d_mat.applyFunc<int>([](const int& num) { return num * num; });
  ASSERT_EQ(s_mat[0][0], 1);
  ASSERT_EQ(s_mat[0][1], 0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
