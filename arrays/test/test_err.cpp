#include <arrays/err.hpp>
#include <vector>
#include <gtest/gtest.h>

TEST(err, Exceptions)
{
  std::vector<int> errs =
    {
      arrays::OK
    };

  for (auto err : errs)
    {
      try
        {
          throw(arrays::error_t(err));
        }
      catch (const arrays::error_t& ex)
        {
          EXPECT_EQ(err, ex.code());
          EXPECT_STREQ(arrays::strerror(err), ex.what());
        }
    }
}
