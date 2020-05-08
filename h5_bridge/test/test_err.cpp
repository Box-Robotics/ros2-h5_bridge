#include <h5_bridge/err.hpp>
#include <vector>
#include <gtest/gtest.h>

TEST(err, Exceptions)
{
  std::vector<int> errs =
    {
      h5_bridge::OK
    };

  for (auto err : errs)
    {
      try
        {
          throw(h5_bridge::error_t(err));
        }
      catch (const h5_bridge::error_t& ex)
        {
          EXPECT_EQ(err, ex.code());
          EXPECT_STREQ(h5_bridge::strerror(err), ex.what());
        }
    }
}
