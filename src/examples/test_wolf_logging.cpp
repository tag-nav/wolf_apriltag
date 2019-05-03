/**
 * \file test_wolf_logging.cpp
 *
 * Created on: Oct 28, 2016
 * \author: Jeremie Deray
 */

#include "core/common/wolf.h"
#include "core/utils/logging.h"

int main(int, char*[])
{
  WOLF_INFO("test info ", 5, " ", 0.123);

  WOLF_WARN("test warn ", 5, " ", 0.123);

  WOLF_ERROR("test error ", 5, " ", 0.123);

  WOLF_TRACE("test trace ", 5, " ", 0.123);

  WOLF_DEBUG("test debug ", 5, " ", 0.123);

  return 0;
}
