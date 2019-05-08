#include "utils_gtest.h"
#include "base/utils/converter.h"
#include "base/common/wolf.h"
#include "base/yaml/parser_yaml.hpp"
#include "base/utils/params_server.hpp"

using namespace std;
using namespace wolf;

std::string wolf_root = _WOLF_ROOT_DIR;

parserYAML parse(string _file, string _path_root)
{
  parserYAML parser = parserYAML(_file, _path_root);
  parser.parse();
  return parser;
}

TEST(ParamsServer, Default)
{
  auto parser = parse("/test/params1.yaml", wolf_root);
  auto params = parser.getParams();
  paramsServer server = paramsServer(params, parser.sensorsSerialization(), parser.processorsSerialization());
  EXPECT_EQ(server.getParam<double>("should_not_exist", "2.6"), 2.6);
  EXPECT_EQ(server.getParam<bool>("my_proc_test/voting_active", "true"), false);
  EXPECT_NE(server.getParam<unsigned int>("my_proc_test/time_tolerance", "23"), 23);
  EXPECT_THROW({ server.getParam<unsigned int>("test error"); }, std::runtime_error);
  EXPECT_NE(server.getParam<unsigned int>("my_proc_test/time_tolerance"), 23);
  EXPECT_EQ(server.getParam<bool>("my_proc_test/voting_active"), false);
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
