#include "utils_gtest.h"
#include "base/utils/converter.h"
#include "base/common/wolf.h"
#include "base/yaml/parser_yaml.hpp"

using namespace std;
using namespace wolf;

std::string wolf_root = _WOLF_ROOT_DIR;

parserYAML parse(string _file, string _path_root)
{
  parserYAML parser = parserYAML(_file, _path_root);
  parser.parse();
  return parser;
}

TEST(ParserYAML, RegularParse)
{
  auto parser = parse("/test/params1.yaml", wolf_root);
  auto params = parser.getParams();
  // for(auto it : params)
  //   cout << it.first << " %% " << it.second << endl;
  EXPECT_EQ(params["odom/intrinsic/k_rot_to_rot"], "0.1");
  EXPECT_EQ(params["processor1/sensor name"], "odom");
}
TEST(ParserYAML, ParseMap)
{
  auto parser = parse("/test/params2.yaml", wolf_root);
  auto params = parser.getParams();
  EXPECT_EQ(params["processor1/mymap"], "[{k1:v1},{k2:v2},{k3:[v3,v4,v5]}]");
}
TEST(ParserYAML, JumpFile)
{
  auto parser = parse("/test/params3.yaml", wolf_root);
  auto params = parser.getParams();
  EXPECT_EQ(params["my_proc_test/max_buff_length"], "100");
  EXPECT_EQ(params["my_proc_test/jump/voting_active"], "false");
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
