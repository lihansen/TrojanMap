#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"



TEST(TrojanMapStudentTest, Test_Get_functions) {
  TrojanMap tm = TrojanMap();

  // GetLat
  EXPECT_EQ(tm.GetLat("9591449441"), 34.0222564);
  EXPECT_EQ(tm.GetLat("9591449456"), 34.0243490);
  EXPECT_EQ(tm.GetLat("000"), -1);

  // GetLon
  EXPECT_EQ(tm.GetLon("9591449441"), -118.2801326);
  EXPECT_EQ(tm.GetLon("9591449456"), -118.2844736);
  EXPECT_EQ(tm.GetLon("000"), -1);

  // GetName
  EXPECT_EQ(tm.GetName("9591449441"), "Chase");
  EXPECT_EQ(tm.GetName("9591449456"), "Lululemon");
  EXPECT_EQ(tm.GetName("000"), "NULL");

  // GetNeighborIDs
  std::vector<std::string> empty_vec = {};
  std::vector<std::string> vec_9591449441 = {"9559739232"};
  std::vector<std::string> vec_9591449456 = {"9591449455"};
  EXPECT_EQ(tm.GetNeighborIDs("9591449441"), vec_9591449441);
  EXPECT_EQ(tm.GetNeighborIDs("9591449456"), vec_9591449456);
  EXPECT_EQ(tm.GetNeighborIDs("000"), empty_vec);

  // GetID
  EXPECT_EQ(tm.GetID("Chase"), "9591449441");
  EXPECT_EQ(tm.GetID("Lululemon"), "9591449456");
  EXPECT_EQ(tm.GetID("empty"), "");

  // GetPosition
  EXPECT_EQ(tm.GetPosition("Chase"), std::make_pair(34.0222564, -118.2801326));
  EXPECT_EQ(tm.GetPosition("Lululemon"), std::make_pair(34.0243490, -118.2844736));
  EXPECT_EQ(tm.GetPosition("empty"), std::make_pair(-1.0, -1.0));

  // // Autocomplete
  std::vector <std::string> res1 =  { "Chinese Street Food", "Chase", "Chucks Chicken & Waffles", 
                "Cheebos Burger", "Chevron", "Chase Plaza Heliport", "Chipotle", "Chevron 2", 
                "Church of Christ", "Chevron 1", "Chick-fil-A" };
  std::vector <std::string> res2 =  {"City Tacos", "City of Angels Independent Studies School"};
  EXPECT_EQ(tm.Autocomplete("Ch"), res1);
  EXPECT_EQ(tm.Autocomplete("city"), res2);
  EXPECT_EQ(tm.Autocomplete("null"), empty_vec);


  // CalculateEditDistance
  EXPECT_EQ(tm.CalculateEditDistance("a", "a"), 0);
  EXPECT_EQ(tm.CalculateEditDistance("a", "ab"), 1);
  EXPECT_EQ(tm.CalculateEditDistance("abc", "acc"), 1);
  EXPECT_EQ(tm.CalculateEditDistance("aCC", "aC"), 1);

  // FindClosestName
  EXPECT_EQ(tm.FindClosestName("Rolphs"), "Ralphs");
  EXPECT_EQ(tm.FindClosestName("Chose"), "Chase");
  EXPECT_EQ(tm.FindClosestName("Chase"), "Chase");

  
  std::vector <std::string> path = {
    "2578244375","4380040154","4380040153","4380040152",
    "4380040148","6818427920","6818427919","6818427918",
    "6818427892","6818427898","6818427917","6818427916",
    "7232024780","6813416145","6813416154","6813416153",
    "2613117902","6818390178","2613117882","6818390165",
    "2613117885","6807374562","6818390172","6818390171",
    "6818390170","2613117861","6817230316","3642819026",
    "6817230310","7811699597","5565967545","123318572",
    "6813405206","6813405205","6813405204","6813405203",
    "6813405202","6813405201","6818390145","6818390144",
    "6813379475","6813379474","6813379385","6045054380",
    "6813360984","6813360983","6047234445","5237400231",
    "5237377002","6813360938","6813379467","6813379439",
    "544671962","2783295153","6813379408","9591449452",
    "6807200381","6813379456","6818390146","3872400990",
    "3402917921","3402917919","9559739236","6045067409",
    "3398574883","9559739232","9591449441"
  };

  // CalculateShortestPath_Dijkstra
  EXPECT_EQ(tm.CalculateShortestPath_Dijkstra("Ralphs", "Chase"), path);

  // CalculateShortestPath_Bellman_Ford
  EXPECT_EQ(tm.CalculateShortestPath_Bellman_Ford("Ralphs", "Chase"), path);

  // CycleDetection
  std::vector <double> square1 = {-118.299, -118.264, 34.032, 34.011};
  std::vector <double> square2 = {-118.290, -118.289, 34.030, 34.020};
  std::vector <std::string > subgraph = {};
  EXPECT_EQ(tm.CycleDetection(subgraph, square1), true);
  subgraph.clear();
  EXPECT_EQ(tm.CycleDetection(subgraph, square2), false);

  // DeliveringTrojan
  std::vector<std::string> location_names = {"Chick-fil-A", "KFC", "Ralphs"};
  std::vector<std::vector <std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"Chick-fil-A", "KFC"}};
  std::vector<std::string> result = { "Ralphs", "Chick-fil-A", "KFC" };
  EXPECT_EQ(tm.DeliveringTrojan(location_names, dependencies), result);

}

// Test cycle detection function - pass
TEST(TrojanMapTest, CycleDetection) {
  TrojanMap m;
  
  // Test case 1
  std::vector<double> square1 = {-118.299, -118.264, 34.032, 34.011};
  auto sub1 = m.GetSubgraph(square1);
  bool result1 = m.CycleDetection(sub1, square1);
  EXPECT_EQ(result1, true);

  // Test case 2
  std::vector<double> square2 = {-118.290, -118.289, 34.030, 34.020};
  auto sub2 = m.GetSubgraph(square2);
  bool result2 = m.CycleDetection(sub2, square2);
  EXPECT_EQ(result2, false);
}

// Test topological sort function pass
TEST(TrojanMapTest, TopologicalSort) {
  TrojanMap m;
  
  std::vector<std::string> location_names = {"Ralphs", "Chick-fil-A", "KFC"};
  std::vector<std::vector<std::string>> dependencies = {{"Ralphs","KFC"}, {"Ralphs","Chick-fil-A"}, {"KFC","Chick-fil-A"}};
  auto result = m.DeliveringTrojan(location_names, dependencies);
  std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  EXPECT_EQ(result, gt);
}


// Test CalculateShortestPath_Dijkstra function - pass in 155.8 sec
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
      "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
      "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
      "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
      "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
      "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
      "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
      "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
      "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
      "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
      "6814916515","6820935910","4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Dijkstra("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}



// Test CalculateShortestPath_Bellman_Ford function
TEST(TrojanMapTest, CalculateShortestPath_Bellman_Ford) {
  TrojanMap m;
  
  // Test from Ralphs to Chick-fil-A
  auto path = m.CalculateShortestPath_Bellman_Ford("Ralphs", "Chick-fil-A");
  std::vector<std::string> gt{
      "2578244375","4380040154","4380040153","4380040152","4380040148","6818427920","6818427919",
      "6818427918","6818427892","6818427898","6818427917","6818427916","7232024780","6813416145",
      "6813416154","6813416153","6813416152","6813416151","6813416155","6808069740","6816193785",
      "6816193786","123152294","4015203136","4015203134","4015203133","21098539","6389467809",
      "4015203132","3195897587","4015203129","4015203127","6352865690","6813379589","6813379483",
      "3402887081","6814958394","3402887080","602606656","4872897515","4399697589","6814958391",
      "123209598","6787673296","122728406","6807762271","4399697304","4399697302","5231967015",
      "1862347583","3233702827","4540763379","6819179753","6820935900","6820935901","6813379556",
      "6820935898","1781230450","1781230449","4015405542","4015405543","1837212104","1837212107",
      "2753199985","6820935907","1837212100","4015372458","6813411588","1837212101","6814916516",
      "6814916515","6820935910","4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to Chick-fil-A
  path = m.CalculateShortestPath_Bellman_Ford("Chick-fil-A", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}


// Phase 3

// Test TSP function
TEST(TrojanMapTest, TSP1) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Brute_force(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;

  // std::pair<double, std::vector<std::vector<std::string>>> test = {0.1, {{"111", "222"}, {"111"}}};
  EXPECT_EQ(result.second[result.second.size()-1], gt);
  // EXPECT_EQ(gt.size(), result.second.size());
  // EXPECT_EQ(flag, true);

  // auto item = std::find(result.second.begin(), result.second.end(), gt);
  // EXPECT_EQ((item == result.second.end()), false);
}


// TravellingTrojan_Backtracking pass
TEST(TrojanMapTest, TSP2) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_Backtracking(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;
  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  
  EXPECT_EQ(flag, true);
  // EXPECT_EQ(gt.size(), result.second.size());
}



// travellingTrojan_2opt pass
TEST(TrojanMapTest, TSP3) {
  TrojanMap m;
  
  std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: "  << result.first << "miles" << std::endl; // Print the result path lengths
  std::vector<std::string> gt{"6819019976","1873055993","8566227656","122702233","8566227783","6816180153","7771782316","6820935923","6819019976"}; // Expected order
  std::cout << "GT path length: "  << m.CalculatePathLength(gt) << "miles" << std::endl; // Print the gt path lengths
  bool flag = false;
  if (gt == result.second[result.second.size()-1]) // clockwise
    flag = true;

  std::reverse(gt.begin(),gt.end()); // Reverse the expected order because the counterclockwise result is also correct
  if (gt == result.second[result.second.size()-1]) 
    flag = true;
  // gt.push_back("" + result.second.size());
  // std::pair<double, std::vector<std::vector<std::string>>> test = {0.1, {{"111", "222"}, {"111"}}};
  // EXPECT_EQ(result.second, test.second);
  EXPECT_EQ(result.first, m.CalculatePathLength(gt));
  EXPECT_EQ(flag, true);
}


// Test FindNearby points pass 
TEST(TrojanMapTest, FindNearby) {
  TrojanMap m;
  
  auto result = m.FindNearby("supermarket", "Ralphs", 10, 10);
  std::vector<std::string> ans{"5237417649", "6045067406", "7158034317"};
  EXPECT_EQ(result, ans);
}


TEST(TrojanMaptest, ReadLocationsFromCSVFile){
  TrojanMap m;

  auto input = "/home/hansen/ee538/final-project-lihansen/input/topologicalsort_locations.csv";
  // auto input2 = "/home/hansen/ee538/final-project-lihansen/input/topologicalsort_dependencies.csv";
  std::vector <std::string> output = {"Ralphs", "KFC", "Chick-fil-A"};
  EXPECT_EQ(m.ReadLocationsFromCSVFile(input), output);
  // auto location_names = m.ReadLocationsFromCSVFile(input);
  // auto depend_names = m.ReadDependenciesFromCSVFile(input2);

  // auto result = m.DeliveringTrojan(location_names, depend_names);
  // std::vector<std::string> gt ={"Ralphs", "KFC","Chick-fil-A"};
  // EXPECT_EQ(result, gt);
  

}


TEST(TrojanMaptest, ReadDependenciesFromCSVFile){
  TrojanMap m;

  auto input = "/home/hansen/ee538/final-project-lihansen/input/topologicalsort_dependencies.csv";
  std::vector<std::vector<std::string>> output = {{"Ralphs","Chick-fil-A"}, {"Ralphs","KFC"}, {"Chick-fil-A","KFC"}};
  EXPECT_EQ(m.ReadDependenciesFromCSVFile(input), output);
}