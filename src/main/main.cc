#include <iostream>
#include "src/lib/trojanmap.h"
#include "src/lib/mapui.h"
// #define NCURSES

int main() {  
  // for debugging
  TrojanMap tm = TrojanMap();

  // auto result = tm.CalculateShortestPath_Dijkstra("Ralphs", "Chase");
  // for (auto item : result){
  //   std::cout<<"\""<<item<<"\",";
  // }
  // tm.Autocomplete("Chas");
  // tm.FindClosestName("Rolphs");

  // std::vector <double> square1 = {-118.299, -118.264, 34.032, 34.011};
  // std::vector <double> square2 = {-118.290, -118.289, 34.030, 34.020};
  // std::vector <std::string > subgraph = {};
  // auto res = tm.CycleDetection(subgraph, square2);
  // std::cout<<true;
  // TrojanMap m;
  
  // std::vector<std::string> input{"6819019976","6820935923","122702233","8566227783","8566227656","6816180153","1873055993","7771782316"}; // Input location ids 
  // auto result = m.TravellingTrojan_Brute_force(input);

  MapUI x;
  #ifdef NCURSES
    x.PlotMap();
    x.DynamicPrintMenu();
  #else
    x.PlotMap();
    x.PrintMenu();
  #endif
  return 0;
}






