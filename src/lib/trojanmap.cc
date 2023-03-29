#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string& id) {
  if (this->data.count(id)){
    return double(this->data[id].lat);
  }
  return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist, return -1.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string& id) { 
  if (this->data.count(id)){
    return double(this->data[id].lon);
  }
  return -1;
}

/**
 * GetName: Get the name of a Node given its id. I f id does not exist, return "NULL".
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string& id) { 
  if (this->data.count(id)){
    return this->data[id].name;
  }
  return "NULL";
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return an empty vector.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string& id) {
  if (this->data.count(id)){
    return this->data[id].neighbors;
  }
  return {};
}

/**
 * GetID: Given a location name, return the id. 
 * If the node does not exist, return an empty string. 
 *
 * @param  {std::string} name          : location name
 * @return {int}  : id
 */
std::string TrojanMap::GetID(const std::string& name) {
  std::string res = "";
  for (auto pair: this->data){
    if (pair.second.name == name){
      return pair.second.id;
    }
  }

  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  auto id = this->GetID(name);
  if (id != ""){
    return std::make_pair(this->data[id].lat, this->data[id].lon);
  }

  std::pair<double, double> results(-1, -1);
  return results;
}


/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * 
 */


int TrojanMap::CauculateEditDistance_recursion(std::string a, std::string b, int i, int j ){
  if (j == 0 ){
    return i;
  }else if(i == 0){
    return j;
  }else if (a[i-1] == b[j-1]){
    return CauculateEditDistance_recursion(a, b, i-1, j-1);
  }else{
    int m1 = CauculateEditDistance_recursion(a, b ,i-1, j);
    int m2 = CauculateEditDistance_recursion(a, b, i, j-1);
    int m3 = CauculateEditDistance_recursion(a, b, i-1, j-1);
    return std::min({m1, m2, m3}) +1;
  }
}

int TrojanMap::CalculateEditDistance(std::string a, std::string b){
  // return CauculateEditDistance_recursion(a, b, a.size()-1, b.size()-1);
  int srcLength = a.size();
	int targetLength = b.size();
	int dp[srcLength + 1][targetLength + 1];
  // init distance matrix 
  // target string is empty
  for (int i = 0; i <= srcLength; ++i){
		dp[i][0] = i;        
	}
  // src string is empty 
	for (int j = 0; j <= targetLength; ++j){
		dp[0][j] = j;
	}
	for (int i = 1; i <= srcLength; ++i){
		for (int j = 1; j <= targetLength; ++j){
			if (a[i - 1] == b[j - 1]){ // same char, same distance 
				dp[i][j] = dp[i - 1][j - 1];    
			}
			else{   // three ways to get into current state 
				dp[i][j] = std::min({dp[i - 1][j - 1], dp[i - 1][j], dp[i][j - 1]}) + 1;
			}
		}
	}

	return dp[srcLength][targetLength];
}

/**
 * FindClosestName: Given a location name, return the name with smallest edit distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : similar name
 */
std::string TrojanMap::FindClosestName(std::string name) {
  int min_editdist = INT32_MAX;
  std::string closest_name = "";
  for (auto &pair: this->data){
    if (pair.second.name != ""){
      int edit_dist = this->CalculateEditDistance(name, pair.second.name);
      if (edit_dist < min_editdist){
        min_editdist = edit_dist;
        closest_name = pair.second.name;
      }
    }
  }
  return closest_name;
  // if (min_editdist == 0)return closest_name;
  // else{
  //   std::string warning = closest_name;
  //   return warning;
  // }
}


/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results = {};

  for (auto &pair:this->data){
    if (pair.second.name.size() > 0){ //exclude the "" name nodes
      int length = name.size();
      int i=0;
      // from left to right, compare the given name with every node name in out data
      for (i=0;i<name.size();i++){
        char c = name[i];
        char x = pair.second.name[i];
        // convert to lower case
        if (c >= 65 and c <= 90){
          c = c + 32;
        }
        if (x >= 65 and x <= 90){
          x = x + 32;
        }

        if (x != c){
          break;
        }
      }

      if (i == name.size()){
        results.push_back(pair.second.name);
      }
    }
  }
  return results;
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id, const std::string &b_id) {
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  // Do not change this function
  double sum = 0;
  for (int i = 0;i < int(path.size())-1; i++) {
    sum += CalculateDistance(path[i], path[i+1]);
   }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */

Node TrojanMap::findNodeByName(std::string name){
  for(auto pair: this->data){
    // std::string a = pair.second.name;
    if (pair.second.name == name){
      return pair.second;
    }
  }
  // not found
  return Node();
}

std::string TrojanMap::findMinIDButNotInVisited(std::map<std::string, double> &mindis, std::map<std::string, bool> &visited){
  
  std::string minId;
  double m = DBL_MAX;

  for(auto it : mindis){
    //if there is no node it in the visited list, and it has the min-dis 
    if(visited[it.first] != true && it.second < m){
      m = it.second;
      minId = it.first;
    }
  }
  visited[minId] = true;
  return minId;
}


std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(std::string location1_name, 
                                                                    std::string location2_name) {
  std::vector<std::string> path;
  std::string start_id = GetID(location1_name);
  std::string destination_id = GetID(location2_name);
  struct elem
  {
    std::string id;
    double min_weight;
    std::string prev_id;

    bool operator>(const elem &e) const
    {
      return min_weight > e.min_weight;
    }
  };

  std::priority_queue<elem, std::vector<elem>, std::greater<elem>> u;
  u.push({start_id, 0, ""});

  std::unordered_map<std::string, elem> records_by_id;
  for (auto it = data.begin(); it != data.end(); it++)
  {
    std::string node_id = it->second.id;
    records_by_id[node_id] = {node_id, (double)(node_id == start_id ? 0 : __INT_MAX__), ""};
  }

  while (!u.empty())
  {
    auto start_record = u.top();
    std::string start = start_record.id;
    u.pop();

    auto neighbors = GetNeighborIDs(start);
    for (int i = 0; i < neighbors.size(); i++)
    {
      std::string neighbor = neighbors[i];
      double w = CalculateDistance(start, neighbor);
      if (records_by_id[neighbor].min_weight > start_record.min_weight + w)
      {
        records_by_id[neighbor].min_weight = start_record.min_weight + w;
        records_by_id[neighbor].prev_id = start;
        u.push({neighbor, records_by_id[neighbor].min_weight, start});
      }
    }
  }

  while (destination_id != "")
  {
    path.push_back(destination_id);
    destination_id = records_by_id[destination_id].prev_id;
  }

  std::reverse(path.begin(), path.end());
  return path;


}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id. Hint: Do the early termination when there is no change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(std::string location1_name, 
                                                                      std::string location2_name){
  std::vector<std::string> path;
  std::string start_id = GetID(location1_name);
  std::string destination_id = GetID(location2_name);
  struct edge
  {
    std::string from;
    std::string to;
    double w;
  };
  struct elem
  {
    double min_weight;
    std::string prev_id;
  };

  std::vector<edge> Edges;
  for (auto &&node : data)
  {
    std::string from = node.second.id;
    for (int i = 0; i < node.second.neighbors.size(); i++)
    {
      std::string to = node.second.neighbors[i];
      Edges.push_back({from, to, CalculateDistance(from, to)});
    }
  }
  std::unordered_map<std::string, elem> records;
  for (auto it = data.begin(); it != data.end(); it++)
  {
    double opt_w = it->second.id == start_id ? 0 : __INT_MAX__;
    records[it->second.id] = {opt_w, ""};
  }
  int flag_stop = 0;
  double prev_min_weight;
  for (int i = 1; i <= data.size() - 1; i++)
  {
    prev_min_weight = records[destination_id].min_weight;
    for (int j = 0; j < Edges.size(); j++)
    {
      std::string from = Edges[j].from;
      std::string to = Edges[j].to;
      double w = Edges[j].w;
      if (records[from].min_weight != __INT_MAX__ and records[from].min_weight + w < records[to].min_weight)
      {
        records[to].min_weight = records[from].min_weight + w;
        records[to].prev_id = from;
        if (to == destination_id)
          flag_stop = 0;
      }
    }
    if (records[destination_id].min_weight != __INT_MAX__)
    {
      if (records[destination_id].min_weight == prev_min_weight)
        flag_stop++;
      if (flag_stop == 15)
        break;
    }
  }
  while (destination_id != "")
  {
    path.push_back(destination_id);
    destination_id = records[destination_id].prev_id;
  }
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : 
 * a pair of total distance and the all the progress to get final path
 */


// first elem is the start location 
// traverse all the positions in the location ids 
// calculate the distance to the next location and add them together 
// add the <distance, path> to the records
// find the min distance and push it to the last
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Brute_force(
                                    std::vector<std::string> location_ids) {
  // std::pair<double, std::vector<std::vector<std::string>>> records;
  std::vector<std::vector<std::string>> routes;
  std::vector<std::string> cur_path = {location_ids[0]};
  double min_distance = DBL_MAX;
  TravellingTrojan_Brute_force_aux(routes, location_ids, min_distance, cur_path);
  return std::make_pair(min_distance, routes);
}



void TrojanMap::TravellingTrojan_Brute_force_aux(std::vector<std::vector<std::string>> &records,
                                                  std::vector<std::string> &location_ids, 
                                                  // double cur_distance, 
                                                  double &min_distance, 
                                                  std::vector<std::string> cur_path){
  
  if (cur_path.size() == location_ids.size()){
    // return back to the origin
    // double cur_distance = this->CalculateDistance(cur_path[cur_path.size()-1], location_ids[0]); 
    cur_path.push_back(location_ids[0]);
    double cur_distance = this->CalculatePathLength(cur_path);
    
    if(cur_distance < min_distance){ // min distance, 
      // update 'min_distance' 
      min_distance = cur_distance;
      // min distance, so push this optim path back 
      records.push_back(std::move(cur_path));
    }else{ // not min distance 
      records.insert(records.begin(), std::move(cur_path)); 
    }

  }else{
    for( int i = 0; i< location_ids.size(); i++){
      auto item = std::find(cur_path.begin(), cur_path.end(), location_ids[i]);
      if (item == cur_path.end()){ // not found in cur_path
        // cur_distance += ;
        cur_path.push_back(location_ids[i]);
        this->TravellingTrojan_Brute_force_aux(records, location_ids, 
        // cur_distance+this->CalculateDistance(location_ids[i], cur_path[cur_path.size()-1]),
        min_distance, cur_path);
        cur_path.pop_back();
      }
    }   
  }
}




std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_Backtracking(
                                    std::vector<std::string> location_ids) {

  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::unordered_set<int> visit;
  std::vector<int> current_path;
  if (location_ids.size() == 0) {
    return results;
  }
  if (location_ids.size() == 1) {
    results.first = 0;
    results.second = {{location_ids[0]}};
    return results;
  }

  std::vector<std::vector<double>> weights(location_ids.size(),std::vector<double>(location_ids.size()));
  for(auto i=0;i<location_ids.size();i++){
    for(auto j=i+1;j<location_ids.size();j++){
      weights[i][j] = CalculateDistance(location_ids[i],location_ids[j]);
      weights[j][i] = weights[i][j];
    }
  }

  visit.insert(0);
  current_path.push_back(0);
  double resD = DBL_MAX;
  std::vector<std::vector<std::string>> resP;
  
  TravellingTrojanHelper(resP,location_ids,weights,visit,resD,current_path,0);
  results = std::make_pair(resD,resP);

  return results;
}


// helper
void TrojanMap::TravellingTrojanHelper(std::vector<std::vector<std::string>> &path,
                                      std::vector<std::string> &location_ids,
                                      std::vector<std::vector<double>> &weights, 
                                      std::unordered_set<int> &numvisited,
                                      double &resD, 
                                      std::vector<int> &curP, 
                                      double current_dist){

  if(curP.size()==location_ids.size()){
    double sumD = current_dist + weights[curP.back()][0];
    if(sumD<resD){
      std::vector<std::string> t;
      for(auto i:curP)
      t.push_back(location_ids[i]);
      t.push_back(location_ids[0]);
      path.push_back(move(t));
      resD = sumD;
    }
  }
  if(current_dist<resD){
    for(auto i=1;i<location_ids.size();i++){
      if(numvisited.count(i)==0){
        numvisited.insert(i);
        double deltaDist = weights[curP.back()][i];
        curP.push_back(i);
        TravellingTrojanHelper(path,location_ids,weights,numvisited,resD,curP,current_dist+deltaDist);
        curP.pop_back();
        numvisited.erase(i);
      }
    }
  }
}



std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> location_ids){
  // std::pair<double, std::vector<std::vector<std::string>>> records;
  // return records;

  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::vector<std::string>> path;  
  if (location_ids.size() <= 1) {
    return results;
  }
  std::vector<std::string> curP = location_ids;
  curP.push_back(location_ids[0]);
  double minD = CalculatePathLength(curP);
  path.push_back(curP);
  bool flag = true;
  while(flag){
    flag = false;
    for(auto i=1;i<curP.size()-1;i++){
      for(auto j=i+1;j<curP.size();j++){
        reverse(curP.begin()+i,curP.begin()+j);
        double curD = CalculatePathLength(curP);
        if(curD<minD){
          flag = true;
          minD = curD;
          path.push_back(curP);
        }
        else{
          reverse(curP.begin()+i,curP.begin()+j);
        }
      }
    }
  }
  results = std::make_pair(minD,path);
  
  return results;
}



/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  // locations_filename = 
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line;

  getline(fin, line); // skip first line: 'Name'
  while(getline(fin,line)){
    location_names_from_csv.push_back(line);
  }

  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;

  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;

  getline(fin, line); // skip first line: 'Source, Destination'
  while(getline(fin, line)){
    std::stringstream s(line);
    
    std::vector <std::string> pair = {};
    while (getline(s , word, ',')){
      pair.push_back(word);
    }
    dependencies_from_csv.push_back(pair);
  }

  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies. If there is no way to do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  // std::vector<std::string> result;
  // return result;              

  std::vector<std::string> result;
  
  std::unordered_map<std::string,std::vector<std::string>> neighbors;
  //initialize neighbors list
  for(auto &loc:locations){
    neighbors[loc] = std::vector<std::string>();//all neighbors of the location is empty
  }
  for(auto &n:dependencies){//iterate dependencies list
    //no feasible route exists
    if(neighbors.count(n[0])==0 || neighbors.count(n[1])==0){//one of the two depending elements are not in the neighnors list 
      return {};
    }
    //push n[1] into n[0]'s neighbors list
    neighbors[n[0]].push_back(n[1]);
  }
  //initialize the visited list
  std::unordered_map<std::string,int> states;
  for(auto &loc:locations){
    states[loc] = 0;
  }
  for(auto &loc:locations){
    //if unvisited node doesn't have the delivering path
    if(states[loc]==0 && !DeliveringTrojanHelper(loc,neighbors,states,result)){
      return {};
    }
  }
  reverse(result.begin(),result.end());   
  return result;   

}



bool TrojanMap::DeliveringTrojanHelper(std::string &loc,
            std::unordered_map<std::string,std::vector<std::string>> &neighbors,
            std::unordered_map<std::string,int> &states,
            std::vector<std::string> &result){
  states[loc] = 1;//current location state
  for(std::string &n:neighbors[loc]){//get all neighbors of current location
    //if one of its neighbor is visited
    if(states[n]==1){
      return false;
    }
    //if the n is not visited, and it doesn't have the delivering path
    if(states[n]==0 && !DeliveringTrojanHelper(n,neighbors,states,result)){
      return false;
    }
  }
  states[loc] = 2;
  result.push_back(loc);
  return true;
}

/**
 * inSquare: Give a id retunr whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square){
  double left = square[0];
  double right = square[1];
  double upper = square[2];
  double lower = square[3];
  double lon = this->GetLon(id);
  double lat = this->GetLat(id);

  return lon>=left and lon<=right and lat<=upper and lat>=lower;

}


/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square) {
  std::vector<std::string> subgraph;
  for(auto &n:data){
    if(this->inSquare(n.first, square)){
      subgraph.push_back(n.first);
    }  
  }
  return subgraph;
}



/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<std::string>} subgraph: list of location ids in the square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, 
  std::vector<double> &square) {
  std::unordered_map<std::string, bool> visited;
  std::unordered_map<std::string, std::string> pred;
  subgraph = this->GetSubgraph(square);

  for (auto &id : subgraph){
    visited[id] = false;
  }
  for(auto &pair : visited){ // pair: <id, bool>
    if (pair.second == false and hasCycle(pair.first, visited, pred, square)){
      return true;
    }
  }

  return false;
}

bool TrojanMap::hasCycle(std::string current_id, std::unordered_map<std::string, bool> &visited,
std::unordered_map<std::string, std::string> &pred, std::vector<double> &square){
  visited[current_id] = true;
  auto neighbours = GetNeighborIDs(current_id);
  for (auto &neighbour_id:neighbours){
    pred[neighbour_id] = current_id;
    // if (this->inSquare(neighbour_id, square) ){
      if (visited[neighbour_id] == false ){
        return hasCycle(neighbour_id, visited, pred, square);
      }
      if (visited[neighbour_id] == true and neighbour_id != pred[current_id]){
        return true;
      }
    // }
  }
  return false;
}




/**
 * FindNearby: Given a class name C, a location name L and a number r, 
 * find all locations in class C on the map near L with the range of r and return a vector of string ids
 * 
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {int} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k) {
  auto location_id = this->GetID(name);
  std::vector <std::string> res = {};
  std::vector <std::pair<std::string, double>> nearby = {};
  
  for (auto pair : this->data){
    if (pair.first != location_id){ // same node, skip
      if (pair.second.attributes.count(attributesName) > 0){ // target attribute in this pair
        auto distance = this->CalculateDistance(location_id, pair.first);
        if (distance < r){
          auto p = std::make_pair(pair.first, distance);
          nearby.push_back(p);
        }
      }
    }
  }

  std::sort(nearby.begin(), nearby.end(), this->CompareByDistance);
  for (auto &pair : nearby){
    res.push_back(pair.first);
  }
  return res;
}


bool TrojanMap::CompareByDistance(std::pair<std::string, double> l1, 
                                  std::pair<std::string, double> l2){
  return l1.second < l2.second;
}



/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

