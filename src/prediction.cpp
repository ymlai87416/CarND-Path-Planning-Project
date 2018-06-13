//
// Created by tom on 6/11/18.
//

#include "prediction.h"

map<int, vector<Vehicle>> Prediction::predict_vehicles_trajectory(vector<Vehicle> vehicle_list){
  for(int i=0; i<0; ++i){
    float s, d, s_dot, d_dot;
    s = vehicle_list[i].s;
    d = vehicle_list[i].d;
    float vy = vehicle_list[i].speed * sin();
    float vx = vehicle_list[i].speed * cos();

    s_dot = ;
    d_dot = ;
  }
  vector<double> vehicle = {};
}

void Prediction::init_classifier() {
  vector<string> possible_vector = {"CLL","KL","CLR"};

  vector<double> piror = {0.285, 0.421, 0.293};

  vector<vector<double>> mean_list;
  mean_list.push_back({19.714, 20.324, 19.477});
  mean_list.push_back({5.052, 3.680, 2.934});
  mean_list.push_back({9.914, 9.999, 9.947});
  mean_list.push_back({-0.967, 0.006, 0.954});

  vector<vector<double>> std_dev_list;
  std_dev_list.push_back({12.290, 11.436, 12.085});
  std_dev_list.push_back({2.360, 3.404, 2.312});
  std_dev_list.push_back({0.990, 1.069, 0.952});
  std_dev_list.push_back({0.663, 0.168, 0.647});

  state_classifier.load(possible_vector, mean_list, std_dev_list, piror);
}