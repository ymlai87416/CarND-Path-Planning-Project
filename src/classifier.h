#ifndef CLASSIFIER_H
#define CLASSIFIER_H
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class GNB {
public:

  vector<string> possible_labels = {"left","keep","right"};


  /**
    * Constructor
    */
  GNB();

  /**
   * Destructor
   */
  virtual ~GNB();

  void train(vector<vector<double> > data, vector<string>  labels);

  void load(vector<string> labels, vector<vector<double>> means, vector<vector<double>> std_devs, vector<double> prior);

  string predict(vector<double>);

  string predict_detail(vector<double>);

private:
  double** means;
  double** std_devs;
  double* pirors;
  void reset();
};

#endif



