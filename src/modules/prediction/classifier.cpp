#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "classifier.h"
#include <cstring>
#define PI 3.1415926
/**
 * Initializes GNB
 */



GNB::GNB() {
  means= 0;
  std_devs = 0;
  pirors = 0;
}

GNB::~GNB() {
  Reset();
}

void GNB::Reset()
{
  if(means != 0){
    for(int i=0; i<possible_labels.size(); ++i)
      delete[] means[i];
  }
  delete[] means;

  if(std_devs != 0){
    for(int i=0; i<possible_labels.size(); ++i)
      delete[] std_devs[i];
  }
  delete[] std_devs;
  delete[] pirors;

  means = 0; std_devs = 0; pirors=0;
}

void GNB::Train(vector<vector<double>> data, vector<string> labels)
{

  /*
    Trains the classifier with N data points and labels.

    INPUTS
    data - array of N observations
      - Each observation is a tuple with 4 values: s, d,
        s_dot and d_dot.
      - Example : [
          [3.5, 0.1, 5.9, -0.02],
          [8.0, -0.3, 3.0, 2.2],
          ...
        ]

    labels - array of N labels
      - Each label is one of "left", "keep", or "right".
  */

  Reset();

  int attribute_len = data[0].size();
  int num_labels = possible_labels.size();
  means = new double*[num_labels];
  std_devs = new double*[num_labels];
  pirors = new double[possible_labels.size()];

  for(int i=0; i<num_labels; ++i){
    means[i] = new double[attribute_len];
    std_devs[i] = new double[attribute_len];
  }

  double* mean = new double[attribute_len];
  double* std_dev = new double[attribute_len];

  for(int p=0; p<possible_labels.size(); ++p){
    string label = possible_labels[p];


    memset(mean, 0, sizeof(double) * attribute_len);
    memset(std_dev, 0, sizeof(double) * attribute_len);
    int n = 0;

    for(int i=0; i<data.size(); ++i){
      if(labels[i] == label){
        //online mean and std-dev calculation
        ++n;
        for(int j=0; j<attribute_len; ++j){
          double prev_mean = mean[j];
          mean[j] = mean[j] + (data[i][j]-mean[j]) / n;
          std_dev[j] = std_dev[j] + (data[i][j]-mean[j]) * (data[i][j]-prev_mean);
        }
      }
    }

    for(int j=0; j<attribute_len; ++j){
      means[p][j] = mean[j];
      std_devs[p][j] = sqrt(std_dev[j]/(n));
    }

    pirors[p] = n*1.0/data.size();
  }

  delete[] mean;
  delete[] std_dev;

  /*
    for(int i=0; i<num_labels; ++i){
        string s = possible_labels[i];
        for(int j=0; j<attribute_len; ++j){
            printf("%s attr %d: %.3f %.3f\n", s.c_str(), j, means[i][j], std_devs[i][j]);
        }
    }
    */
}

string GNB::Predict(vector<double> sample)
{
  /*
    Once trained, this method is called and expected to return
    a predicted behavior for the given observation.

    INPUTS

    observation - a 4 tuple with s, d, s_dot, d_dot.
      - Example: [3.5, 0.1, 8.5, -0.2]

    OUTPUT

    A label representing the best guess of the classifier. Can
    be one of "left", "keep" or "right".
    """
    # TODO - complete this
  */

  int attribute_len = sample.size();
  int num_labels = possible_labels.size();
  double* scores = new double[num_labels];
  for(int i=0; i<num_labels; ++i) scores[i] = pirors[i];

  for(int i=0; i<num_labels; ++i){
    for(int j=0; j<attribute_len; ++j){
      //exponent = math.exp(-(math.pow(x-mean,2)/(2*math.pow(stdev,2))))
      //return (1 / (math.sqrt(2*math.pi) * stdev)) * exponent
      double exponent = exp(-pow(sample[j]-this->means[i][j], 2)/2/pow(this->std_devs[i][j], 2));
      double score_attr = (1 / sqrt(2*PI)/ this->std_devs[i][j]) * exponent;
      //printf("aa %.3f %.3f", exponent, score_attr);
      scores[i] = scores[i] * score_attr;
    }
  }
  //printf("scoring (%.3f %.3f %.3f %.3f) => %.5f %.5f %.5f\n", sample[0],sample[1],sample[2],sample[3], scores[0], scores[1], scores[2] );

  string result = this->possible_labels[0];
  double cur_max = scores[0];
  for(int i=1; i<num_labels; ++i){
    if(cur_max < scores[i]){
      cur_max = scores[i];
      result = this->possible_labels[i];
    }
  }
  //printf("scoring (%.3f %.3f %.3f %.3f) => %s\n",sample[0],sample[1],sample[2],sample[3], result.c_str());

  delete[] scores;

  return result;

}

string GNB::PredictDetail(vector<double> sample)
{
  int attribute_len = sample.size();
  int num_labels = possible_labels.size();
  double* scores = new double[num_labels];
  for(int i=0; i<num_labels; ++i) scores[i] = pirors[i];

  for(int i=0; i<num_labels; ++i){
    for(int j=0; j<attribute_len; ++j){
      //exponent = math.exp(-(math.pow(x-mean,2)/(2*math.pow(stdev,2))))
      //return (1 / (math.sqrt(2*math.pi) * stdev)) * exponent
      double exponent = exp(-pow(sample[j]-this->means[i][j], 2)/2/pow(this->std_devs[i][j], 2));
      double score_attr = (1 / sqrt(2*PI)/ this->std_devs[i][j]) * exponent;
      //printf("aa %.3f %.3f", exponent, score_attr);
      scores[i] = scores[i] * score_attr;
    }
  }

  double sum = 0;
  for(int i=0; i<num_labels; ++i)
    sum += scores[i];

  double* norm_scores = new double[num_labels];
  for(int i=0; i<num_labels; ++i)
    norm_scores[i] = scores[i] / sum;

  string result = "";
  for(int i=0; i<num_labels;++i){
    result = result + possible_labels[i] + "(" + std::to_string(norm_scores[i]) + ") ";
  }

  delete[] scores;
  delete[] norm_scores;

  return result;

}

void GNB::Load(vector<string> labels, vector<vector<double>> mean_list, vector<vector<double>> std_dev_list,
               vector<double> prior_list)
{
  this->possible_labels = labels;

  int label_count = labels.size();
  int attr_count = mean_list[0].size();

  means = new double*[label_count];
  for(int i=0; i<label_count; ++i) {
    means[i] = new double[attr_count];
    for(int j=0; j<attr_count; ++j)
      means[i][j] = mean_list[i][j];
  }

  std_devs = new double*[label_count];
  for(int i=0; i<label_count; ++i) {
    std_devs[i] = new double[attr_count];
    for(int j=0; j<attr_count; ++j)
      std_devs[i][j] = std_dev_list[i][j];
  }

  pirors = new double[label_count];
  for(int i=0; i<label_count; ++i)
    pirors[i] = prior_list[i];
}