#include <iostream>
#include <vector>

class RandomVector{
  std::vector<double> vect;
  
  public:
    RandomVector(int size, double max_val = 1);
    void print();
    double mean();
    double max();
    double min();
    void printHistogram(int bins);
};