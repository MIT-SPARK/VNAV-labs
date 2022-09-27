#include <cstdlib> // required for srand
#include <iostream>
#include "random_vector.h"

int main(){ 
  // initialize rand with constant seed for reproducibility
  std::srand(314159);

  RandomVector rv(20);
  rv.print();
  std::cout << "Mean: " << rv.mean() << std::endl;
  std::cout << "Min: " << rv.min() << std::endl;
  std::cout << "Max: " << rv.max() << std::endl;

  std::cout << "Histogram:" << std::endl;
  rv.printHistogram(5);
  std::cout << std::endl;
}
