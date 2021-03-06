
#include "PressFilter.h"


PressFilter::PressFilter() {
  for(int i=0; i<pressFilterSize; i++){
    filerElements[i] = 0;
  }
  
}

int PressFilter::filter(int dataIn) {
  int sum = 0;
  // 1- in put data
    for(int i=pressFilterSize-1; i>0; i--){
     filerElements[i] = filerElements[i-1];
     sum |= filerElements[i];
    }
    filerElements[0] = dataIn;
    sum |= filerElements[0]; 
   return sum;
}
