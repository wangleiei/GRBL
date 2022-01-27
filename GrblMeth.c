#include "GrblMeth.h"

#ifdef __cplusplus
   extern "C"{ //在最顶层调用h文件使用，
#endif 
void GrblInit(){

}

GRBL_METH GrblMeth;


void printInteger(int32_t n){
  uint8_t temp[20];
  sprintf(temp,"%d",n);
  GrblMeth.printPgmString(temp);
}
void printFloat(float n){
  uint8_t temp[20];
  sprintf(temp,"%f",n);
  GrblMeth.printPgmString(temp);
}

#ifdef __cplusplus
}
#endif