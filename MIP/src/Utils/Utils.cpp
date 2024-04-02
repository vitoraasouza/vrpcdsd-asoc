#include "Utils.h"

int cmp(double x, double y, double tol) {
   return ( x <= y + tol ) ? ( x + tol < y ) ? -1 : 0 : 1;
}
