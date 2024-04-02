#ifndef UTILS_H_
#define UTILS_H_

#include <chrono>
#include <iostream>
#include <sys/time.h>
#include <sys/times.h>
#include <unistd.h>
#include <utility>
#include <vector>

using namespace std;

int cmp(double x, double y, double tol = 1e-6);

#define time_now() chrono::high_resolution_clock::now()
#define duration(x) chrono::duration<double>(x).count()

#endif /* UTILS_H_ */
