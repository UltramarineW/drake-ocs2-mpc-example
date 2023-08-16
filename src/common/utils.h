#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <gflags/gflags.h>

std::string GetAbsolutePath(const std::string &path);

#endif // UTILS_H_