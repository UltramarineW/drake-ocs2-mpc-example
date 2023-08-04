#pragma once
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <gflags/gflags.h>

std::string GetAbsolutePath(const std::string &path);