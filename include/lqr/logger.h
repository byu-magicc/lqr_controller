#pragma once

#include <cstdint>
#include <deque>
#include <fstream>
#include <unistd.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Logger
{
public:
    Logger(std::string filename)
    {
        file_.open(filename);
    }

    ~Logger()
    {
        file_.close();
    }
    template <typename... T>
    void log(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)&data, sizeof(T)), 1)... };
    }

    template <typename... T>
    void logVectors(T... data)
    {
        int dummy[sizeof...(data)] = { (file_.write((char*)data.data(), sizeof(typename T::Scalar)*data.rows()*data.cols()), 1)... };
    }

private:
    std::ofstream file_;
};
