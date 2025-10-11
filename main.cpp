#include <fstream>
#include <iostream>
#include <ostream>

#include "height_estimate.h"

int main()
{
    // log1
    HeightEstimate heightEstimator(100.0, 9.0, 10.0); // Terrain AMSL = 100m, Outlier Threshold = 9.0, Initial Height = 10m
    std::ifstream fin("../data/log1.csv");
    if (!fin.is_open())
    {
        std::cerr << "Error: failed to open data/log1.csv" << std::endl;
        return 1;
    }
    std::string line;
    int64_t timestamp;
    double gps, alt1, alt2;
    std::ofstream outFile("../data/log1_smoothed.csv");
    std::cout << "[DHARA_DEBUG] Starting..\n";
    while (std::getline(fin, line))
    {
        std::sscanf(line.c_str(), "%lld,%lf,%lf,%lf", &timestamp, &gps, &alt1, &alt2);
        if (timestamp % 100 == 0) {
            std::cout << "[DHARA_DEBUG] Processing timestamp: " << timestamp << "\n";
        }
        double heightAGL = heightEstimator.TrueHeightAboveGroundLevel(gps, alt1, alt2);
        outFile << timestamp << "," << gps << "," << alt1 << "," << alt2 << "," << heightAGL << std::endl;
    }
}