#include <fstream>
#include <iostream>
#include <ostream>

#include "height_estimate.h"

int main()
{
    // log1
    // TerrainASML: GPS MEDIAN  - avg(two altimeters)
    HeightEstimate heightEstimator(52.6, 3.5, 10.0);
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
        int parsed = std::sscanf(line.c_str(), "%lld,%lf,%lf,%lf", &timestamp, &gps, &alt1, &alt2);
        if (parsed != 4)
        {
            std::cerr << "Error: failed to parse line: " << line << std::endl;
            continue;
        }
        if (timestamp % 100 == 0) {
            std::cout << "[DHARA_DEBUG] Processing timestamp: " << timestamp << "\n";
        }
        double heightAGL = heightEstimator.TrueHeightAboveGroundLevel(gps, alt1, alt2);
        outFile << timestamp << "," << gps << "," << alt1 << "," << alt2 <<"," << heightAGL << std::endl;
    }
}