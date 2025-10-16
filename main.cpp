#include <fstream>
#include <iostream>
#include <ostream>

#include "height_estimate.h"

// saving some time. This functionality should go to class
// Also return some enum/state
bool readLogAndSmooth(HeightEstimate& heightEstimator, const std::string &orInFilePath, const std::string& orOutFilePath)
{
    std::ifstream fin(orInFilePath);
    if (!fin.is_open())
    {
        std::cerr << "Error: failed to open data/log1.csv" << std::endl;
        return false;
    }
    std::string line;
    int64_t timestamp;
    double gps, alt1, alt2;
    std::ofstream outFile(orOutFilePath);
    std::cout << "[DHARA_DEBUG] Starting..\n";
    outFile << "timestamp,gps_altitude,altimeter_1_altitude,altimeter_2_altitude,estimated\n";
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
    return true;
}

int main()
{
    // log1
    // TerrainASML: GPS MEDIAN  - avg(two altimeters) 78.26 - 25.69 = 52.57 ~ 52.6
    // Outlier threshold: 3.5 (I tried standard 9, didn't work out, asked AI to help wiht this, this seems to work)
    // I think plane is taking off, initially both altemeters shows 0, so 10 looks fine
    HeightEstimate heightEstimator(52.6, 3.5, 10.0);
    readLogAndSmooth(heightEstimator, "..data/log1.csv", "../data/log1_smoothed.csv");

    // ugh, I should have made setter for terrainAMSL and initial height
    // log2
    HeightEstimate heightEstimatorLog2(148.0, 3.5, 66);
    readLogAndSmooth(heightEstimatorLog2, "../data/log2.csv", "../data/log2_smoothed.csv");
}