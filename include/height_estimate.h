#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

/**
 * @brief Running height-above-ground-level estimate using GPS and altimeter fusion.
 * Implements a 2-state (height, vertical speed) Kalman filter.
 */
class HeightEstimate
{
public:
    /**
     * @param fTerrainAMSL Terrain's AMSL (Above Mean Sea Level)
     * @param fOutlierThreshold Outlier threshold for residual test (default: 9.0)
     * @param fInitialHeight Initial height AGL estimate, default 0.0
     */
    HeightEstimate(double fTerrainAMSL, double fOutlierThreshold = 9.0, double fInitialHeight = 0.0);
    ~HeightEstimate() = default;

    /**
     * @brief Fuse GPS and two altimeter measurements to estimate true height AGL.
     */
    double TrueHeightAboveGroundLevel(double fGpsAltitude,
                                      double fAltimeter1Altitude,
                                      double fAltimeter2Altitude);

    bool isValidMeasurement(double fMeasurement);

private:
    Eigen::Matrix<double, 2, 1> oStateVector_;              ///< [height, vertical speed]
    Eigen::Matrix<double, 2, 2> oStateTransition_;          ///< F
    Eigen::Matrix<double, 2, 2> oStateCovariance_;          ///< P
    Eigen::Matrix<double, 2, 2> oProcessNoiseCovariance_;   ///< Q
    Eigen::Matrix<double, 1, 2> oMeasurementMatrix_;        ///< H
    std::array<double, 3> oMeasurementNoiseVariance_{}; ///< R for each sensor (GPS, Alt1, Alt2)


    double fTerrainAMSL_{0.0};  ///< Terrain's AMSL (Above Mean Sea Level)
    double fOutlierThreshold_{9.0};
    double fMinAltitude_{0.0};
    double fMaxAltitude_{15000.0};
    const double dT{0.01}; ///< Time step (sec) todo: make configurable
};
