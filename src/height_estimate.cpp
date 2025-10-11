#include "height_estimate.h"

#include <iostream>

HeightEstimate::HeightEstimate(double fTerrainAsml, double fOutlierThreshold, double fInitialHeight)
    : fTerrainAMSL_(fTerrainAsml)
    , fOutlierThreshold_(fOutlierThreshold)
{
    // State Transition Matrix (F)
    oStateTransition_ << 1.0, dT,
                        0.0, 1.0;

    /// todo Initial conditions
    /// Make it internally all struct. Then use yaml to push it here
    /// 1. accept an initial vertical velocity estimate
    oStateVector_ << fInitialHeight, 0.0; // Initial height and vertical speed
    /// 2. Tune this initial variance based on log?
    oStateCovariance_ << 1e6, 0.0,            // Height variance: ±1000 m
                         0.0, 100.0;          // Velocity variance: ±10 m/s

    /// 3. Calibrate Q/R based on sensor specs, log, make it more configurable via yaml
    oProcessNoiseCovariance_ << 0.1, 0.0,
                                0.0, 0.1; // Initial process noise covariance

    /// 3. Make this configurable
    // Measurement noise variance per sensor (R_i = σ²)
    // Order: [GPS, Altimeter1, Altimeter2]
    // Typical std dev (m): GPS≈2.0, Alt1≈1.0, Alt2≈1.5
    oMeasurementNoiseVariance_ = {
        4.0,   // GPS variance = (2.0 m)^2
        1.0,   // Altimeter 1 variance = (1.0 m)^2
        2.25   // Altimeter 2 variance = (1.5 m)^2
    };

    oMeasurementNoiseVariance_ = { 4.0, 1.0, 2.25 };

    oMeasurementMatrix_  << 1.0, 0.0; // We measure height directly
}

bool HeightEstimate::isValidMeasurement(double fMeasurement)
{
    return (fMeasurement > fMinAltitude_) && (fMeasurement < fMaxAltitude_);
}

double HeightEstimate::TrueHeightAboveGroundLevel(double fGpsAltitude, double fAltimeter1Altitude, double fAltimeter2Altitude) {
    // Predict Step
    // x_t = F * x_(t-1)
    oStateVector_ = oStateTransition_ * oStateVector_;
    // P_t = F * P_(t-1) * F^T + Q
    oStateCovariance_ = oStateTransition_ * oStateCovariance_ * oStateTransition_.transpose() + oProcessNoiseCovariance_;

    std::vector sensorReadings{fGpsAltitude - fTerrainAMSL_, // GPS → AGL
                                fAltimeter1Altitude,
                                fAltimeter2Altitude};

    // Measurement Update Step
    for (auto iIndex = 0; iIndex < sensorReadings.size(); ++iIndex) {
        double measurement = sensorReadings[iIndex];

        if (!isValidMeasurement(measurement)) {
            std::cout << "Invalid measurement: " << measurement << std::endl;
            continue; // Skip invalid measurement for update step
        }

        // Compute the measurement residual
        // y_t = z_t - H * x_t
        double y = measurement - (oMeasurementMatrix_ * oStateVector_)(0, 0);

        // Residual covariance
        double s = (oMeasurementMatrix_ * oStateCovariance_ * oMeasurementMatrix_.transpose())(0, 0)
                     + oMeasurementNoiseVariance_[iIndex];
        // Check for outlier
        // if |yₜ| > threshold * sqrt(Sₜ)
        if (std::abs(y) > fOutlierThreshold_ * std::sqrt(s)) {
            continue; // Skip outlier measurements
        }

        // Kalman Gain (K)
        Eigen::Matrix<double, 2, 1> oKalmanGain = oStateCovariance_ * oMeasurementMatrix_.transpose() * (1.0 / s);

        // Update state estimate and covariance
        // x_t = x_t + K_t * y_
        oStateVector_ = oStateVector_ + oKalmanGain * y;
        // P_t = (I - K_t * H) * P_t
        oStateCovariance_ = (Eigen::Matrix2d::Identity() - oKalmanGain * oMeasurementMatrix_) * oStateCovariance_;
    }
    // Return height above ground level (AGL)
    return oStateVector_(0, 0);
}