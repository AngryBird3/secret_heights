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
    oStateCovariance_ << 25.0, 0.0,   // height std ≈ 5 m
                         0.0,  4.0;   // vel   std ≈ 2 m/s


    /// 3. Calibrate Q/R based on sensor specs, log, make it more configurable via yaml
    oProcessNoiseCovariance_ << 0.05, 0.0,
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

    oMeasurementMatrix_  << 1.0, 0.0; // We measure height directly
}

bool HeightEstimate::isValidMeasurement(double fMeasurement)
{
    if (!std::isfinite(fMeasurement)) return false;
    return (fMeasurement >= fMinAltitude_) && (fMeasurement <= fMaxAltitude_); // >= and <=

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

    static int debug_count = 0;

    // Measurement Update Step
    for (auto iIndex = 0; iIndex < sensorReadings.size(); ++iIndex) {
        double measurement = sensorReadings[iIndex];

        if (!isValidMeasurement(measurement)) {
            std::cout << "[Sensor " << iIndex << "] Invalid measurement: " << measurement << std::endl;
            continue; // Skip invalid measurement for update step
        }

        // Compute the measurement residual
        // y_t = z_t - H * x_t
        double y = measurement - (oMeasurementMatrix_ * oStateVector_)(0, 0);

        // Residual covariance
        double s = (oMeasurementMatrix_ * oStateCovariance_ * oMeasurementMatrix_.transpose())(0, 0)
                     + oMeasurementNoiseVariance_[iIndex];
        if (s < 1e-10)
        { // Arbitrary small threshold
            std::cout << "[Sensor " << iIndex << "] Residual covariance too small: " << s << std::endl;
            continue;
        }
        // Check for outlier
        // if |yₜ| > threshold * sqrt(Sₜ)
        if (std::abs(y) > fOutlierThreshold_ * std::sqrt(s)) {
            std::cout << "[Sensor " << iIndex << "] Outlier detected: residual = " << y << std::endl;
            continue; // Skip outlier measurements
        }

        // Kalman Gain (K)
        Eigen::Matrix<double, 2, 1> oKalmanGain = oStateCovariance_ * oMeasurementMatrix_.transpose() * (1.0 / s);

        // Update state estimate and covariance
        // x_t = x_t + K_t * y_
        oStateVector_ = oStateVector_ + oKalmanGain * y;
        // P_t = (I - K_t * H) * P_t
        // oStateCovariance_ = (Eigen::Matrix2d::Identity() - oKalmanGain * oMeasurementMatrix_) * oStateCovariance_;
        // Joseph form for numerical stability
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        Eigen::Matrix2d KH = oKalmanGain * oMeasurementMatrix_;
        oStateCovariance_ = (I - KH) * oStateCovariance_ * (I - KH).transpose() +
                            oKalmanGain * oMeasurementNoiseVariance_[iIndex] * oKalmanGain.transpose();

        if (debug_count++ < 100) {
            std::cout << "[UPDATE s=" << iIndex << "] z=" << measurement
                      << " y=" << y
                      << " S=" << s
                      << " K=" << oKalmanGain.transpose()
                      << " x=" << oStateVector_.transpose()
                      << std::endl;
        }

    }

    // Return height above ground level (AGL)
    return oStateVector_(0, 0);
}
