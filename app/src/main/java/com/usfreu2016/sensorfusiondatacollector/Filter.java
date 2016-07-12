package com.usfreu2016.sensorfusiondatacollector;

import com.estimote.sdk.Utils;
import com.estimote.sdk.eddystone.Eddystone;

import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class Filter {

    /** Process noise - estimated low value - most noise comes from measurements */
    final double PROCESS_NOISE = 0.008;
    final double PROCESS_NOISE_VARIANCE = 0.0064;

    /** Measurement noise*/
    final double MEASUREMENT_NOISE = 3;
    final double MEASUREMENT_NOISE_VARIANCE = 40;

    /** State variance/covariance */
    final double STATE_VARIANCE = 500.0;

    private KalmanFilter filter;

    /** Constructor */
    public Filter() {

        /** Matrices and Vectors */

        // State Vector (1 dimensional for test purposes)
        // x = [ X  ]
        //     [ dX ]
        // Initial state = [ 15 -1 ]^T
        RealVector x = new ArrayRealVector(new double[] {15, 0});

        // State Transition Matrix
        // delta T equals 1 seconds
        // A = [ 1 1 ]
        //     [0  1 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[][] {{1, 1},{0, 1}});

        // Control Input Matrix
        // No control input
        // B = [ ]
        RealMatrix B = null;

        // Measurement Matrix
        // Currently, only observing changes in position
        // H = [ 1 0 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[][] {{1, 0}, {0, 0}});

        // Process Noise Covariance Matrix
        // Q = [ PROCESS_NOISE_VARIANCE 0 ]
        //     [ 0 PROCESS_NOISE_VARIANCE ]
        RealMatrix Q = new Array2DRowRealMatrix(
                new double[][] {{PROCESS_NOISE_VARIANCE, 0}, {0, PROCESS_NOISE_VARIANCE}});

        // Measurement Noise Covariance Matrix
        // R = [ MEASUREMENT_NOISE_VARIANCE 0]
        //     [0 MEASUREMENT_NOISE_VARIANCE ]
        RealMatrix R = new Array2DRowRealMatrix(
                new double[][] {{MEASUREMENT_NOISE_VARIANCE, 0}, {0, MEASUREMENT_NOISE_VARIANCE}});

        // State Covariance Matrix (Error Covariance Matrix)
        // P = [ STATE_VARIANCE 0 ]
        //     [ 0 STATE_VARIANCE ]
        RealMatrix P = new Array2DRowRealMatrix(
                new double[][] {{STATE_VARIANCE, 0}, {0, STATE_VARIANCE}});;


        /** Build Process Model for Kalman Filter
         * Built using:
         * A - the state transition matrix,
         * B - the control input matrix - null in this case since there will be no control variable
         * Q - the process noise covariance matrix.
         * x - the state matrix
         * P - the state covariance matrix
         */
        ProcessModel process = new DefaultProcessModel(A, B, Q, x, P);

        /** Build Measurement Model for Kalman Filter
         * Built using:
         * H - the measurement matrix
         * R - the measurement noise covariance matrix
         */
        MeasurementModel measurement = new DefaultMeasurementModel(H, R);

        /** Build Kalman Filter */
        filter = new KalmanFilter(process, measurement);
    }

    public void predict() {
        filter.predict();
    }

    public void correct(double displacement) {
        RealVector z = new ArrayRealVector(new double[] {displacement, 0});
        filter.correct(z);
    }

    public void predict(ArrayRealVector u) {
        filter.predict(u);
    }

    public RealVector getStateEstimationVector() {
        return filter.getStateEstimationVector();
    }

    public RealMatrix getErrorCovarianceMatrix() {
        return filter.getErrorCovarianceMatrix();
    }

}
