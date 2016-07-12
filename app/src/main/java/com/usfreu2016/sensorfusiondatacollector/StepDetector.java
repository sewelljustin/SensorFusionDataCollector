package com.usfreu2016.sensorfusiondatacollector;

import android.app.Service;
import android.hardware.SensorManager;

public class StepDetector extends Service{

    /** References for step length and step detection */
    private double stepThreshold;
    private int steps;
    private double stepLength;
    private float maxAcc;
    private float minAcc;
    private boolean maxAccRecorded;
    private boolean startMinAccRecording;
    private boolean minAccRecorded;

    private final static float K = 0.53f;

    /** Constructor */
    public StepDetector(double threshold) {
        stepThreshold = threshold;
        steps = 0;
        reset();
    }

    public boolean stepDetected(float data) {
        // if max and min acc recorded and value greater than threshold, then step detected
        if (maxAccRecorded && minAccRecorded && (data > stepThreshold)) {
            stepLength = Utils.computeStepLength(maxAcc, minAcc);
            steps++;
            reset();
            return true;
        }

        if (data > stepThreshold) {

            // record max acceleration
            if (data > maxAcc) {
                maxAcc = data;
                maxAccRecorded = true;
            }
        }

        // check if slope of acc is negative
        if (maxAccRecorded && (data - maxAcc) < 0) {
            startMinAccRecording = true;
        }

        if (startMinAccRecording && data < minAcc) {
            minAcc = data;
            minAccRecorded = true;
        }

        return false;

    }

    public int getSteps() {
        return steps;
    }

    public double getStepLength() {
        return stepLength;
    }

    private void reset() {
        maxAcc = minAcc = 0;
        maxAccRecorded = startMinAccRecording = minAccRecorded = false;
    }

    public static double computeStepLength(double max, double min) {
        double difference = max - min;
        return K * (float) Math.pow(difference, 1.0 / 4);
    }

}
