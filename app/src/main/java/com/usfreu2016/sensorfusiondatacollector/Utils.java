package com.usfreu2016.sensorfusiondatacollector;

public class Utils {

    private final static float K = 0.53f;

    public static double computeStepLength(double max, double min) {
        double difference = max - min;
        return K * (float) Math.pow(difference, 1.0 / 4);
    }

}
