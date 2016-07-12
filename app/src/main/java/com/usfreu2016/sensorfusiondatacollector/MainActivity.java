package com.usfreu2016.sensorfusiondatacollector;

import android.Manifest;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Environment;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import com.estimote.sdk.SystemRequirementsChecker;
import com.estimote.sdk.Utils;
import com.estimote.sdk.eddystone.Eddystone;
import com.estimote.sdk.BeaconManager;
import com.estimote.sdk.repackaged.okhttp_v2_2_0.com.squareup.okhttp.internal.Util;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.w3c.dom.Text;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;
import java.util.List;

public class MainActivity extends AppCompatActivity implements SensorEventListener{

    /** Constant for permission checking */
    final int REQUEST_CODE_ASK_PERMISSIONS = 1;

    /** Threshold for step detection */
    final double STEP_THRESHOLD = 3.8;

    /** References for file writing */
    String dirName = "Sensor Fusion Data";
    BufferedWriter beaconDataWriter;
    BufferedWriter accelerationDataWriter;
    BufferedWriter stepsDataWriter;

    /** Flag to indicate data collection */
    boolean collectingData;

    /** References for beacon signal data collection */
    private BeaconManager beaconManager;
    String scanId;
    boolean scanning;
    boolean serviceReady;
    String beaconSignalData;
    Eddystone beacon;

    /** References for accelerometer data collection */
    SensorManager sensorManager;
    String accelerometerData;

    /** References for ground truth */
    int gtStepsTaken;

    /** References for step detection */
    StepDetector stepDetector;
    double maxAcc;
    double minAcc;
    long stepTime;
    final int MINIMUM_STEP_TIME = 1000;

    /** Kalman Filter */
    final double PROCESS_NOISE = 0.008;
    final double PROCESS_NOISE_VARIANCE = 0.0064;

    /** Measurement noise*/
    final double MEASUREMENT_NOISE = 3;
    final double MEASUREMENT_NOISE_VARIANCE = 40;

    /** State variance/covariance */
    final double STATE_VARIANCE = 500.0;
    KalmanFilter filter;

    /** States */
    double xGroundTruth;
    double xEstimate;
    double vEstimate;

    /** Views */
    Button startButton;
    Button stepButton;
    TextView stateEstimateTextView;
    TextView beaconDistanceTextView;
    TextView stepDistanceTextView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        /** Check write permissions */
        checkPermissions();

        /** Initially not collecting data */
        collectingData = false;

        /** Link references to views */
        startButton = (Button) findViewById(R.id.startButton);
        startButton.setOnClickListener(startButtonListener);
        stepButton = (Button) findViewById(R.id.stepButton);
        stepButton.setOnClickListener(stepButtonListener);
        stateEstimateTextView = (TextView) findViewById(R.id.stateEstimateTextView);
        beaconDistanceTextView = (TextView) findViewById(R.id.beaconDistanceTextView);
        stepDistanceTextView = (TextView) findViewById(R.id.stepDistanceTextView);

        /** Set up references for beacon signal data collection */
        beaconManager = new BeaconManager(getApplicationContext());
        beaconManager.setForegroundScanPeriod(500,0);
        beaconManager.setEddystoneListener(beaconListener);
        beaconManager.connect(new BeaconManager.ServiceReadyCallback() {
            @Override
            public void onServiceReady() {
                serviceReady = true;
            }
        });

        /** Set up references for accelerometer data collection */
        sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

        /** Set up step detection */
        stepDetector = new StepDetector(STEP_THRESHOLD);

        /** Set up Kalman Filter */
        //filter = new Filter();
        buildFilter();
        xEstimate = filter.getStateEstimationVector().toArray()[0];
        stateEstimateTextView.setText(String.format("%.2f", xEstimate));
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (collectingData) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_LINEAR_ACCELERATION:
                    float xA = event.values[0];
                    float yA = event.values[1];
                    float zA = event.values[2];
                    long time = Calendar.getInstance().getTimeInMillis();

                    if (zA > maxAcc) {
                        maxAcc = zA;
                    }
                    else if (zA < minAcc) {
                        minAcc = zA;
                    }

                    accelerometerData = String.format("%.3f,%.3f,%.3f,%d\n", xA, yA, zA, time);
                    recordAcceleration();
//
                    long currentTime = Calendar.getInstance().getTimeInMillis();
                    if (stepDetector.stepDetected(zA) && (currentTime - stepTime >= MINIMUM_STEP_TIME) && beacon != null) {
                        stepTime = currentTime;
                        stepDistanceTextView.setText(String.valueOf(stepDetector.getStepLength()));
//                        updatePosition(stepDetector.getStepLength());
                        updatePositionFromStep();
                    }
//                case Sensor.TYPE_STEP_DETECTOR:
//                    Log.d("onSensorChanged", "Step detector");
//                    long currentTime = Calendar.getInstance().getTimeInMillis();
//                    if (beacon != null && (currentTime - stepTime >= MINIMUM_STEP_TIME)) {
//                        stepDistanceTextView.setText(String.valueOf(StepDetector.computeStepLength(maxAcc, minAcc)));
//                        stepTime = currentTime;
//                        updatePosition();
//                        maxAcc = 0;
//                        minAcc = 0;
//                    }
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /** listener for Eddystone beacons */
    BeaconManager.EddystoneListener beaconListener = new BeaconManager.EddystoneListener() {
        @Override
        public void onEddystonesFound(List<Eddystone> list) {
            if (collectingData) {
                for (Eddystone eddystone : list) {
                    beacon = eddystone;
                    beaconDistanceTextView.setText("" + Utils.computeAccuracy(beacon));
                    String address = eddystone.macAddress.toStandardString();
                    int beaconRssi = eddystone.rssi;
                    long time = Calendar.getInstance().getTimeInMillis();

                    beaconSignalData = String.format("%s,%d,%d\n", address, beaconRssi, time);
//                    recordBeaconSignal();
//                    updatePosition(Utils.computeAccuracy(beacon));
                    updatePositionFromBeacon();
                }
            }
        }
    };

    /** Listener for start button */
    View.OnClickListener startButtonListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            if (startButton.getText().equals("Start")) {
                startCollection();
                startButton.setText("Stop");
            }
            else {
                stopCollection();
                startButton.setText("Start");
            }
        }
    };

    /** Listener for step button */
    View.OnClickListener stepButtonListener = new View.OnClickListener() {
        @Override
        public void onClick(View v) {
            if (beacon != null) {
//                updatePosition();
            }
            recordStep();
        }
    };

    /** Start scanning for Edddystone beacons */
    private void startScanning() {
        SystemRequirementsChecker.checkWithDefaultDialogs(this);
        scanId = beaconManager.startEddystoneScanning();
        scanning = true;
    }

    /** Stop scanning for Eddystone beacons */
    private void stopScanning() {
        beaconManager.stopEddystoneScanning(scanId);
        scanning = false;
    }

    /** Start collecting and recording accelerometer and beacon data */
    private void startCollection() {
        startScanning();
        sensorManager.registerListener(MainActivity.this,
                sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION),
                SensorManager.SENSOR_DELAY_FASTEST);
//        sensorManager.registerListener(MainActivity.this,
//                sensorManager.getDefaultSensor(Sensor.TYPE_STEP_DETECTOR),
//                SensorManager.SENSOR_DELAY_NORMAL);

        /** Initialize step time */
        stepTime = Calendar.getInstance().getTimeInMillis();

        /** Create filewriters */
        createFileWriters();

        /** Initially no steps taken */
        gtStepsTaken = 0;

        collectingData = true;
    }

    /** Stop collecting and recording accelerometer and beacon data */
    private void stopCollection() {
        stopScanning();
        sensorManager.unregisterListener(MainActivity.this);

        /** Close Filewriters */
        try {
            beaconDataWriter.close();
            accelerationDataWriter.close();
            stepsDataWriter.close();
        }
        catch (IOException e) {
            Log.e("Error", "IOException");
        }

        collectingData = false;
    }

    /** Record beacon signal strength */
    private void recordBeaconSignal() {
        try {
            beaconDataWriter.append(beaconSignalData);
        }
        catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    /** Record acceleration values */
    private void recordAcceleration() {
        try {
            accelerationDataWriter.append(accelerometerData);
        }
        catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    /** Record step taken */
    private void recordStep() {
        try {
            stepsDataWriter.append(String.format("%d,%d\n", gtStepsTaken,
                    Calendar.getInstance().getTimeInMillis()));
        }
        catch (IOException e) {
            Log.e("Error", "IOException");
        }
        gtStepsTaken++;
    }

    /** Create filewriters for data recording */
    private void createFileWriters() {
        File dir = new File(
                Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DCIM), dirName);
        if (!dir.exists()) {
            if (!dir.mkdirs()) {
                Log.e("Error", "Directory not created.");
            }
        }
        File beaconDataFile = new File(dir, "beaconData.csv");
        File accelerationDataFile = new File(dir, "accelerationData.csv");
        File stepsDataFile = new File(dir, "stepData.csv");
        try {
            FileWriter writer1 = new FileWriter(beaconDataFile, true);
            FileWriter writer2 = new FileWriter(accelerationDataFile, true);
            FileWriter writer3 = new FileWriter(stepsDataFile, true);
            beaconDataWriter = new BufferedWriter(writer1);
            accelerationDataWriter = new BufferedWriter(writer2);
            stepsDataWriter = new BufferedWriter(writer3);

            /** Write headers to files */
            writeHeaders();
        } catch (FileNotFoundException e) {
            Log.e("Error", "File not found exception");
        } catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    private void writeHeaders() {
        try {
            beaconDataWriter.append("Address,RSSI,Time\n");
            accelerationDataWriter.append("X,Y,Z,Time\n");
            stepsDataWriter.append("Step,Time\n");
        }
        catch (IOException e) {
            Log.e("Error", "IOException");
        }
    }

    private void updatePositionFromBeacon() {
        filter.predict();

        RealVector z = new ArrayRealVector(new double[] {Utils.computeAccuracy(beacon)});

        filter.correct(z);

        xEstimate = filter.getStateEstimationVector().toArray()[0];
        stateEstimateTextView.setText(String.format("%.2f", xEstimate));
    }

    private void updatePositionFromStep() {
        RealVector u = new ArrayRealVector(new double[] {-1.0});
        RealVector z = new ArrayRealVector(new double[] {xEstimate - stepDetector.getStepLength()});

        filter.predict(u);
        filter.correct(z);

        xEstimate = filter.getStateEstimationVector().toArray()[0];
        stateEstimateTextView.setText(String.format("%.2f", xEstimate));
    }

    /** update filter and get new position estimate */
    private void updatePosition(double measurement) {

        /** Std. Dev. of beacon signal and step length */
        final double BEACON_NOISE = 4.11;
        final double DR_NOISE = 20;

        /** Get displacements according to PM and DR */
        double pmMeasurement = Utils.computeAccuracy(beacon);
        double drMeasurement = xEstimate - StepDetector.computeStepLength(maxAcc, minAcc);

        /** Fuse displacements */
        final double m1 = pmMeasurement;
        final double m2 = drMeasurement;
        final double sigma1 = Math.pow(BEACON_NOISE, 2);
        final double sigma2 = Math.pow(DR_NOISE, 2);
        double fusedMeasurement = ((sigma2 * m1) + (sigma1 * m2)) / (sigma1 + sigma2);

//        /** LOGS for debugging */
//        Log.d("Logs", "pmMeasurement: " + pmMeasurement);
//        Log.d("Logs", "drMeasurement: " + drMeasurement);
//        Log.d("Logs", "fused estimate: " + fusedMeasurement);

        RealVector z = new ArrayRealVector(new double[] {measurement, 0});

        /** Run the Kalman Filter */
        filter.predict();
        filter.correct(z);
        xEstimate = filter.getStateEstimationVector().toArray()[0];
        stateEstimateTextView.setText(String.format("%.2f %.2f", xEstimate));
        Log.d("TAG", "State estimate: " + xEstimate + " " + vEstimate);
    }

    private void buildFilter() {
        /** Matrices and Vectors */

        // State Vector (1 dimensional for test purposes)
        // x = [ X  ]
        // Initial state = [ 15 ]
        RealVector x = new ArrayRealVector(new double[] {15});

        // State Transition Matrix
        // A = [ 1 0 ]
        RealMatrix A = new Array2DRowRealMatrix(new double[] {1});

        // Control Input Matrix
        // No control input
        // B = [ 0.5 ]
        RealMatrix B = new Array2DRowRealMatrix(new double[] {1});

        // Measurement Matrix
        // Currently, only observing changes in position
        // H = [ 1 0 ]
        RealMatrix H = new Array2DRowRealMatrix(new double[] {1});

        // Process Noise Covariance Matrix
        // Q = [ PROCESS_NOISE_VARIANCE ]
        RealMatrix Q = new Array2DRowRealMatrix(
                new double[] {PROCESS_NOISE_VARIANCE});

        // Measurement Noise Covariance Matrix
        // R = [ MEASUREMENT_NOISE_VARIANCE ]
        RealMatrix R = new Array2DRowRealMatrix(
                new double[] {MEASUREMENT_NOISE_VARIANCE});

        // State Covariance Matrix (Error Covariance Matrix)
        // P = [ STATE_VARIANCE ]
        RealMatrix P = new Array2DRowRealMatrix(
                new double[] {STATE_VARIANCE});


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

    /** Check permissions for file reading and writing */
    private void checkPermissions() {
        int hasWriteExternalStoragePermission = checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        if (hasWriteExternalStoragePermission != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
                    REQUEST_CODE_ASK_PERMISSIONS);
            return;
        }
    }

    /** Callback if permissions needed */
    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        switch (requestCode) {
            case REQUEST_CODE_ASK_PERMISSIONS:
                if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                    // Permission Granted
                }
                else {
                    Toast.makeText(this, "Permissions Denied", Toast.LENGTH_SHORT).show();
                }
                break;
            default:
                super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }
    }
}
