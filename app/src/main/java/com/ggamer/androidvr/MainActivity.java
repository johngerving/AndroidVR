package com.ggamer.androidvr;

import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.pm.ActivityInfo;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private SensorManager mSensorManager;

    private Sensor mSensorAccelerometer;
    private Sensor mSensorMagnetometer;
    private Sensor mSensorGyroscope;

    private TextView mTextSensorAzimuth;
    private TextView mTextSensorPitch;
    private TextView mTextSensorRoll;

    private static final float VALUE_DRIFT = 0.05f;

    private float[] mAccelerometerData = new float[3];
    private float[] mMagnetometerData = new float[3];
    private float[] mGyroscopeData = new float[3];

    private float fusedAzimuth = 0.0f;
    private float fusedPitch = 0.0f;
    private float fusedRoll = 0.0f;

    private float dt = 0.0f;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private final float[] deltaRotationVector = new float[4];
    private float timestamp;
    final float EPSILON = 0.000001f;

    private float[] gyroscopeOrientation = {0.0f, 0.0f, 0.0f};


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        mTextSensorAzimuth = (TextView) findViewById(R.id.value_azimuth);
        mTextSensorPitch = (TextView) findViewById(R.id.value_pitch);
        mTextSensorRoll = (TextView) findViewById(R.id.value_roll);

        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensorMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

    }

    @Override
    protected void onStart() {
        super.onStart();

        if(mSensorAccelerometer != null) {
            mSensorManager.registerListener(this, mSensorAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
        }

        if(mSensorMagnetometer != null) {
            mSensorManager.registerListener(this, mSensorMagnetometer, SensorManager.SENSOR_DELAY_NORMAL);
        }

        if(mSensorGyroscope != null) {
            mSensorManager.registerListener(this, mSensorGyroscope, SensorManager.SENSOR_DELAY_FASTEST);
        }
    }

    @Override
    protected void onStop() {
        super.onStop();

        mSensorManager.unregisterListener(this);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        int sensorType = event.sensor.getType();

        switch (sensorType) {
            case Sensor.TYPE_ACCELEROMETER:
                mAccelerometerData = event.values.clone();
                break;
            case Sensor.TYPE_MAGNETIC_FIELD:
                mMagnetometerData = event.values.clone();
                break;
            case Sensor.TYPE_GYROSCOPE:
                mGyroscopeData = event.values.clone();
                break;
            default:
                return;
        }

        float[] rotationMatrix = new float[9];
        boolean rotationOK = SensorManager.getRotationMatrix(rotationMatrix, null, mAccelerometerData, mMagnetometerData);

        float orientationValues[] = new float[3];
        if(rotationOK) {
            SensorManager.getOrientation(rotationMatrix, orientationValues);
        }

        float accAzimuth = orientationValues[0];
        float accPitch = orientationValues[1];
        float accRoll = orientationValues[2];

        // This timestep's delta rotation to be multiplied by the current rotation
        // after computing it from the gyro sample data.
//        if (timestamp != 0) {
//            final float dT = (event.timestamp - timestamp) * NS2S;
//            // Axis of the rotation sample, not normalized yet.
//            float axisX = mGyroscopeData[0];
//            float axisY = mGyroscopeData[0];
//            float axisZ = mGyroscopeData[0];
//
//            // Calculate the angular speed of the sample
//            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);
//
//            // Normalize the rotation vector if it's big enough to get the axis
//            // (that is, EPSILON should represent your maximum allowable margin of error)
//            if (omegaMagnitude > EPSILON) {
//                axisX /= omegaMagnitude;
//                axisY /= omegaMagnitude;
//                axisZ /= omegaMagnitude;
//            }
//
//            // Integrate around this axis with the angular speed by the timestep
//            // in order to get a delta rotation from this sample over the timestep
//            // We will convert this axis-angle representation of the delta rotation
//            // into a quaternion before turning it into the rotation matrix.
//            float thetaOverTwo = omegaMagnitude * dT / 2.0f;
//            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
//            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
//            deltaRotationVector[0] = sinThetaOverTwo * axisX;
//            deltaRotationVector[1] = sinThetaOverTwo * axisY;
//            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
//            deltaRotationVector[3] = cosThetaOverTwo;
//        }
//        timestamp = event.timestamp;
//        float[] deltaRotationMatrix = new float[9];
//        SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);
//
//        float[] gyroscopeOrientationVector = new float[3];
//
//        gyroscopeOrientationVector[0] = (float) (Math.cos(gyroscopeOrientation[0]) * Math.cos(gyroscopeOrientation[1]));
//        gyroscopeOrientationVector[1] = (float) (Math.sin(gyroscopeOrientation[0]) * Math.cos(gyroscopeOrientation[1]));
//        gyroscopeOrientationVector[2] = (float) (Math.sin(gyroscopeOrientation[1]));
//
//        float[] newGyroscopeOrientationVector = new float[3];
//
//        newGyroscopeOrientationVector = multiplyVectorByMatrix(gyroscopeOrientationVector, deltaRotationMatrix);
//        SensorManager.getRotationMatrixFromVector(newGyroscopeOrientationVector, deltaRotationMatrix);
//        SensorManager.getOrientation(deltaRotationMatrix, gyroscopeOrientation);
        // User code should concatenate the delta rotation we computed with the current rotation
        // in order to get the updated rotation.
        // rotationCurrent = rotationCurrent * deltaRotationMatrix;

        if(timestamp != 0) {
            dt = (event.timestamp - timestamp) * NS2S;
            if(fusedAzimuth == 0.0f && fusedPitch == 0.0f && fusedRoll == 0.0f) {
                fusedAzimuth = accAzimuth;
                fusedPitch = accPitch;
                fusedRoll = accRoll;
            } else {
                fusedAzimuth += Math.toDegrees(mGyroscopeData[0]) * dt;
                fusedPitch += Math.toDegrees(mGyroscopeData[1]) * dt;
                fusedRoll += Math.toDegrees(mGyroscopeData[2]) * dt;
            }

            int forceMagnitudeApprox = Math.round(Math.abs(accAzimuth) + Math.abs(accPitch) + Math.abs(accRoll));
            if(forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768) {


                accPitch = (float) (Math.atan2(accPitch, accRoll) * 180 / Math.PI);
                fusedPitch = (float) (fusedPitch * 0.98 + accPitch * 0.02);

                accRoll = (float) (Math.atan2(accAzimuth, accRoll) * 180 / Math.PI);
                fusedRoll = (float) (fusedRoll * 0.98 + accRoll * 0.02);
            }

            mTextSensorAzimuth.setText(getResources().getString(
                    R.string.value_format, fusedAzimuth));
            mTextSensorPitch.setText(getResources().getString(
                    R.string.value_format, fusedPitch));
            mTextSensorRoll.setText(getResources().getString(
                    R.string.value_format, fusedRoll));
        }
        timestamp = event.timestamp;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    public float[] multiplyVectorByMatrix(float[] vector, float[] matrix) {
        float[] result = new float[3];

        result[0] = matrix[0]*vector[0] + matrix[1]*vector[1] + matrix[2]*vector[2];
        result[1] = matrix[3]*vector[0] + matrix[4]*vector[1] + matrix[5]*vector[2];
        result[2] = matrix[6]*vector[0] + matrix[7]*vector[1] + matrix[8]*vector[2];

        return result;
    }
}