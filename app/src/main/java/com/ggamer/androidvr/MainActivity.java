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
    // Create sensor manager and sensors - accelerometer, magnetometer, and gyroscope
    private SensorManager mSensorManager;

    private Sensor mSensorAccelerometer;
    private Sensor mSensorMagnetometer;
    private Sensor mSensorGyroscope;

    private TextView mTextSensorAzimuth;
    private TextView mTextSensorPitch;
    private TextView mTextSensorRoll;

    // Arrays to hold information from sensors
    private float[] mAccelerometerData = new float[3];
    private float[] mMagnetometerData = new float[3];
    private float[] mGyroscopeData = new float[3];
    private float[] mPreviousGyroscopeData = new float[3];

    private final float[] deltaRotationVector = new float[4];

    // Quaternions for current gyro and accel orientations
    Quaternion gyroQuaternion = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
    Quaternion accQuaternion;

    // Quaternion holding the current orientation; result of sensory fusion with complementary filter
    private Quaternion fusedQuaternion = new Quaternion(.0f, 0.0f, 0.0f, 0.0f);
    private Quaternion previousFusedQuaternion = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

    // Quaternion with the change in rotation determined by gyroscope measurements
    private Quaternion deltaQ;

    private boolean isOrientationInitialized = false;

    // Time since last measurement
    private float dt = 0.0f;

    private static final float NS2S = 1.0f / 1000000000.0f;
    private float timestamp;
    final float EPSILON = 0.000001f;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);

        mTextSensorAzimuth = (TextView) findViewById(R.id.value_azimuth);
        mTextSensorPitch = (TextView) findViewById(R.id.value_pitch);
        mTextSensorRoll = (TextView) findViewById(R.id.value_roll);

        // Initialize sensor manager and sensors
        mSensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        mSensorAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensorMagnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        mSensorGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

    }

    @Override
    protected void onStart() {
        super.onStart();

        // Check if sensors are available on device - if so, register sensor listener
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

        // Check if sensor readings are available - if so, copy them to respective array
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

        // Convert accelerometer and magnetometer arrays into rotation matrix
        float[] rotationMatrix = new float[9];
        SensorManager.getRotationMatrix(rotationMatrix, null, mAccelerometerData, mMagnetometerData);

        // Convert rotation matrix to quaternion
        accQuaternion = rotationMatrixToQuaternion(rotationMatrix);

        if(timestamp != 0) {
            // Determine time since last measurement was taken
            dt = (event.timestamp - timestamp) * NS2S;

            float axisX = mGyroscopeData[0];
            float axisY = mGyroscopeData[1];
            float axisZ = mGyroscopeData[2];

            float omegaMagnitude = (float) Math.sqrt(axisX*axisX + axisY*axisY + axisZ*axisZ);

            if(omegaMagnitude > EPSILON) {
                axisX /= omegaMagnitude;
                axisY /= omegaMagnitude;
                axisZ /= omegaMagnitude;
            }

            float thetaOverTwo = omegaMagnitude * dt / 2.0f;
            float sinThetaOverTwo = (float) Math.sin(thetaOverTwo);
            float cosThetaOverTwo = (float) Math.cos(thetaOverTwo);
            deltaRotationVector[0] = sinThetaOverTwo * axisX;
            deltaRotationVector[1] = sinThetaOverTwo * axisY;
            deltaRotationVector[2] = sinThetaOverTwo * axisZ;
            deltaRotationVector[3] = cosThetaOverTwo;

            float[] deltaRotationMatrix = new float[9];
            SensorManager.getRotationMatrixFromVector(deltaRotationMatrix, deltaRotationVector);

            Quaternion gyroDeltaQ = rotationMatrixToQuaternion(deltaRotationMatrix);

            fusedQuaternion.multiply(gyroDeltaQ);

            fusedQuaternion.slerp(accQuaternion, 0.03f);

            previousFusedQuaternion = new Quaternion(fusedQuaternion);

            float[] orientationEuler = new float[3];
            orientationEuler = quaternionToEuler(fusedQuaternion);

            mTextSensorAzimuth.setText(getResources().getString(
                    R.string.value_format, orientationEuler[0]));
            mTextSensorPitch.setText(getResources().getString(
                    R.string.value_format, orientationEuler[1]));
            mTextSensorRoll.setText(getResources().getString(
                    R.string.value_format, orientationEuler[2]));
        }
        timestamp = event.timestamp;
        for(int i = 0; i < 3; i++) {
            mPreviousGyroscopeData[i] = mGyroscopeData[i];
        }
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

    public Quaternion eulerToQuaternion(float yaw, float pitch, float roll) {
        float cy = (float) Math.cos(yaw * 0.5);
        float sy = (float) Math.sin(yaw * 0.5);
        float cp = (float) Math.cos(pitch * 0.5);
        float sp = (float) Math.sin(pitch * 0.5);
        float cr = (float) Math.cos(roll * 0.5);
        float sr = (float) Math.sin(roll * 0.5);

        float[] q = new float[4];
        q[0] = cr * cp * cy + sr * sp * sy;
        q[1] = sr * cp * cy - cr * sp * sy;
        q[2] = cr * sp * cy + sr * cp * sy;
        q[3] = cr * cp * sy - sr * sp * cy;

        Quaternion result = new Quaternion(q[0], q[1], q[2], q[3]);
        return result;
    }

    public float[] quaternionToEuler(Quaternion q) {
        float[] angles = new float[3];

        double sinr_cosp = 2 * (q.getW() * q.getI() + q.getJ() * q.getK());
        double cosr_cosp = 1 - 2 * (q.getI() * q.getI() + q.getJ() * q.getJ());
        angles[2] = (float) Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.getW() * q.getJ() - q.getK() * q.getI());
        if (Math.abs(sinp) >= 1) {
            angles[1] = (float) Math.copySign(Math.PI / 2, sinp); // use 90 degrees if out of range
        } else {
            angles[1] = (float) Math.asin(sinp);
        }
        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.getW() * q.getK() + q.getI() * q.getJ());
        double cosy_cosp = 1 - 2 * (q.getJ() * q.getJ() + q.getK() * q.getK());
        angles[0] = (float) Math.atan2(siny_cosp, cosy_cosp);

        for(int n = 0; n < 3; n++) {
            angles[n] = (float) Math.toDegrees(angles[n]);
        }

        return angles;
    }

    public Quaternion rotationMatrixToQuaternion(float[] m) {
        float tr = m[0] + m[4] + m[8];

        float qw;
        float qx;
        float qy;
        float qz;

        if(tr > 0) {
            float S = (float) (Math.sqrt(tr + 1.0)) * 2;
            qw = 0.25f * S;
            qx = (m[7] - m[5]) / S;
            qy = (m[2] - m[6]) / S;
            qz = (m[3] - m[1]) / S;
        } else if((m[0] > m[4]) && (m[0] > m[8])) {
            float S = (float) (Math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2);
            qw = (m[7] - m[5]) / S;
            qx = 0.25f * S;
            qy = (m[1] + m[3]) / S;
            qz = (m[2] + m[6]) / S;
        } else if(m[4] > m[8]) {
            float S = (float) (Math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2);
            qw = (m[2] - m[6]) / S;
            qx = (m[1] + m[3]) / S;
            qy = 0.25f * S;
            qz = (m[5] + m[7]) / S;
        } else {
            float S = (float) (Math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2);
            qw = (m[3] - m[1]) / S;
            qx = (m[2] + m[6]) / S;
            qy = (m[5] + m[6]) / S;
            qz = 0.25f * S;
        }

        Quaternion result = new Quaternion(qw, qx, qy, qz);
        return result;
    }

    public Quaternion gyroToQuaternionDelta(float[] gyro, float dt) {
        Quaternion quat = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
        Vector3 axis;
        Vector3 gyroVector = new Vector3(gyro[0], gyro[1], gyro[2]);
        axis = new Vector3(gyro[0], gyro[1], gyro[2]);
        axis.normalize();
        quat.setFromAxisAngle(axis, gyroVector.length() * dt);
        return quat;
    }

}