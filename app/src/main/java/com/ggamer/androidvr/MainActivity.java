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
    private float[] mPreviousGyroscopeData = new float[3];

    private Quaternion fusedQuaternion = new Quaternion(.0f, 0.0f, 0.0f, 0.0f);
    private Quaternion previousFusedQuaternion = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

    private Quaternion invFusedQuaternion = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

    private Quaternion deltaQ;

    private boolean isOrientationInitialized = false;

    private Vector3 estimatedGravity;
    private Vector3 measuredGravity;

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
                measuredGravity = new Vector3(mAccelerometerData[0], mAccelerometerData[1], mAccelerometerData[2]);
                measuredGravity.normalize();
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
        SensorManager.getRotationMatrix(rotationMatrix, null, mAccelerometerData, mMagnetometerData);


        Quaternion accQuaternion;
        accQuaternion = rotationMatrixToQuaternion(rotationMatrix);

        Quaternion gyroQuaternion = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);


        if(timestamp != 0) {
            dt = (event.timestamp - timestamp) * NS2S;
            if(!isOrientationInitialized) {
                previousFusedQuaternion = accQuaternion;
                isOrientationInitialized = true;
            }
            Quaternion gyroDeltaQ = gyroToQuaternionDelta(mGyroscopeData, dt);

            fusedQuaternion = new Quaternion(previousFusedQuaternion);
            fusedQuaternion.multiply(gyroDeltaQ);

            invFusedQuaternion = new Quaternion(fusedQuaternion);
            invFusedQuaternion.inverse();

            estimatedGravity = new Vector3(0, 0, -1);
            estimatedGravity.applyQuaternion(invFusedQuaternion);
            estimatedGravity.normalize();

            if(measuredGravity == null) {
                measuredGravity = new Vector3(0, 0, -1);
            }
            deltaQ = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
            deltaQ.setFromUnitVectors(estimatedGravity, measuredGravity);
            deltaQ.inverse();

            Quaternion targetQ = new Quaternion(0.0f, 0.0f, 0.0f, 0.0f);
            targetQ = new Quaternion(fusedQuaternion);
            targetQ.multiply(deltaQ);



            fusedQuaternion.slerp(targetQ, 1.0f - 0.98f);

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
        mGyroscopeData[0] = mPreviousGyroscopeData[0];
        mGyroscopeData[1] = mPreviousGyroscopeData[1];
        mGyroscopeData[2] = mPreviousGyroscopeData[2];
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