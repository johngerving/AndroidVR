package com.ggamer.androidvr;

public class Vector3 {
    private float x;
    private float y;
    private float z;

    public Vector3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public float getX() {
        return x;
    }

    public void setX(float x) {
        this.x = x;
    }

    public float getY() {
        return y;
    }

    public void setY(float y) {
        this.y = y;
    }

    public float getZ() {
        return z;
    }

    public void setZ(float z) {
        this.z = z;
    }

    public void normalize() {
        float norm = (float) Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
        this.x = this.x / norm;
        this.y = this.y / norm;
        this.z = this.z / norm;
    }

    public float length() {
        float length = (float) Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
        return length;
    }

    public void applyQuaternion(Quaternion q) {
        float x = this.x, y = this.y, z = this.z;
		float qx = q.getI(), qy = q.getJ(), qz = q.getK(), qw = q.getW();

        // calculate quat * vector

		float ix = qw * x + qy * z - qz * y;
		float iy = qw * y + qz * x - qx * z;
		float iz = qw * z + qx * y - qy * x;
		float iw = - qx * x - qy * y - qz * z;

        // calculate result * inverse quat

        this.x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
        this.y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
        this.z = iz * qw + iw * - qz + ix * - qy - iy * - qx;
    }

    public float dot(Vector3 v) {
        float dot_product = this.x*v.getX() + this.y*v.getY() + this.z*v.getZ();
        return dot_product;
    }
}
