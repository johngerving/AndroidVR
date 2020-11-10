package com.ggamer.androidvr;

public class Quaternion {
    private float w;
    private float i;
    private float j;
    private float k;

    private final float EPSILON = 0.000001f;

    public Quaternion(float w, float i, float j, float k) {
        this.w = w;
        this.i = i;
        this.j = j;
        this.k = k;
    }

    public Quaternion(Quaternion other) {
        this.w = other.getW();
        this.i = other.getI();
        this.j = other.getJ();
        this.k = other.getK();
    }

    public float getW() {
        return w;
    }

    public void setW(float w) {
        this.w = w;
    }

    public float getI() {
        return i;
    }

    public void setI(float i) {
        this.i = i;
    }

    public float getJ() {
        return j;
    }

    public void setJ(float j) {
        this.j = j;
    }

    public float getK() {
        return k;
    }

    public void setK(float k) {
        this.k = k;
    }

    public Quaternion multiply(Quaternion quat) {
        Quaternion product = new Quaternion(
                this.w*quat.getW() - this.i*quat.getI() - this.j*quat.getJ() - this.k - quat.getK(),
                this.w*quat.getI() + quat.getW()*this.i + this.j*quat.getK() - quat.getJ()*this.k,
                this.w*quat.getJ() + quat.getW()*this.j + this.k*quat.getI() - quat.getK()*this.i,
                this.w*quat.getK() + quat.getW()*this.k + this.i*quat.getJ() - quat.getI()*this.j);
        return product;
    }

    public float norm() {
        return (float) Math.sqrt(this.w*this.w + this.i*this.i + this.j*this.j + this.k*this.k);
    }

    public void normalize() {
        this.w = this.w / this.norm();
        this.i = this.i / this.norm();
        this.j = this.j / this.norm();
        this.k = this.k / this.norm();
    }

    public void inverse() {
        this.i = -this.i;
        this.j = -this.j;
        this.k = -this.k;

        this.w = this.w / (this.norm()*this.norm());
        this.i = this.i / (this.norm()*this.norm());
        this.j = this.j / (this.norm()*this.norm());
        this.k = this.k / (this.norm()*this.norm());
    }

    public void setFromAxisAngle(Vector3 axis, float angle) {
        float s = (float) Math.sin(angle/2);
        this.i = axis.getX() * s;
        this.j = axis.getY() * s;
        this.k = axis.getZ() * s;
        this.w = (float) Math.cos(angle/2);
    }

    public void setFromUnitVectors(Vector3 vFrom, Vector3 vTo) {
        vFrom.normalize();
        vTo.normalize();

        double EPS = 0.000001;

        float r = vFrom.dot(vTo) + 1.0f;

        if(r < EPS) {
            r = 0;

            if(Math.abs(vFrom.getX()) > Math.abs(vFrom.getZ())) {
                this.i = -vFrom.getY();
                this.j = vFrom.getX();
                this.k = 0.0f;
                this.w = r;
            } else {
                this.i = 0.0f;
                this.j = -vFrom.getZ();
                this.k = vFrom.getY();
                this.w = r;
            }
        } else {
            this.i = vFrom.getY() * vTo.getZ() - vFrom.getZ() * vTo.getY();
            this.j = vFrom.getZ() * vTo.getX() - vFrom.getX() * vTo.getZ();
            this.w = r;
        }
    }

    public void slerp(Quaternion qb, float t) {
        if(t == 0) {
            return;
        }
        if(t == 1) {
            return;
        }

        float x = this.i, y = this.j, z = this.k, w = this.w;

        float cosHalfTheta = w * qb.getW() + x * qb.getI() + y * qb.getJ() + z * qb.getK();

        if ( cosHalfTheta < 0 ) {

            this.w = - qb.getW();
            this.i = - qb.getI();
            this.j = - qb.getJ();
            this.k = - qb.getK();

            cosHalfTheta = - cosHalfTheta;

        } else {
            this.w = qb.getW();
            this.i = qb.getI();
            this.j = qb.getJ();
            this.k = qb.getK();
        }

        if ( cosHalfTheta >= 1.0 ) {

            this.w = w;
            this.i = x;
            this.j = y;
            this.k = z;

            return;
        }

		float sqrSinHalfTheta = (float) (1.0 - cosHalfTheta * cosHalfTheta);


        if ( sqrSinHalfTheta <= EPSILON ) {

			float s = 1 - t;
            this.w = s * w + t * this.w;
            this.i = s * x + t * this.i;
            this.j = s * y + t * this.j;
            this.k = s * z + t * this.k;

            this.normalize();
        }

		float sinHalfTheta = (float) Math.sqrt( sqrSinHalfTheta );
        //System.out.println("First: " + sinHalfTheta);
		float halfTheta = (float) Math.atan2( sinHalfTheta, cosHalfTheta );
        //System.out.println("Second: " + sinHalfTheta + " " + halfTheta);
		float ratioA = (float) (Math.sin( ( 1.0f - t ) * halfTheta ) / sinHalfTheta);
        //System.out.println("Third: " + sinHalfTheta + " " + halfTheta + " " + ratioA);
        float ratioB = (float) (Math.sin( t * halfTheta ) / sinHalfTheta);
        //System.out.println("Fourth: " + sinHalfTheta + " " + halfTheta + " " + ratioA + " " + ratioB);

        this.w = ( w * ratioA + this.w * ratioB );
        this.i = ( x * ratioA + this.i * ratioB );
        this.j = ( y * ratioA + this.j * ratioB );
        this.k = ( z * ratioA + this.k * ratioB );
    }

    public void copy(Quaternion q) {
        this.w = q.getW();
        this.i = q.getI();
        this.j = q.getJ();
        this.k = q.getK();
    }
}
