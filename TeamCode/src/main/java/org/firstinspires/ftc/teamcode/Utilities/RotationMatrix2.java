package org.firstinspires.ftc.teamcode.Utilities;

public class RotationMatrix2 {

    public double [][] rawMatrix;

    public RotationMatrix2(){
        rawMatrix = new double[][]{{1.0, 0.0}, {0.0, 1.0}};
    }

    public RotationMatrix2(double angle){
        setFromAngle(angle);
    }

    public RotationMatrix2(double[][] raw){
        rawMatrix = raw;
    }

    /**
     * Returns vector after this matrix is applied to it
     *
     * @param vector the vector that will be translated
     * @return vector
     */
    public Vector2 multiply(Vector2 vector){
        double[][] m1 = rawMatrix;
        return new Vector2(m1[0][0]*vector.x+m1[0][1]*vector.y,m1[1][0]*vector.x+m1[1][1]*vector.y);
    }

    /**
     * Returns vector after this matrix is applied to it
     *
     * @param x the x value of the vector that will be translated
     * @param y the y value of the vector that will be translated
     * @return vector
     */
    public Vector2 multiply(int x, int y){
        double[][] m1 = rawMatrix;
        return new Vector2(m1[0][0]*x+m1[0][1]*y,m1[1][0]*x+m1[1][1]*y);
    }

    /**
     * Returns new matrix, the product of this * matrix
     *
     * @param matrix the matrix this will be multiplied to
     * @return product of this * matrix
     */
    public RotationMatrix2 multiply(RotationMatrix2 matrix){
        double[][] m1 = rawMatrix;
        double[][] m2 = matrix.rawMatrix;
        double[][] resultRaw = {{m1[0][0]*m2[0][0] + m1[0][1]*m2[1][0],m1[0][0]*m2[0][1] + m1[0][1]*m2[1][1]},{m1[1][0]*m2[0][0] + m1[1][1]*m2[1][0],m1[1][0]*m2[0][1] + m1[1][1]*m2[1][1]}};
        return new RotationMatrix2(resultRaw);
    }

    /**
     * Multiplies this by scalar
     *
     * @param factor the factor this is multiplied by
     * @return this
     */
    public RotationMatrix2 multiplyScalar(double factor){
        rawMatrix[0][0] *= factor;
        rawMatrix[0][1] *= factor;
        rawMatrix[1][0] *= factor;
        rawMatrix[1][1] *= factor;
        return this;
    }

    /**
     * Returns inverse of this matrix
     *
     * @return new matrix, inverse of this
     */
    public RotationMatrix2 inverse(){
        double[][] m1 = rawMatrix;
        double determinant = m1[0][0] * m1[1][1] - (m1[1][0]*m1[0][1]);
        double[][] rawInverse = {{m1[1][1],-m1[0][1]},{m1[0][0],-m1[1][0]}};
        return new RotationMatrix2(rawInverse).multiplyScalar(determinant);
    }

    /**
     * Sets this matrix based on angle (radians)
     *
     * @param angle the angle this matrix will be set from
     * @return this
     */
    public RotationMatrix2 setFromAngle(double angle){
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        rawMatrix = new double[][]{{cos,-1 * sin},{sin,cos}};
        return this;
    }
    /**
     * Makes this matrix lookAt the vector, upvector is left
     * first row of matrix is set to vector,
     * second row of matrix is set to (-y,x) of vector, the same as the angle + pi/2 radians
     *
     * @param vector vector to look at
     * @return sets this matrix to look at the vector
     */
    public RotationMatrix2 lookAt(Vector2 vector){
        Vector2 unit = vector.unit();
        rawMatrix[0][0] = unit.x;
        rawMatrix[0][1] = unit.y;
        rawMatrix[1][0] = -unit.y;
        rawMatrix[1][1] = unit.x;
        return this;
    }

    /**
     * Returns the angle of this matrix
     *
     * @return angle of the lookVector(1,0) of this matrix
     */
    public double getAngle(){
        Vector2 lookVector = multiply(new Vector2(1,0));
        return Math.atan2(lookVector.y, lookVector.x);
    }
}
