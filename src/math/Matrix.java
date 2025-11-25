package math;

public class Matrix {

    private final double[][] data;
    private final int rows;
    private final int cols;

    // Constructor: create empty matrix (all zeros)
    public Matrix(int rows, int cols) {
        this.rows = rows;
        this.cols = cols;
        this.data = new double[rows][cols];  // automatically filled with 0.0
    }

    // Constructor: create matrix from existing 2D array
    public Matrix(double[][] values) {
        this.rows = values.length;
        this.cols = values[0].length;
        this.data = new double[rows][cols];

        // Safe copy (so outside code can't modify our internal data)
        for (int r = 0; r < rows; r++) {
            System.arraycopy(values[r], 0, this.data[r], 0, cols);
        }
    }

    // Basic getters
    public int rows() {
        return rows;
    }

    public int cols() {
        return cols;
    }

    // Get and set an element in the matrix
    public double get(int r, int c) {
        return data[r][c];
    }

    public void set(int r, int c, double value) {
        data[r][c] = value;
    }

    // Multiply matrix by vector
    public Vector multiply(Vector v) {
        if (v.size() != this.cols) {
            throw new IllegalArgumentException("Matrix/Vector dimension mismatch");
        }

        Vector result = new Vector(this.rows);

        for (int r = 0; r < this.rows; r++) {
            double sum = 0.0;
            for (int c = 0; c < this.cols; c++) {
                sum += this.get(r, c) * v.get(c);  // important: multiply, not add
            }
            result.set(r, sum);
        }

        return result;
    }

    // OPTIONAL: multiply two matrices (useful later, but simple)
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Matrix/Matrix dimension mismatch");
        }

        Matrix result = new Matrix(this.rows, other.cols);

        for (int r = 0; r < this.rows; r++) {
            for (int c = 0; c < other.cols; c++) {
                double sum = 0.0;
                for (int k = 0; k < this.cols; k++) {
                    sum += this.get(r, k) * other.get(k, c);
                }
                result.set(r, c, sum);
            }
        }
        return result;
    }

    // Identity matrix builder
    public static Matrix identity(int size) {
        Matrix I = new Matrix(size, size);
        for (int i = 0; i < size; i++) {
            I.set(i, i, 1.0);
        }
        return I;
    }

    // Add two matrices
    public Matrix add(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix/Matrix dimension mismatch in add()");
        }

        Matrix result = new Matrix(this.rows, this.cols);
        for (int r = 0; r < this.rows; r++) {
            for (int c = 0; c < this.cols; c++) {
                result.set(r, c, this.get(r, c) + other.get(r, c));
            }
        }
        return result;
    }

    // Transpose of the matrix
    public Matrix transpose() {
        Matrix result = new Matrix(this.cols, this.rows);
        for (int r = 0; r < this.rows; r++) {
            for (int c = 0; c < this.cols; c++) {
                result.set(c, r, this.get(r, c));
            }
        }
        return result;
    }

    // Inverse for 1x1 or 2x2 matrices
    public Matrix inverse() {
        if (rows == 1 && cols == 1) {
            return new Matrix(new double[][] {
                    {1.0 / data[0][0]}
            });
        }

        if (rows == 2 && cols == 2) {
            double a = data[0][0];
            double b = data[0][1];
            double c = data[1][0];
            double d = data[1][1];

            double det = a*d - b*c;
            if (Math.abs(det) < 1e-9) {
                throw new IllegalArgumentException("Matrix not invertible");
            }

            double[][] inv = {
                    { d / det, -b / det },
                    { -c / det, a / det }
            };

            return new Matrix(inv);
        }

        throw new UnsupportedOperationException("Inverse only implemented for 1x1 or 2x2 matrices");
    }

    // multiply matrix by scalar
    public Matrix multiply(double scalar) {
        Matrix result = new Matrix(rows, cols);
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                result.set(r, c, scalar * this.get(r, c));
            }
        }
        return result;
    }



}
