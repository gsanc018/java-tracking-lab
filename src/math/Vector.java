package math;

public class Vector {

    private final double[] data;

    public Vector(int size) {
        data = new double[size];
    }

    public Vector(double[] values) {
        data = values.clone();
    }

    public int size() {
        return data.length;
    }

    public double get(int i) {
        return data[i];
    }

    public void set(int i, double value) {
        data[i] = value;
    }

    public Vector add(Vector other) {
        Vector result = new Vector(size());
        for (int i = 0; i < size(); i++) {
            result.set(i, this.get(i) + other.get(i));
        }
        return result;
    }

    public Vector subtract(Vector other) {
        Vector result = new Vector(size());
        for (int i = 0; i < size(); i++) {
            result.set(i, this.get(i) - other.get(i));
        }
        return result;
    }
}
