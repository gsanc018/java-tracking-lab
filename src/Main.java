import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CVModel;

public class Main {

    public static void main(String[] args) {

        double dt = 1.0;

        // Initial state: x, y, vx, vy
        Vector x0 = new Vector(new double[] {
                0.0,   // x
                0.0,   // y
                10.0,  // vx
                5.0    // vy
        });

        // Large initial uncertainty
        Matrix P0 = Matrix.identity(4).multiply(1000.0);

        // Motion model
        CVModel model = new CVModel(1.0);   // q = 1.0

        // Measurement model (measures x and y only)
        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });

        // Measurement noise (R = 4 meters^2 noise)
        Matrix R = new Matrix(new double[][] {
                {4, 0},
                {0, 4}
        });

        // Build the Kalman Filter
        KalmanFilter kf = new KalmanFilter(x0, P0, model, H, R);

        // Simulate 20 steps of CV motion with noisy measurements
        for (int i = 0; i < 20; i++) {

            // True position follows: x = 10*(i+1), y = 5*(i+1)
            double trueX = 10.0 * (i + 1);
            double trueY = 5.0 * (i + 1);

            // Noisy measurement
            double zX = trueX + (Math.random() * 4.0 - 2.0);
            double zY = trueY + (Math.random() * 4.0 - 2.0);

            Vector z = new Vector(new double[] {zX, zY});

            // KF predict + update
            kf.predict(dt);
            kf.update(z);

            // Get estimated state
            Vector est = kf.getState();

            System.out.printf(
                    "Step %2d | True=(%.2f, %.2f) | Measured=(%.2f, %.2f) | Estimated=(%.2f, %.2f)%n",
                    i,
                    trueX, trueY,
                    zX, zY,
                    est.get(0), est.get(1)
            );
        }
    }
}
