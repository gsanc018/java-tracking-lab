import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CTModel;

public class Main3 {

    public static void main(String[] args) {

        double dt = 1.0;

        // State: [x, y, vx, vy]
        Vector x0 = new Vector(new double[] {
                1000.0, 0.0,   // starting at (1000, 0)
                0.0,    50.0   // speed 50 m/s, straight up initially
        });

        // Large initial covariance
        Matrix P0 = Matrix.identity(4).multiply(1000.0);

        // Turn rate, e.g. 5 degrees/sec
        double omega = Math.toRadians(5.0);
        CTModel model = new CTModel(0.5, omega);

        // Measurement: we measure x and y only
        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0},
                {0, 1, 0, 0}
        });

        // Measurement noise
        Matrix R = new Matrix(new double[][] {
                {25, 0},
                {0, 25}
        });

        KalmanFilter kf = new KalmanFilter(x0, P0, model, H, R);

        // Simulate 30 steps
        for (int k = 0; k < 30; k++) {

            // "Truth" here: we will just use the model's F to propagate truth too
            // so you can see a nice arc without building a separate truth simulator.
            // In a fancier version, you'd have a separate truth state and use the same CT equations.
            kf.predict(dt);

            // Grab the predicted state as our "truth" for now
            Vector predicted = kf.getState();
            double trueX = predicted.get(0);
            double trueY = predicted.get(1);

            // Generate noisy measurement from "truth"
            double zX = trueX + (Math.random() * 10.0 - 5.0);
            double zY = trueY + (Math.random() * 10.0 - 5.0);

            Vector z = new Vector(new double[]{zX, zY});

            // Update with measurement
            kf.update(z);

            Vector est = kf.getState();

            System.out.printf(
                    "Step %2d | True=(%.2f, %.2f) | Meas=(%.2f, %.2f) | Est=(%.2f, %.2f)%n",
                    k,
                    trueX, trueY,
                    zX, zY,
                    est.get(0), est.get(1)
            );
        }
    }
}
