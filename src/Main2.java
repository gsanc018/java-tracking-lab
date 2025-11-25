import filters.KalmanFilter;
import math.Matrix;
import math.Vector;
import models.CAModel;

public class Main2 {

    public static void main(String[] args) {

        double dt = 1.0;

        // State: [x, y, vx, vy, ax, ay]
        Vector x0 = new Vector(new double[] {
                0.0,  0.0,   // position
                10.0, 5.0,   // velocity
                0.0,  0.0    // acceleration
        });

        // Large initial covariance
        Matrix P0 = Matrix.identity(6).multiply(1000.0);

        // CA motion model with process noise intensity q
        CAModel model = new CAModel(0.5);

        // Measurement matrix: measure x and y only
        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0, 0, 0},
                {0, 1, 0, 0, 0, 0}
        });

        // Measurement noise
        Matrix R = new Matrix(new double[][] {
                {4, 0},
                {0, 4}
        });

        KalmanFilter kf = new KalmanFilter(x0, P0, model, H, R);

        // Simulate 20 steps like CV
        for (int i = 0; i < 20; i++) {

            // True position for simple CV motion
            double trueX = 10.0 * (i + 1);
            double trueY = 5.0  * (i + 1);

            // Noisy measurement
            double zX = trueX + (Math.random() * 4.0 - 2.0);
            double zY = trueY + (Math.random() * 4.0 - 2.0);

            Vector z = new Vector(new double[]{zX, zY});

            kf.predict(dt);
            kf.update(z);

            Vector est = kf.getState();

            System.out.printf(
                    "Step %2d | True=(%.2f, %.2f) | Meas=(%.2f, %.2f) | Est=(%.2f, %.2f) | vel=(%.2f, %.2f) | acc=(%.2f, %.2f)%n",
                    i,
                    trueX, trueY,
                    zX, zY,
                    est.get(0), est.get(1),   // estimated x,y
                    est.get(2), est.get(3),   // vx, vy
                    est.get(4), est.get(5)    // ax, ay
            );
        }
    }
}
