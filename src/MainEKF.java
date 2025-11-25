import filters.EKF;
import math.Matrix;
import math.Vector;
import models.CTEKFModel;

public class MainEKF {

    public static void main(String[] args) {

        double dt = 1.0;

        // State: [x, y, vx, vy, omega]
        Vector x0 = new Vector(new double[] {
                1000.0, 0.0,     // starting position
                0.0,   50.0,     // velocity
                Math.toRadians(5.0)  // initial guess for omega
        });

        Matrix P0 = Matrix.identity(5).multiply(1000.0);

        CTEKFModel model = new CTEKFModel(0.5);

        // Measurement: x, y
        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0}
        });

        Matrix R = new Matrix(new double[][] {
                {25, 0},
                {0, 25}
        });

        EKF ekf = new EKF(x0, P0, model, H, R);

        // Separate truth state (so we're not chasing ourselves)
        Vector xTrue = new Vector(new double[] {
                1000.0, 0.0,
                0.0,   50.0,
                Math.toRadians(5.0)   // true omega
        });

        for (int k = 0; k < 30; k++) {

            // Propagate truth with the same model f (could also write a pure truth function)
            xTrue = model.f(xTrue, dt);
            double trueX = xTrue.get(0);
            double trueY = xTrue.get(1);

            // Noisy measurement
            double zX = trueX + (Math.random() * 10.0 - 5.0);
            double zY = trueY + (Math.random() * 10.0 - 5.0);
            Vector z = new Vector(new double[]{zX, zY});

            // EKF predict + update
            ekf.predict(dt);
            ekf.update(z);

            Vector xEst = ekf.getState();

            System.out.printf(
                "Step %2d | True=(%.2f, %.2f, ω=%.3f) | Meas=(%.2f, %.2f) | Est=(%.2f, %.2f, ω=%.3f)%n",
                k,
                trueX, trueY, xTrue.get(4),
                zX, zY,
                xEst.get(0), xEst.get(1), xEst.get(4)
            );
        }
    }
}
