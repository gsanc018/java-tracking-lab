import filters.EKF;
import math.Matrix;
import math.Vector;
import models.CTEKFModel;

public class MainCTEKF {

    public static void main(String[] args) {

        double dt = 1.0;

        // True turn-rate (rad/s)
        double omegaTrue = Math.toRadians(5.0);

        // Motion model with some process noise
        CTEKFModel model = new CTEKFModel(0.5);

        // State: [x, y, vx, vy, omega]
        Vector x0 = new Vector(new double[] {
                1000.0, 0.0,       // x, y
                0.0,   50.0,       // vx, vy
                Math.toRadians(2.0) // initial guess for omega (wrong on purpose)
        });

        Matrix P0 = Matrix.identity(5).multiply(500.0);

        // We measure x and y only
        Matrix H = new Matrix(new double[][] {
                {1, 0, 0, 0, 0},
                {0, 1, 0, 0, 0}
        });

        Matrix R = new Matrix(new double[][] {
                {25, 0},
                {0, 25}
        });

        EKF ekf = new EKF(x0, P0, model, H, R);

        // Separate "truth" state, same form: [x, y, vx, vy, omega]
        Vector xTrue = new Vector(new double[] {
                1000.0, 0.0,
                0.0,   50.0,
                omegaTrue
        });

        for (int k = 0; k < 30; k++) {

            // Propagate truth using the same f (you could also write a pure truth function)
            xTrue = model.f(xTrue, dt);
            double trueX = xTrue.get(0);
            double trueY = xTrue.get(1);
            double trueOmega = xTrue.get(4);

            // Noisy measurement of x,y
            double zX = trueX + noise(5.0);
            double zY = trueY + noise(5.0);
            Vector z = new Vector(new double[]{zX, zY});

            // EKF predict + update
            ekf.predict(dt);
            ekf.update(z);

            Vector xEst = ekf.getState();

            System.out.printf(
                    "Step %2d | True=(x=%.2f, y=%.2f, ω=%.3f) | Est=(x=%.2f, y=%.2f, ω=%.3f)%n",
                    k,
                    trueX, trueY, trueOmega,
                    xEst.get(0), xEst.get(1), xEst.get(4)
            );
        }
    }

    private static double noise(double scale) {
        return (Math.random() - 0.5) * scale;
    }
}
