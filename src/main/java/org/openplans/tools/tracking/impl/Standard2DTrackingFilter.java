package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.math.ComplexNumber;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.matrix.mtj.decomposition.EigenDecompositionRightMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

public class Standard2DTrackingFilter extends AbstractKalmanFilter {

  /**
   * 
   */
  private static final long serialVersionUID = -3818533301279461087L;

  /**
   * Motion model of the underlying system.
   */
  protected LinearDynamicalSystem model;

  private final double gVariance;
  private final double aVariance;
  private final double a0Variance;

  /*
   * If this value isn't null, then we have constrained motion.
   */
  private final Double angle;

  /*
   * Observation matrix
   */
  private static Matrix O;

  /**
   * Standard 2D tracking model with the following state equation: {@latex[ D_
   * x_t = G x_ t-1} + A \epsilon_t} Also, when angle != null, a constraint
   * matrix is created for the state covariance, with perpendicular variance
   * a0Variance.
   * 
   * @param gVariance
   * @param aVariance
   * @param a0Variance
   * @param angle
   */
  public Standard2DTrackingFilter(double gVariance, double aVariance,
    double a0Variance, Double angle) {

    super(VectorFactory.getDefault().createVector(4),
        createStateCovarianceMatrix(1d, aVariance, a0Variance, angle),
        MatrixFactory.getDefault().createIdentity(2, 2).scale(gVariance));

    this.aVariance = aVariance;
    this.gVariance = gVariance;
    this.a0Variance = a0Variance;
    this.angle = angle;

    final LinearDynamicalSystem model = new LinearDynamicalSystem(0, 4);

    final Matrix Gct = createStateTransitionMatrix(currentTimeDiff);
    final Matrix G = MatrixFactory.getDefault().createIdentity(4, 4);
    G.setSubMatrix(0, 0, Gct);

    model.setA(G);
    model.setB(MatrixFactory.getDefault().createMatrix(4, 4));
    model.setC(O);

    this.model = model;
  }

  static {
    O = MatrixFactory.getDefault().createMatrix(2, 4);
    O.setElement(0, 0, 1);
    O.setElement(1, 2, 1);
  }

  private double currentTimeDiff = 1d;

  private double prevTimeDiff = 1d;

  @Override
  public Standard2DTrackingFilter clone() {
    final Standard2DTrackingFilter filter = (Standard2DTrackingFilter) super
        .clone();
    return filter;
  }

  public void constrainMotion(Double angle) {
    this.setModelCovariance(createStateCovarianceMatrix(this.currentTimeDiff,
        this.aVariance, this.a0Variance, angle));
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    return new MultivariateGaussian(this.model.getState(),
        this.getModelCovariance());
  }

  public double getCurrentTimeDiff() {
    return currentTimeDiff;
  }

  public double getgVariance() {
    return gVariance;
  }

  @Override
  public void measure(MultivariateGaussian belief, Vector observation) {

    final Matrix C = this.model.getC();

    // Figure out what the model says the observation should be
    final Vector xpred = belief.getMean();
    final Vector ypred = C.times(xpred);

    // Update step... compute the difference between the observation
    // and what the model says.
    // Then compute the Kalman gain, which essentially indicates
    // how much to believe the observation, and how much to believe model
    final Vector innovation = observation.minus(ypred);
    this.computeMeasurementBelief(belief, innovation, C);

    // XXX covariance was set in the previous call
    // if (!checkPosDef((DenseMatrix)belief.getCovariance()))
    // return;

  }

  @Override
  public void predict(MultivariateGaussian belief) {
    /*
     * From KalmanFilter.class
     */
    // Load the belief into the model and then predict the next state
    this.model.evaluate(this.currentInput, belief.getMean());
    final Vector xpred = this.model.getState();

    // Calculate the covariance, which will increase due to the
    // inherent uncertainty of the model.
    if (currentTimeDiff != prevTimeDiff) {
      final Matrix modelCovariance = createStateCovarianceMatrix(
          currentTimeDiff, aVariance, a0Variance, angle);
      this.setModelCovariance(modelCovariance);
      final Matrix G = createStateTransitionMatrix(currentTimeDiff);
      this.model.setA(G);
    }
    final Matrix P = this.computePredictionCovariance(this.model.getA(),
        belief.getCovariance());

    // Load the updated belief
    belief.setMean(xpred);

    // if (!checkPosDef((DenseMatrix)P))
    // return;

    belief.setCovariance(P);

  }
  
  /**
   * Evaluates the predictive observation likelihood given the predictive state distribution.  
   * @param obs
   * @param belief
   * @return
   */
  public double predictiveLikelihood(Vector obs, MultivariateGaussian belief) {
    Matrix Q = belief.getCovariance();
    Q = this.model.getC().times( Q ).times( this.model.getC().transpose() );
    Q.plusEquals( this.measurementCovariance);
    MultivariateGaussian.PDF pdf = new MultivariateGaussian.PDF(
        this.model.getC().times(belief.getMean()), Q);
    return pdf.evaluate(obs);
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
  }

  public static boolean checkPosDef(DenseMatrix covar) {
    final EigenDecompositionRightMTJ decomp = EigenDecompositionRightMTJ
        .create(covar);
    for (final ComplexNumber eigenVal : decomp.getEigenValues()) {
      if (eigenVal.getRealPart() < 0)
        return false;
    }
    return true;
  }

  private static Matrix createStateCovarianceMatrix(double timeDiff,
    double aVariance, double a0Variance, Double angle) {
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(4, 2);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(3, 1, timeDiff);
    
    final Matrix Q = getVarianceConstraintMatrix(aVariance, a0Variance, angle);
    final Matrix A = A_half.times(Q).times(A_half.transpose());
    return A;
  }

  private static Matrix createStateTransitionMatrix(double timeDiff) {

    final Matrix Gct = MatrixFactory.getDefault().createIdentity(4, 4);
    Gct.setElement(0, 1, timeDiff);
    Gct.setElement(2, 3, timeDiff);

    return Gct;
  }

  public static Matrix getObservationMatrix() {
    return O;
  }

  private static Matrix getVarianceConstraintMatrix(double aVariance,
    double a0Variance, Double angle) {

    if (angle == null)
      return MatrixFactory.getDefault().createIdentity(2, 2).scale(aVariance);

    final Matrix rotationMatrix = MatrixFactory.getDefault().createIdentity(2,
        2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp = MatrixFactory.getDefault().createDiagonal(
        VectorFactory.getDefault().copyArray(
            new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(rotationMatrix.transpose());
  }

}
