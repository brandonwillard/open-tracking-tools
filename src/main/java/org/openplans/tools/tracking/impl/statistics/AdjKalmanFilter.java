package org.openplans.tools.tracking.impl.statistics;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.AbstractMTJMatrix;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrixFactoryMTJ;
import gov.sandia.cognition.math.matrix.mtj.DenseVectorFactoryMTJ;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;
import no.uib.cipr.matrix.DenseMatrix;
import no.uib.cipr.matrix.DenseVector;
import no.uib.cipr.matrix.UpperSPDDenseMatrix;

/**
 * This is an improved (computationally) filter based on the Sandia KalmanFilter
 * class.
 * 
 * @author bwillard
 * 
 */
public class AdjKalmanFilter extends AbstractKalmanFilter {

  private static final long serialVersionUID = 8046227346384488242L;

  /**
   * Default autonomous dimension, {@value} .
   */
  public static final int DEFAULT_DIMENSION = 1;

  /**
   * Motion model of the underlying system.
   */
  protected LinearDynamicalSystem model;

  /**
   * Creates a new instance of KalmanFilter
   */
  public AdjKalmanFilter() {
    this(DEFAULT_DIMENSION);
  }

  /**
   * Creates an autonomous, fully observable linear dynamical system with the
   * given dimensionality
   * 
   * @param dim
   *          Dimensionality of the LDS
   */
  public AdjKalmanFilter(int dim) {
    // Autonomous Dynamical System:
    // xn+1 = A*xn
    // yn+1 = xn+1
    // Also, we're using an identity for the model covariance and
    // the measurement covariance
    this(new LinearDynamicalSystem(MatrixFactory.getDefault()
        .createIdentity(dim, dim), MatrixFactory.getDefault()
        .createMatrix(dim, dim), MatrixFactory.getDefault()
        .createIdentity(dim, dim)), MatrixFactory.getDefault()
        .createIdentity(dim, dim), MatrixFactory.getDefault()
        .createIdentity(dim, dim));
  }

  /**
   * Creates a new instance of LinearUpdater
   * 
   * @param model
   *          Motion model of the underlying system.
   * @param modelCovariance
   *          Covariance associated with the system's model.
   * @param measurementCovariance
   *          Covariance associated with the measurements.
   */
  public AdjKalmanFilter(LinearDynamicalSystem model,
    Matrix modelCovariance, Matrix measurementCovariance) {
    super(VectorFactory.getDefault().createVector(
        model.getInputDimensionality()), modelCovariance,
        measurementCovariance);
    this.setModel(model);
  }

  @Override
  public AdjKalmanFilter clone() {
    final AdjKalmanFilter clone = (AdjKalmanFilter) super.clone();
    clone.setModel(ObjectUtil.cloneSafe(this.getModel()));
    return clone;
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    return new MultivariateGaussian(
        this.model.getState(), this.getModelCovariance());
  }

  /**
   * Getter for model
   * 
   * @return Motion model of the underlying system.
   */
  public LinearDynamicalSystem getModel() {
    return this.model;
  }

  @Override
  public void measure(MultivariateGaussian belief, Vector observation) {
    final Matrix C = this.model.getC();

    // Figure out what the model says the observation should be
    //    final Vector xpred = belief.getMean();
    //    final Vector ypred = C.times(xpred);

    // Update step... compute the difference between the observation
    // and what the model says.
    // Then compute the Kalman gain, which essentially indicates
    // how much to believe the observation, and how much to believe model
    //    final Vector innovation = observation.minus(ypred);
    //    this.computeMeasurementBelief(belief, innovation, C);

    final Vector a = belief.getMean();
    final Matrix R = belief.getCovariance();
    final Matrix Q = C.times(R).times(C.transpose())
        .plus(this.getMeasurementCovariance());
    /*
     * This is the source of one major improvement:
     * uses the solve routine for a positive definite matrix
     */
    final UpperSPDDenseMatrix Qspd = new UpperSPDDenseMatrix(
        ((AbstractMTJMatrix) Q).getInternalMatrix(), false);
    final no.uib.cipr.matrix.Matrix CRt = ((AbstractMTJMatrix) C
        .times(R.transpose())).getInternalMatrix();

    final DenseMatrix Amtj = new DenseMatrix(
        Qspd.numRows(), CRt.numColumns());
    Qspd.transSolve(CRt, Amtj);

    final DenseMatrix AtQt = new DenseMatrix(
        Amtj.numColumns(), Qspd.numRows());
    Amtj.transABmult(Qspd, AtQt);

    final DenseMatrix AtQtAMtj = new DenseMatrix(
        AtQt.numRows(), Amtj.numColumns());
    AtQt.mult(Amtj, AtQtAMtj);

    final Matrix AtQtA = ((DenseMatrixFactoryMTJ) MatrixFactory
        .getDenseDefault()).createWrapper(AtQtAMtj);

    final DenseVector e = new DenseVector(
        ((gov.sandia.cognition.math.matrix.mtj.DenseVector) observation
            .minus(C.times(a))).getArray(), false);

    final DenseVector AteMtj = new DenseVector(Amtj.numColumns());
    Amtj.transMult(e, AteMtj);
    final Vector Ate = ((DenseVectorFactoryMTJ) VectorFactory
        .getDenseDefault()).createWrapper(AteMtj);

    final Matrix CC = R.minus(AtQtA);
    final Vector m = a.plus(Ate);
    belief.setCovariance(CC);
    belief.setMean(m);
  }

  @Override
  public void predict(MultivariateGaussian belief) {
    // Load the belief into the model and then predict the next state
    this.getModel().evaluate(this.currentInput, belief.getMean());
    final Vector xpred = this.model.getState();

    // Calculate the covariance, which will increase due to the
    // inherent uncertainty of the model.
    final Matrix P = this.computePredictionCovariance(
        this.model.getA(), belief.getCovariance());

    // Load the updated belief
    belief.setMean(xpred);
    belief.setCovariance(P);
  }

  /**
   * Setter for model
   * 
   * @param model
   *          Motion model of the underlying system.
   */
  public void setModel(LinearDynamicalSystem model) {
    this.model = model;
  }

}
