package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultivariateGaussianMeanBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.Collection;
import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.distributions.AdjMultivariateGaussian;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.testng.internal.Nullable;

import com.google.common.base.Preconditions;

/**
 * This class encapsulates the motion model, i.e. on/off-road position and
 * velocity. Here we define predictions and updates to the motion state.
 * 
 * @author bwillard
 * 
 * @param <O>
 * @param <V>
 */
public class MotionStateEstimatorPredictor extends
    AbstractCloneableSerializable implements
    BayesianEstimatorPredictor<Vector, Vector, MultivariateGaussian>,
    IncrementalLearner<Vector, MultivariateGaussian> {

  /**
   * We allow {@value} meters of error when checking distance values on a path.
   */
  private static final double edgeLengthErrorTolerance = 1d;
  /**
   * Extracts the ground coordinates from a ground state.
   */
  public static Matrix Og;

  /**
   * Extracts the distance from a road state.
   */
  protected static Matrix Or;

  private static final long serialVersionUID = -3818533301279461087L;

  /**
   * This matrix converts (x, y, vx, vy) to (x, vx, y, vy).
   */
  public static Matrix U;

  /**
   * Extracts the velocity vector from a ground state.
   */
  public static Matrix Vg;

  /**
   * Extracts the velocity from a road state.
   */
  protected static Matrix Vr;

  public final static Vector zeros2D = VectorFactory.getDefault()
      .copyValues(0, 0);

  static {
    MotionStateEstimatorPredictor.Vg =
        MatrixFactory.getDenseDefault().copyArray(
            new double[][] { { 0, 1, 0, 0 }, { 0, 0, 0, 1 } });

    MotionStateEstimatorPredictor.Vr =
        MatrixFactory.getDenseDefault().copyArray(
            new double[][] { { 0, 1 } });

    MotionStateEstimatorPredictor.Og =
        MatrixFactory.getDefault().createMatrix(2, 4);
    MotionStateEstimatorPredictor.Og.setElement(0, 0, 1);
    MotionStateEstimatorPredictor.Og.setElement(1, 2, 1);

    MotionStateEstimatorPredictor.U =
        MatrixFactory.getDefault().createMatrix(4, 4);
    MotionStateEstimatorPredictor.U.setElement(0, 0, 1);
    MotionStateEstimatorPredictor.U.setElement(1, 2, 1);
    MotionStateEstimatorPredictor.U.setElement(2, 1, 1);
    MotionStateEstimatorPredictor.U.setElement(3, 3, 1);

    MotionStateEstimatorPredictor.Or =
        MatrixFactory.getDefault().createMatrix(1, 2);
    MotionStateEstimatorPredictor.Or.setElement(0, 0, 1);
  }

  protected static Matrix createStateCovarianceMatrix(
    double timeDiff, Matrix Q, boolean isRoad) {

    final Matrix A_half =
        MotionStateEstimatorPredictor.getCovarianceFactor(timeDiff,
            isRoad);
    final Matrix A = A_half.times(Q).times(A_half.transpose());

    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) A);

    return A;
  }

  protected static Matrix createStateTransitionMatrix(
    double timeDiff, boolean isRoad) {

    final int dim;
    if (isRoad) {
      dim = 2;
    } else {
      dim = 4;
    }
    final Matrix Gct =
        MatrixFactory.getDefault().createIdentity(dim, dim);
    Gct.setElement(0, 1, timeDiff);
    if (dim > 2) {
      Gct.setElement(2, 3, timeDiff);
    }

    return Gct;
  }

  public static Matrix getCovarianceFactor(double timeDiff,
    boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half =
        MatrixFactory.getDefault().createMatrix(dim * 2, dim);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }

    return A_half;
  }

  public static Matrix getCovarianceFactorLeftInv(double timeDiff,
    boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half =
        MatrixFactory.getDefault().createMatrix(dim, dim * 2);
    A_half.setElement(0, 0, 1d / Math.pow(timeDiff, 2));
    A_half.setElement(0, 1, 1d / (2d * timeDiff));
    //    A_half.setElement(0, 0, 2d * timeDiff/
    //        (Math.pow(timeDiff, 2) + 4d));
    //    A_half.setElement(0, 1, 4d/
    //        (Math.pow(timeDiff, 2) + 4d));
    if (dim == 2) {
      //      A_half.setElement(1, 2, 2d * timeDiff/
      //          (Math.pow(timeDiff, 2) + 4d));
      //      A_half.setElement(1, 3, 4d/
      //          (Math.pow(timeDiff, 2) + 4d));
      A_half.setElement(1, 2, 1d / Math.pow(timeDiff, 2));
      A_half.setElement(1, 3, 1d / (2d * timeDiff));
    }
    //    A_half.scale(1/timeDiff);

    return A_half;
  }

  public static double getEdgeLengthErrorTolerance() {
    return MotionStateEstimatorPredictor.edgeLengthErrorTolerance;
  }

  public static Matrix getGroundObservationMatrix() {
    return MotionStateEstimatorPredictor.Og;
  }

  public static Matrix getOg() {
    return MotionStateEstimatorPredictor.Og;
  }

  public static Matrix getOr() {
    return MotionStateEstimatorPredictor.Or;
  }

  public static Matrix getRoadObservationMatrix() {
    return MotionStateEstimatorPredictor.Or;
  }

  protected static Matrix getRotatedCovarianceMatrix(
    double aVariance, double a0Variance, double angle) {

    final Matrix rotationMatrix =
        MatrixFactory.getDefault().createIdentity(2, 2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp =
        MatrixFactory.getDefault().createDiagonal(
            VectorFactory.getDefault().copyArray(
                new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(
        rotationMatrix.transpose());
  }

  public static Matrix getU() {
    return MotionStateEstimatorPredictor.U;
  }

  public static Matrix getVg() {
    return MotionStateEstimatorPredictor.Vg;
  }

  public static Matrix getVr() {
    return MotionStateEstimatorPredictor.Vr;
  }

  public static Vector getZeros2d() {
    return MotionStateEstimatorPredictor.zeros2D;
  }

  protected VehicleStateDistribution<? extends GpsObservation> currentState;

  protected double currentTimeDiff = 0d;

  @Nonnull
  protected transient InferenceGraph graph;

  /**
   * The filter that applies movement and updates prior distributions for the
   * ground model.
   */
  protected AbstractKalmanFilter groundFilter;

  /**
   * State movement and observation model for the ground-state.
   */
  protected LinearDynamicalSystem groundModel;

  protected double prevTimeDiff = 0d;

  protected Random rng;

  /**
   * The filter that applies movement and updates prior distributions for the
   * road model.
   */
  protected AbstractKalmanFilter roadFilter;

  /*
   * Default value assumes that there's roughly 10 meters of
   * error in our determination of the road's actual location.
   */
  protected Matrix roadMeasurementError = MatrixFactory.getDefault()
      .copyArray(new double[][] { { 10 * 5, 0 }, { 0, 0 } });

  /**
   * State movement and observation model for the road-state.
   */
  protected LinearDynamicalSystem roadModel;

  /**
   * This estimator defines two dynamic linear models, or
   * 
   * @see <a
   *      href="Kalman Filters">http://en.wikipedia.org/wiki/Kalman_filter</a>,
   *      one for which the state equation is on-road and 2-dimensional
   *      (distance, velocity), and another that is off-road and 4-dimensional
   *      (2d location and velocity). The measurement equation for the on-road
   *      state models the error in road specification, e.g. error in edge
   *      locations or width. The measurement equation for the off-road state
   *      models the GPS error.
   * 
   * @param currentState
   * @param rng
   * @param currentTimeDiff
   */
  public MotionStateEstimatorPredictor(
    @Nonnull VehicleStateDistribution<?> currentState, @Nonnull Random rng,
    @Nullable Double currentTimeDiff) {

    this.currentState = currentState;
    this.graph = currentState.getGraph();
    this.rng = rng;

    if (currentState.getParentState() != null) {
      this.currentTimeDiff =
          (currentState.getObservation().getTimestamp().getTime() - currentState
              .getParentState().getObservation().getTimestamp()
              .getTime()) / 1000d;
    } else {
      this.currentTimeDiff =
          Preconditions.checkNotNull(currentTimeDiff);
    }

    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel =
        new LinearDynamicalSystem(0, 2);
    final Matrix roadG =
        MotionStateEstimatorPredictor.createStateTransitionMatrix(
            currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setC(MotionStateEstimatorPredictor.Or);
    this.roadModel = roadModel;

    this.roadFilter =
        new TruncatedRoadKalmanFilter(roadModel,
            MotionStateEstimatorPredictor
                .createStateCovarianceMatrix(currentTimeDiff,
                    currentState.getOnRoadModelCovarianceParam()
                        .getValue(), true), this.roadMeasurementError,
                        Double.MAX_VALUE, 0d);

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel =
        new LinearDynamicalSystem(0, 4);

    final Matrix groundGct =
        MotionStateEstimatorPredictor.createStateTransitionMatrix(
            currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setC(MotionStateEstimatorPredictor.Og);

    this.groundModel = groundModel;
    this.groundFilter =
        new TruncatedRoadKalmanFilter(groundModel,
            MotionStateEstimatorPredictor
                .createStateCovarianceMatrix(currentTimeDiff,
                    currentState.getOffRoadModelCovarianceParam()
                        .getValue(), false), currentState
                .getObservationCovarianceParam().getValue(),
                Double.MAX_VALUE, 0d);

  }

  @Override
  public MotionStateEstimatorPredictor clone() {
    final MotionStateEstimatorPredictor clone =
        (MotionStateEstimatorPredictor) super.clone();
    clone.currentTimeDiff = this.currentTimeDiff;
    clone.groundFilter = this.groundFilter.clone();
    clone.groundModel = this.groundModel.clone();
    clone.roadFilter = this.roadFilter.clone();
    clone.roadModel = this.roadModel.clone();
    clone.currentState = this.currentState.clone();
    clone.graph = ObjectUtil.cloneSmart(this.graph);

    return clone;
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    MultivariateGaussian initialMotionStateDist = this.groundFilter.createInitialLearnedObject();
    initialMotionStateDist.getMean().setElement(0, this.currentState.getObservation().getProjectedPoint().getElement(0));
    initialMotionStateDist.getMean().setElement(2, this.currentState.getObservation().getProjectedPoint().getElement(1));
    return initialMotionStateDist;
  }

  @Override
  public MultivariateGaussian createPredictiveDistribution(
    MultivariateGaussian prior) {
    final MultivariateGaussian priorPrediction = prior.clone();
    if (priorPrediction.getInputDimensionality() == 4) {
      this.groundFilter.predict(priorPrediction);
    } else {
      this.roadFilter.predict(priorPrediction);
    }
    return priorPrediction;

    //    PathStateDistribution newBelief;
    //    if (this.path.isNullPath()) {
    //      if (!prior.getPathState().isOnRoad()) {
    //        /*-
    //         * Predict free-movement
    //         */
    //        newBelief = prior.clone();
    //        groundFilter.predict(newBelief);
    //      } else {
    //        /*-
    //         * Going off-road
    //         */
    //        newBelief = new PathStateDistribution(this.path, prior.getGroundBelief().clone());
    //
    //        groundFilter.predict(newBelief);
    //      }
    //    } else {
    //
    //      if (!prior.getPathState().isOnRoad()) {
    //        /*-
    //         * Project a current location onto the path, then 
    //         * project movement along the path.
    //         */
    //        MultivariateGaussian localBelief = prior.getLocalStateBelief().clone();
    //        PathUtils.convertToRoadBelief(localBelief, this.path,
    //            Iterables.getFirst(this.path.getPathEdges(), null), true);
    //        newBelief = new PathStateDistribution(this.path, localBelief);
    //      } else {
    //        newBelief = new PathStateDistribution(this.path, prior);
    //      }
    //      roadFilter.predict(newBelief);
    //
    //      /*
    //       * Clamp to path
    //       */
    //      final double distance =
    //          MotionStateEstimatorPredictor.getOr()
    //              .times(newBelief.getMean()).getElement(0);
    //      if (!path.isOnPath(distance)) {
    //        newBelief.getMean().setElement(0, path.clampToPath(distance));
    //      }
    //    }
    //
    //    return newBelief;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final MotionStateEstimatorPredictor other =
        (MotionStateEstimatorPredictor) obj;
    if (Double.doubleToLongBits(this.currentTimeDiff) != Double
        .doubleToLongBits(other.currentTimeDiff)) {
      return false;
    }
    if (Double.doubleToLongBits(this.prevTimeDiff) != Double
        .doubleToLongBits(other.prevTimeDiff)) {
      return false;
    }
    return true;
  }

  public Matrix getCovarianceFactor(boolean isRoad) {
    return MotionStateEstimatorPredictor.getCovarianceFactor(
        this.currentTimeDiff, isRoad);
  }

  public Matrix getCovarianceFactorLeftInv(boolean isRoad) {
    return MotionStateEstimatorPredictor.getCovarianceFactorLeftInv(
        this.currentTimeDiff, isRoad);
  }

  public double getCurrentTimeDiff() {
    return this.currentTimeDiff;
  }

  public AbstractKalmanFilter getGroundFilter() {
    return this.groundFilter;
  }

  public LinearDynamicalSystem getGroundModel() {
    return this.groundModel;
  }

  /**
   * This method gets the predictive/observation/measurement belief, i.e. the
   * measurement model applied to the given state belief.
   * 
   * @param belief
   * @param edge
   * @return
   */
  public MultivariateGaussian getMeasurementBelief(
    final PathStateDistribution stateBelief) {
    final MultivariateGaussian projBelief =
        stateBelief.getGroundBelief().clone();

    final Matrix Q =
        MotionStateEstimatorPredictor.Og.times(
            projBelief.getCovariance()).times(
            MotionStateEstimatorPredictor.Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian res = 
        this.groundFilter.createInitialLearnedObject();
    res.setMean(MotionStateEstimatorPredictor.Og.times(projBelief
                .getMean()));
    res.setCovariance(Q);
    return res;
  }

  /**
   * Returns the distribution formed from the measurement equation, i.e. a
   * distribution in 2d location space.
   * 
   * @param motionState
   * @param edge
   * @return
   */
  public MultivariateGaussian getObservationDistribution(
    MultivariateGaussian motionState, PathEdge edge) {

    final MultivariateGaussian projBelief;
    final Matrix measurementCovariance =
        this.groundFilter.getMeasurementCovariance().clone();
    if (motionState.getInputDimensionality() == 2) {
      projBelief =
          PathUtils.getGroundBeliefFromRoad(motionState, edge, false, true);
    } else {
      projBelief = motionState;
    }

    final Matrix Q =
        MotionStateEstimatorPredictor.Og.times(
            projBelief.getCovariance()).times(
            MotionStateEstimatorPredictor.Og.transpose());
    Q.plusEquals(measurementCovariance);

    final MultivariateGaussian res = this.groundFilter.createInitialLearnedObject();
    res.setMean(MotionStateEstimatorPredictor.Og.times(projBelief
                .getMean()));
    res.setCovariance(Q);

    return res;
  }

  public double getPrevTimeDiff() {
    return this.prevTimeDiff;
  }

  public AbstractKalmanFilter getRoadFilter() {
    return this.roadFilter;
  }

  public LinearDynamicalSystem getRoadModel() {
    return this.roadModel;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(this.currentTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(this.prevTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public MultivariateGaussian
      learn(Collection<? extends Vector> data) {
    return null;
  }

  public Vector addStateTransitionError(Vector state, Random rng) {
    final int dim = state.getDimensionality();
    final Matrix cov =
        dim == 4 ? this.currentState.getOffRoadModelCovarianceParam()
            .getValue() : this.currentState
            .getOnRoadModelCovarianceParam().getValue();

    final Matrix covSqrt = StatisticsUtil.getCholR(cov);
    final Vector qSmpl =
        MultivariateGaussian.sample(VectorFactory.getDefault()
            .createVector(cov.getNumColumns()), covSqrt, rng);

    final Matrix covFactor = this.getCovarianceFactor(dim == 2);
    final Vector error = covFactor.times(qSmpl);
    final Vector stateSmpl = state.plus(error);
    return stateSmpl;
  }

  @Override
  public String toString() {
    return "StandardRoadTrackingFilter [ currentTimeDiff="
        + this.currentTimeDiff + "]";
  }

  @Override
  public void update(MultivariateGaussian target,
    Iterable<? extends Vector> data) {
    for (final Vector point : data) {
      this.update(target, point);
    }
  }

  /**
   * Updates the prior with the information from the observation. Note that the
   * 
   * @param priorPathStateBelief
   * @param observation
   * @param edge
   */
  @Override
  public void update(MultivariateGaussian prior, Vector data) {
    Preconditions.checkArgument(prior.getInputDimensionality() == 4);

    this.groundFilter.measure(prior, data);

  }

}
