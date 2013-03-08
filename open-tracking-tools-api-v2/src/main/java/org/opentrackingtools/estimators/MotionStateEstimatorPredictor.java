package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.AbstractMatrix;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.conjugate.ConjugatePriorBayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultivariateGaussianMeanBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian.PDF;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.AdjMultivariateGaussian;
import org.opentrackingtools.distributions.PathEdgeDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;
import org.testng.internal.Nullable;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

/**
 * This class encapsulates the motion model, i.e. on/off-road position and velocity.
 * Here we define predictions and updates to the motion state. 
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

  public static class Parameter extends AbstractCloneableSerializable
    implements BayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> {
    
    protected MultivariateGaussianMeanBayesianEstimator.Parameter parameter;
    
    public Parameter(MultivariateGaussian conditional, MultivariateGaussian prior) {
      this.parameter = new MultivariateGaussianMeanBayesianEstimator.Parameter(conditional, prior);
    }

    @Override
    public MultivariateGaussian getConditionalDistribution() {
      return parameter.getConditionalDistribution();
    }

    @Override
    public void setValue(Vector value) {
      parameter.setValue(value);
    }

    @Override
    public Vector getValue() {
      return parameter.getValue();
    }

    @Override
    public String getName() {
      return parameter.getName();
    }

    @Override
    public MultivariateGaussian getParameterPrior() {
      return parameter.getParameterPrior();
    }

    @Override
    public void updateConditionalDistribution(Random random) {
      parameter.updateConditionalDistribution(random);
    }

  }

  protected VehicleState<? extends GpsObservation> currentState;
  protected Random rng;

  /**
   * Standard 2D tracking model with the following state equation: {@latex[ D_
   * x_t = G x_ t-1} + A \epsilon_t} Also, when angle != null, a constraint
   * matrix is created for the state covariance, with perpendicular variance
   * a0Variance. aVariance doubles as both the x and y variances for
   * free-motion.
   * 
   */
  public MotionStateEstimatorPredictor(@Nonnull VehicleState<?> currentState, @Nonnull Random rng, @Nullable Double currentTimeDiff) {
    
    this.graph = this.currentState.getGraph();
    this.currentState = currentState;
    this.rng = rng;
    
    if (currentState.getParentState() != null)
      this.currentTimeDiff = (currentState.getObservation().getTimestamp().getTime()
          - currentState.getParentState().getObservation().getTimestamp().getTime())/1000d;
    else
      this.currentTimeDiff = Preconditions.checkNotNull(currentTimeDiff);
    
    /*
     * Create the road-coordinates filter
     */
    final LinearDynamicalSystem roadModel =
        new LinearDynamicalSystem(0, 2);
    final Matrix roadG =
        createStateTransitionMatrix(currentTimeDiff, true);
    roadModel.setA(roadG);
    roadModel.setC(Or);
    this.roadModel = roadModel;
    
    this.roadFilter =
        new AdjKalmanFilter(roadModel, 
            createStateCovarianceMatrix(currentTimeDiff, 
                currentState.getOnRoadModelCovarianceParam().getValue(), true), 
            currentState.getObservationCovarianceParam().getValue());

    /*
     * Create the ground-coordinates filter
     */
    final LinearDynamicalSystem groundModel =
        new LinearDynamicalSystem(0, 4);

    final Matrix groundGct =
        createStateTransitionMatrix(currentTimeDiff, false);

    groundModel.setA(groundGct);
    groundModel.setC(Og);

    this.groundModel = groundModel;
    this.groundFilter =
        new AdjKalmanFilter(groundModel, 
            createStateCovarianceMatrix(currentTimeDiff, 
                currentState.getOffRoadModelCovarianceParam().getValue(), false), 
            currentState.getObservationCovarianceParam().getValue());
    
  }

  private static final long serialVersionUID = -3818533301279461087L;

  /**
   * We allow {@value} meters of error when checking distance values on a path.
   */
  private static final double edgeLengthErrorTolerance = 1d;

  /**
   * State movement and observation model for the ground-state.
   */
  protected LinearDynamicalSystem groundModel;

  /**
   * The filter that applies movement and updates prior distributions for the
   * ground model.
   */
  protected AbstractKalmanFilter groundFilter;

  /**
   * State movement and observation model for the road-state.
   */
  protected LinearDynamicalSystem roadModel;

  /**
   * The filter that applies movement and updates prior distributions for the
   * road model.
   */
  protected AbstractKalmanFilter roadFilter;


  /**
   * Extracts the ground coordinates from a ground state.
   */
  public static Matrix Og;

  /**
   * Extracts the distance from a road state.
   */
  protected static Matrix Or;

  /**
   * Extracts the velocity vector from a ground state.
   */
  public static Matrix Vg;

  /**
   * Extracts the velocity from a road state.
   */
  protected static Matrix Vr;

  /**
   * This matrix converts (x, y, vx, vy) to (x, vx, y, vy).
   */
  public static Matrix U;

  static {
    Vg =
        MatrixFactory.getDenseDefault().copyArray(
            new double[][] { { 0, 1, 0, 0 }, { 0, 0, 0, 1 } });

    Vr =
        MatrixFactory.getDenseDefault().copyArray(
            new double[][] { { 0, 1 } });

    Og = MatrixFactory.getDefault().createMatrix(2, 4);
    Og.setElement(0, 0, 1);
    Og.setElement(1, 2, 1);

    U = MatrixFactory.getDefault().createMatrix(4, 4);
    U.setElement(0, 0, 1);
    U.setElement(1, 2, 1);
    U.setElement(2, 1, 1);
    U.setElement(3, 3, 1);

    Or = MatrixFactory.getDefault().createMatrix(1, 2);
    Or.setElement(0, 0, 1);
  }

  protected double currentTimeDiff = 0d;

  protected double prevTimeDiff = 0d;

  public final static Vector zeros2D = VectorFactory.getDefault()
      .copyValues(0, 0);

  @Nonnull
  protected transient InferenceGraph graph;

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

    // XXX no cloning here
    clone.graph = this.graph;

    return clone;
  }

  public Matrix getCovarianceFactor(boolean isRoad) {
    return getCovarianceFactor(this.currentTimeDiff, isRoad);
  }

  public Matrix getCovarianceFactorLeftInv(boolean isRoad) {
    return getCovarianceFactorLeftInv(this.currentTimeDiff, isRoad);
  }

  public double getCurrentTimeDiff() {
    return currentTimeDiff;
  }

  public AbstractKalmanFilter getGroundFilter() {
    return groundFilter;
  }

  public LinearDynamicalSystem getGroundModel() {
    return groundModel;
  }

  public double getPrevTimeDiff() {
    return prevTimeDiff;
  }

  public AbstractKalmanFilter getRoadFilter() {
    return roadFilter;
  }

  public LinearDynamicalSystem getRoadModel() {
    return roadModel;
  }

  /**
   * This method gets the predictive/observation/measurement belief, i.e. the
   * measurement model applied to the given state belief.
   * 
   * @param belief
   * @param edge
   * @return
   */
  public MultivariateGaussian getObservationBelief(
    final PathStateDistribution stateBelief) {
    final MultivariateGaussian projBelief =
        stateBelief.getGroundBelief().clone();

    final Matrix Q =
        Og.times(projBelief.getCovariance()).times(Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian res =
        new AdjMultivariateGaussian(Og.times(projBelief.getMean()), Q);
    return res;
  }

  /**
   * Updates the 
   * 
   * @param priorPathStateBelief
   * @param observation
   * @param edge
   */
  @Override
  public void update(
    MultivariateGaussian prior, Vector observation) {

    if (prior.getInputDimensionality() == 2) {
//      final MultivariateGaussian obsProj =
//          PathUtils.getRoadObservation(observation, this.roadFilter.getMeasurementCovariance(),
//              prior.getPathState().getPath(), priorPathStateBelief.getPathState().getEdge());
//
//      this.roadFilter.setMeasurementCovariance(obsProj
//          .getCovariance());
//
//      /*
//       * Clamp the projected obs
//       */
//      if (!priorPathStateBelief.getPathState().getPath().isOnPath(
//          obsProj.getMean().getElement(0))) {
//        obsProj.getMean().setElement(
//            0,
//            priorPathStateBelief.getPathState().getPath().clampToPath(
//                obsProj.getMean().getElement(0)));
//      }

      this.roadFilter.measure(prior, observation);

    } else {
      this.groundFilter.measure(prior, observation);
    }
  }

  @Override
  public MultivariateGaussian createInitialLearnedObject() {
    return this.groundFilter.createInitialLearnedObject();
  }

  @Override
  public MultivariateGaussian createPredictiveDistribution(MultivariateGaussian  prior) {
    MultivariateGaussian priorPrediction = prior.clone();
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
  public String toString() {
    return "StandardRoadTrackingFilter [ currentTimeDiff=" + currentTimeDiff + "]";
  }

  public Vector sampleStateTransDist(Vector state, Random rng) {
    final int dim = state.getDimensionality();
    final Matrix cov = dim == 4 ? this.currentState.getOffRoadModelCovarianceParam().getValue() : 
      this.currentState.getOnRoadModelCovarianceParam().getValue();

    final Matrix covSqrt = StatisticsUtil.getCholR(cov);
    final Vector qSmpl =
        MultivariateGaussian.sample(VectorFactory.getDefault()
            .createVector(cov.getNumColumns()), covSqrt, rng);

    final Matrix covFactor = this.getCovarianceFactor(dim == 2);
    final Vector error = covFactor.times(qSmpl);
    final Vector stateSmpl = state.plus(error);
    return stateSmpl;
  }

  protected static Matrix createStateCovarianceMatrix(
    double timeDiff, Matrix Q, boolean isRoad) {

    final Matrix A_half = getCovarianceFactor(timeDiff, isRoad);
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
    if (dim > 2)
      Gct.setElement(2, 3, timeDiff);

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

  public static Matrix getGroundObservationMatrix() {
    return Og;
  }

  public static Matrix getOg() {
    return Og;
  }

  public static Matrix getOr() {
    return Or;
  }

  public static Matrix getRoadObservationMatrix() {
    return Or;
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
    return U;
  }

  public static Vector getZeros2d() {
    return zeros2D;
  }

  public static double getEdgeLengthErrorTolerance() {
    return edgeLengthErrorTolerance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    long temp;
    temp = Double.doubleToLongBits(currentTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    temp = Double.doubleToLongBits(prevTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final MotionStateEstimatorPredictor other =
        (MotionStateEstimatorPredictor) obj;
    if (Double.doubleToLongBits(currentTimeDiff) != Double
        .doubleToLongBits(other.currentTimeDiff)) {
      return false;
    }
    if (Double.doubleToLongBits(prevTimeDiff) != Double
        .doubleToLongBits(other.prevTimeDiff)) {
      return false;
    }
    return true;
  }

  public static Matrix getVg() {
    return Vg;
  }

  public static Matrix getVr() {
    return Vr;
  }

  @Override
  public MultivariateGaussian 
      learn(Collection<? extends Vector> data) {
    return null;
  }

  @Override
  public void update(MultivariateGaussian target,
    Iterable<? extends Vector> data) {
    for (Vector point : data) {
      this.update(target, point);
    }
  }

}
