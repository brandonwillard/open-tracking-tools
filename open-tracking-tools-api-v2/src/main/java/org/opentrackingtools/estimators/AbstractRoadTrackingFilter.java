package org.opentrackingtools.estimators;

import gov.sandia.cognition.math.matrix.AbstractMatrix;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Random;

import javax.annotation.Nonnull;

import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.AdjMultivariateGaussian;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.InferredPath;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathStateBelief;
import org.opentrackingtools.paths.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public abstract class AbstractRoadTrackingFilter extends AbstractCloneableSerializable implements
    Comparable<AbstractRoadTrackingFilter> {

  protected AbstractRoadTrackingFilter(GpsObservation obs, InferenceGraph graph, VehicleStateInitialParameters params,
    Random rng) {
    this.graph = graph;
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
   * Instantaneous road-state transition error covariance in units of
   * acceleration.
   */
  private Matrix Qr;

  /**
   * Instantaneous ground-state transition error covariance in units of
   * acceleration.
   */
  private Matrix Qg;

  /**
   * Instantaneous road-state transition error covariance.
   */
  private Matrix onRoadStateTransCovar;

  /**
   * Instantaneous ground-state transition error covariance.
   */
  private Matrix offRoadStateTransCovar;

  /**
   * Instantaneous gps error covariance.
   */
  protected Matrix obsCovar;

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
    Vg = MatrixFactory.getDenseDefault().copyArray(new double[][] { { 0, 1, 0, 0 }, { 0, 0, 0, 1 } });

    Vr = MatrixFactory.getDenseDefault().copyArray(new double[][] { { 0, 1 } });

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

  public final static Vector zeros2D = VectorFactory.getDefault().copyValues(0, 0);

  @Nonnull
  protected transient InferenceGraph graph;

  @Override
  public AbstractRoadTrackingFilter clone() {
    final AbstractRoadTrackingFilter clone = (AbstractRoadTrackingFilter) super.clone();
    clone.currentTimeDiff = this.currentTimeDiff;
    clone.groundFilter = this.groundFilter.clone();
    clone.groundModel = this.groundModel.clone();
    clone.obsCovar = this.obsCovar.clone();
    clone.offRoadStateTransCovar = this.offRoadStateTransCovar.clone();
    clone.onRoadStateTransCovar = this.onRoadStateTransCovar.clone();
    clone.prevTimeDiff = this.prevTimeDiff;
    clone.Qg = this.Qg.clone();
    clone.Qr = this.Qr.clone();
    clone.roadFilter = this.roadFilter.clone();
    clone.roadModel = this.roadModel.clone();

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

  public Matrix getObsCovar() {
    return obsCovar;
  }

  public Matrix getOffRoadStateTransCovar() {
    return offRoadStateTransCovar;
  }

  public Matrix getOnRoadStateTransCovar() {
    return onRoadStateTransCovar;
  }

  public double getPrevTimeDiff() {
    return prevTimeDiff;
  }

  public Matrix getQg() {
    return Qg;
  }

  public Matrix getQr() {
    return Qr;
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
  public MultivariateGaussian getObservationBelief(final PathStateBelief stateBelief) {
    final MultivariateGaussian projBelief = stateBelief.getGroundBelief().clone();

    final Matrix Q = Og.times(projBelief.getCovariance()).times(Og.transpose());
    Q.plusEquals(this.groundFilter.getMeasurementCovariance());

    final MultivariateGaussian res = new AdjMultivariateGaussian(Og.times(projBelief.getMean()), Q);
    return res;
  }

  /**
   * Updates the road-coordinates prior predictive belief to the posterior for
   * the given observation, edge and path distance to the start of the edge.
   * 
   * @param priorPathStateBelief
   * @param observation
   * @param edge
   */
  public PathStateBelief measure(PathStateBelief priorPathStateBelief, Vector observation, PathEdge edge) {

    final MultivariateGaussian updatedBelief;
    final PathStateBelief result;
    if (priorPathStateBelief.isOnRoad()) {
      //      /*
      //       * TODO FIXME: should probably snap observation to road 
      //       * and then filter, no?
      //       * 
      //       * Convert road-coordinates prior predictive to ground-coordinates
      //       */
      //      updatedBelief = belief.getGroundBelief().clone();
      //
      //      this.groundFilter.measure(updatedBelief, observation);
      //
      //      /*
      //       * Convert back to road-coordinates
      //       */
      //      convertToRoadBelief(updatedBelief, belief.getPath(),
      //          edge, true);
      final MultivariateGaussian obsProj =
          PathUtils.getRoadObservation(observation, this.obsCovar, priorPathStateBelief.getPath(), edge);
      //              Iterables.getLast(belief.getPath().getEdges()));

      this.roadFilter.setMeasurementCovariance(obsProj.getCovariance());
      updatedBelief = priorPathStateBelief.getGlobalStateBelief().clone();

      /*
       * Clamp the projected obs
       */
      if (!priorPathStateBelief.getPath().isOnPath(obsProj.getMean().getElement(0))) {
        obsProj.getMean().setElement(0, priorPathStateBelief.getPath().clampToPath(obsProj.getMean().getElement(0)));
      }

      this.roadFilter.measure(updatedBelief, obsProj.getMean());

      final PathStateBelief tmpBelief = priorPathStateBelief.getPath().getStateBeliefOnPath(updatedBelief);

      /*
       * Use the path actually traveled.
       * TODO: create a "getSubpath" in InferredPath
       */
      result = tmpBelief.getTruncatedPathStateBelief();

    } else {
      updatedBelief = priorPathStateBelief.getGlobalStateBelief().clone();
      this.groundFilter.measure(updatedBelief, observation);
      result = priorPathStateBelief.getPath().getStateBeliefOnPath(updatedBelief);
    }

    return result;
  }

  /**
   * Pass it a road-coordinates prior predictive belief distribution, edge and
   * path starting distance, and it will return the prior predictive
   * distribution for that edge and path. Otherwise, it projects free-movement
   * onto an edge then projects. Note: this will project without regard to path
   * length.
   * 
   * @param startOfEdgeDist
   */
  public PathStateBelief predict(PathStateBelief currentBelief, InferredPath path) {

    Preconditions.checkNotNull(path);
    MultivariateGaussian newBelief;
    if (path.isNullPath()) {
      if (!currentBelief.isOnRoad()) {
        /*-
         * Predict free-movement
         */
        newBelief = currentBelief.getGlobalStateBelief().clone();
        groundFilter.predict(newBelief);
      } else {
        /*-
         * Going off-road
         */
        newBelief = currentBelief.getGroundBelief().clone();

        groundFilter.predict(newBelief);
      }
    } else {

      if (!currentBelief.isOnRoad()) {
        /*-
         * Project a current location onto the path, then 
         * project movement along the path.
         */
        newBelief = currentBelief.getLocalStateBelief().clone();
        PathUtils.convertToRoadBelief(newBelief, path, Iterables.getFirst(path.getPathEdges(), null), true);
      } else {
        final PathStateBelief newBeliefOnPath = path.getStateBeliefOnPath(currentBelief);
        newBelief = newBeliefOnPath.getGlobalStateBelief();
      }
      roadFilter.predict(newBelief);

      /*
       * Clamp to path
       */
      final double distance = AbstractRoadTrackingFilter.getOr().times(newBelief.getMean()).getElement(0);
      if (!path.isOnPath(distance)) {
        newBelief.getMean().setElement(0, path.clampToPath(distance));
      }
    }

    return path.getStateBeliefOnPath(newBelief);
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter.setModelCovariance(createStateCovarianceMatrix(currentTimeDiff, Qg, false));
      roadFilter.setModelCovariance(createStateCovarianceMatrix(currentTimeDiff, Qr, true));

      groundModel.setA(createStateTransitionMatrix(currentTimeDiff, false));
      roadModel.setA(createStateTransitionMatrix(currentTimeDiff, true));
    }
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
  }

  @Override
  public String toString() {
    return "StandardRoadTrackingFilter [" + "groundFilterCov=" + groundFilter.getModelCovariance() + ", roadFilterCov="
        + roadFilter.getModelCovariance() + ", onRoadStateVariance=" + onRoadStateTransCovar
        + ", offRoadStateVariance=" + offRoadStateTransCovar + ", obsVariance=" + obsCovar + ", currentTimeDiff="
        + currentTimeDiff + "]";
  }

  public Vector sampleStateTransDist(Vector state, Random rng) {
    final int dim = state.getDimensionality();
    final Matrix cov = dim == 4 ? this.getQg() : this.getQr();

    final Matrix covSqrt = StatisticsUtil.getCholR(cov);
    final Vector qSmpl =
        MultivariateGaussian.sample(VectorFactory.getDefault().createVector(cov.getNumColumns()), covSqrt, rng);

    final Matrix covFactor = this.getCovarianceFactor(dim == 2);
    final Vector error = covFactor.times(qSmpl);
    final Vector stateSmpl = state.plus(error);
    return stateSmpl;
  }

  protected static Matrix createStateCovarianceMatrix(double timeDiff, Matrix Q, boolean isRoad) {

    final Matrix A_half = getCovarianceFactor(timeDiff, isRoad);
    final Matrix A = A_half.times(Q).times(A_half.transpose());

    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) A);

    return A;
  }

  protected static Matrix createStateTransitionMatrix(double timeDiff, boolean isRoad) {

    final int dim;
    if (isRoad) {
      dim = 2;
    } else {
      dim = 4;
    }
    final Matrix Gct = MatrixFactory.getDefault().createIdentity(dim, dim);
    Gct.setElement(0, 1, timeDiff);
    if (dim > 2)
      Gct.setElement(2, 3, timeDiff);

    return Gct;
  }

  public static Matrix getCovarianceFactor(double timeDiff, boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(dim * 2, dim);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }

    return A_half;
  }

  public static Matrix getCovarianceFactorLeftInv(double timeDiff, boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half = MatrixFactory.getDefault().createMatrix(dim, dim * 2);
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

  protected static Matrix getRotatedCovarianceMatrix(double aVariance, double a0Variance, double angle) {

    final Matrix rotationMatrix = MatrixFactory.getDefault().createIdentity(2, 2);
    rotationMatrix.setElement(0, 0, Math.cos(angle));
    rotationMatrix.setElement(0, 1, -Math.sin(angle));
    rotationMatrix.setElement(1, 0, Math.sin(angle));
    rotationMatrix.setElement(1, 1, Math.cos(angle));

    final Matrix temp =
        MatrixFactory.getDefault().createDiagonal(
            VectorFactory.getDefault().copyArray(new double[] { a0Variance, aVariance }));
    return rotationMatrix.times(temp).times(rotationMatrix.transpose());
  }

  public static Matrix getU() {
    return U;
  }

  public static Vector getZeros2d() {
    return zeros2D;
  }

  protected void setQr(Matrix qr) {
    Preconditions.checkArgument(qr.getNumColumns() == 1);
    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) qr);
    Qr = qr;
    // TODO should set trans covar?
  }

  protected void setQg(Matrix qg) {
    Preconditions.checkArgument(qg.getNumColumns() == 2);
    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) qg);
    Qg = qg;
    // TODO should set trans covar?
  }

  protected void setOnRoadStateTransCovar(Matrix onRoadStateVariance) {
    Preconditions.checkArgument(onRoadStateVariance.getNumColumns() == 2);
    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) onRoadStateVariance);
    this.onRoadStateTransCovar = onRoadStateVariance.clone();
    this.roadFilter.setModelCovariance(onRoadStateVariance);
  }

  protected void setOffRoadStateTransCovar(Matrix offRoadStateVariance) {
    Preconditions.checkArgument(offRoadStateVariance.getNumColumns() == 4);
    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) offRoadStateVariance);
    this.offRoadStateTransCovar = offRoadStateVariance.clone();
    this.groundFilter.setModelCovariance(offRoadStateVariance);
  }

  protected void setObsCovar(Matrix obsVariance) {
    Preconditions.checkArgument(obsVariance.getNumColumns() == 2);
    Preconditions.checkState(!obsVariance.isZero());
    assert StatisticsUtil.isPosSemiDefinite((DenseMatrix) obsVariance);
    this.obsCovar = obsVariance.clone();
    this.groundFilter.setMeasurementCovariance(obsVariance);
    this.roadFilter.setMeasurementCovariance(obsVariance);
  }

  public static double getEdgeLengthErrorTolerance() {
    return edgeLengthErrorTolerance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((Qg == null) ? 0 : ((AbstractMatrix) Qg).hashCode());
    result = prime * result + ((Qr == null) ? 0 : ((AbstractMatrix) Qr).hashCode());
    long temp;
    temp = Double.doubleToLongBits(currentTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    result = prime * result + ((obsCovar == null) ? 0 : ((AbstractMatrix) obsCovar).hashCode());
    result =
        prime * result + ((offRoadStateTransCovar == null) ? 0 : ((AbstractMatrix) offRoadStateTransCovar).hashCode());
    result =
        prime * result + ((onRoadStateTransCovar == null) ? 0 : ((AbstractMatrix) onRoadStateTransCovar).hashCode());
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
    final AbstractRoadTrackingFilter other = (AbstractRoadTrackingFilter) obj;
    if (Qg == null) {
      if (other.Qg != null) {
        return false;
      }
    } else if (!((AbstractMatrix) Qg).equals((other.Qg))) {
      return false;
    }
    if (Qr == null) {
      if (other.Qr != null) {
        return false;
      }
    } else if (!((AbstractMatrix) Qr).equals((other.Qr))) {
      return false;
    }
    if (Double.doubleToLongBits(currentTimeDiff) != Double.doubleToLongBits(other.currentTimeDiff)) {
      return false;
    }
    if (obsCovar == null) {
      if (other.obsCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) obsCovar).equals((other.obsCovar))) {
      return false;
    }
    if (offRoadStateTransCovar == null) {
      if (other.offRoadStateTransCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) offRoadStateTransCovar).equals((other.offRoadStateTransCovar))) {
      return false;
    }
    if (onRoadStateTransCovar == null) {
      if (other.onRoadStateTransCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) onRoadStateTransCovar).equals((other.onRoadStateTransCovar))) {
      return false;
    }
    if (Double.doubleToLongBits(prevTimeDiff) != Double.doubleToLongBits(other.prevTimeDiff)) {
      return false;
    }
    return true;
  }

  public abstract void update(VehicleState state, GpsObservation obs, PathStateBelief updatedBelief,
    PathStateBelief pathAdjustedPriorBelief, Random rng);

  public static Matrix getVg() {
    return Vg;
  }

  public static Matrix getVr() {
    return Vr;
  }

}
