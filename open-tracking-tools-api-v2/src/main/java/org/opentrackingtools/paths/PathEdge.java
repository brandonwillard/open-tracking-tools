package org.opentrackingtools.paths;

import java.util.ArrayList;
import java.util.Random;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import org.opentrackingtools.distributions.BayesianEstimableDistribution;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Ordering;
import com.vividsolutions.jts.geom.Geometry;

public class PathEdge extends AbstractCloneableSerializable implements
    Comparable<PathEdge>, BayesianEstimableDistribution<PathEdge, PathEdge> {

  private static final long serialVersionUID = 2615199504616160384L;

  protected InferenceGraphEdge edge;
  protected Double distToStartOfEdge;
  protected Boolean isBackward;

  protected final static PathEdge nullPathEdge = new PathEdge(
      InferenceGraphEdge.getNullEdge());

  protected PathEdge(InferenceGraphEdge edge) {
    this.edge = edge;
    this.distToStartOfEdge = null;
    this.isBackward = null;
  }

  protected PathEdge(InferenceGraphEdge edge,
    Double distToStartOfEdge, Boolean isBackward) {
    Preconditions.checkState((isBackward != Boolean.TRUE)
        || distToStartOfEdge <= 0d);
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
    this.isBackward = isBackward;
  }

  @Override
  public int compareTo(PathEdge o) {
    return ComparisonChain
        .start()
        .compare(this.edge, o.getInferredEdge())
        .compare(this.isBackward(), o.isBackward(),
            Ordering.natural().nullsLast())
        .compare(this.distToStartOfEdge, o.getDistToStartOfEdge(),
            Ordering.natural().nullsLast()).result();
  }

  public EdgePredictiveResults getPredictiveLikelihoodResults(
    Path path, VehicleState state,
    PathStateDistribution beliefPrediction, GpsObservation obs) {

    final PathStateDistribution locationPrediction;

    /*
     * Edge marginal predictive likelihoods.
     * Note: doesn't apply to free-movement, defaults to 0.
     */
    final double edgePredMarginalLogLik;
    if (edge.isNullEdge()) {
      edgePredMarginalLogLik = 0d;
      locationPrediction = beliefPrediction;
    } else {

      MultivariateGaussian edgePrediction =
          this.getPriorPredictive(beliefPrediction, obs);

      edgePredMarginalLogLik =
          this.marginalPredictiveLogLikelihood(state, path,
              beliefPrediction.getRawStateBelief());

      locationPrediction = path.getStateBeliefOnPath(edgePrediction);
    }

    final double measurementPredLik;
    final double edgePredTransLogLik;
    if (locationPrediction != null) {

      measurementPredLik =
          locationPrediction.priorPredictiveLogLikelihood(
              obs.getProjectedPoint(), state.getMovementFilter());

      edgePredTransLogLik =
          state.getEdgeTransitionDist().logEvaluate(
              state.getBelief().getEdge().getInferredEdge(),
              locationPrediction.getEdge().getInferredEdge());

    } else {
      edgePredTransLogLik = Double.NaN;
      measurementPredLik = Double.NaN;
    }

    return new EdgePredictiveResults(beliefPrediction,
        locationPrediction, edgePredMarginalLogLik,
        edgePredTransLogLik, measurementPredLik);
  }

  public double marginalPredictiveLogLikelihood(VehicleState state,
    Path path, MultivariateGaussian beliefPrediction) {

    Preconditions.checkArgument(!this.isNullEdge());
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);

    return marginalPredictiveLogLikInternal(path, beliefPrediction);
  }

  /**
   * Truncated normal mixing component.
   * 
   * @param beliefPrediction
   * @return
   */
  protected double marginalPredictiveLogLikInternal(Path path,
    MultivariateGaussian beliefPrediction) {

    final double direction = this.isBackward() ? -1d : 1d;
    final double thisStartDistance =
        Math.abs(this.getDistToStartOfEdge());
    final double thisEndDistance =
        edge.getLength() + thisStartDistance;

    final Matrix Or = AbstractRoadTrackingFilter.getOr();

    final double var =
        Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0);

    final double mean =
        direction
            * Or.times(beliefPrediction.getMean()).getElement(0);

    final double t1 =
        UnivariateGaussian.CDF.evaluate(thisEndDistance, mean, var)
            - UnivariateGaussian.CDF.evaluate(thisStartDistance,
                mean, var);

    final double Z =
        UnivariateGaussian.CDF.evaluate(
            Math.abs(path.getTotalPathDistance()), mean, var)
            - UnivariateGaussian.CDF.evaluate(0, mean, var);

    return Math.log(t1) - Math.log(Z);
  }

  private double marginalPredictiveLogLikInternalOld1(
    MultivariateGaussian beliefPrediction) {

    final Matrix Or = AbstractRoadTrackingFilter.getOr();
    final double var =
        Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0)
            + Math.pow(this.getInferredEdge().getLength(), 2d) / 12d;
    final double mean =
        Or.times(beliefPrediction.getMean()).getElement(0);
    final double direction = this.isBackward ? -1d : 1d;

    final double evalPoint =
        (this.getDistToStartOfEdge() + (this.getDistToStartOfEdge() + direction
            * this.getInferredEdge().getLength())) / 2d;

    final double result =
        UnivariateGaussian.PDF.logEvaluate(evalPoint, mean, var);

    return result;
  }

  private double marginalPredictiveLogLikInternalOld2(
    MultivariateGaussian beliefPrediction) {

    Preconditions.checkArgument(!this.isNullEdge());
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);

    final Matrix Or = AbstractRoadTrackingFilter.getOr();
    final double stdDev =
        Math.sqrt(Or.times(beliefPrediction.getCovariance())
            .times(Or.transpose()).getElement(0, 0));
    final double mean =
        Or.times(beliefPrediction.getMean()).getElement(0);
    final double direction = this.isBackward ? -1d : 1d;
    final double distToEndOfEdge =
        direction * this.getInferredEdge().getLength()
            + this.getDistToStartOfEdge();
    final double startDistance =
        direction > 0d ? this.getDistToStartOfEdge()
            : distToEndOfEdge;
    final double endDistance =
        direction > 0d ? distToEndOfEdge : this
            .getDistToStartOfEdge();

    // FIXME use actual log calculations
    final double result =
        LogMath
            .subtract(StatisticsUtil.normalCdf(endDistance, mean,
                stdDev, true), StatisticsUtil.normalCdf(
                startDistance, mean, stdDev, true));

    return result;
  }

  private double getTruncatedNormalMean(final double origMean,
    double stdDev) {
    final double direction = this.isBackward() ? -1d : 1d;
    final double mean =
        (Math.signum(origMean) * direction) * Math.abs(origMean);
    final double startDistance =
        Math.abs(this.getDistToStartOfEdge());
    final double endDistance = edge.getLength() + startDistance;

    final double tt1 =
        UnivariateGaussian.PDF.logEvaluate(startDistance, mean,
            stdDev * stdDev);
    final double tt2 =
        UnivariateGaussian.PDF.logEvaluate(endDistance, mean, stdDev
            * stdDev);
    final double t1 =
        LogMath
            .subtract(tt1 > tt2 ? tt1 : tt2, tt1 > tt2 ? tt2 : tt1);

    final double t2 =
        LogMath
            .subtract(StatisticsUtil.normalCdf(endDistance, mean,
                stdDev, true), StatisticsUtil.normalCdf(
                startDistance, mean, stdDev, true));

    final double d2 = Math.log(stdDev) + t1 - t2;
    final double tmean = mean + (tt2 > tt1 ? -1d : 1d) * Math.exp(d2);

    return direction * tmean;
  }

  public MultivariateGaussian getPriorPredictive(
    PathStateDistribution belief, GpsObservation obs) {

    Preconditions.checkArgument(belief.isOnRoad());
    Preconditions.checkArgument(!this.isNullEdge());

    /*-
     * TODO really, this should just be the truncated/conditional
     * mean and covariance for the given interval/edge
     */
    final Matrix Or = AbstractRoadTrackingFilter.getOr();
    final double S =
        Or.times(belief.getCovariance()).times(Or.transpose())
            .getElement(0, 0)
            // + 1d;
            + Math
                .pow(
                    this.getInferredEdge().getLength()
                        / Math.sqrt(12), 2);
    final Matrix W =
        belief.getCovariance().times(Or.transpose()).scale(1 / S);
    final Matrix R =
        belief.getCovariance().minus(W.times(W.transpose()).scale(S));

    final double direction = this.isBackward() ? -1d : 1d;
    final double mean =
        (this.getDistToStartOfEdge() + (this.getDistToStartOfEdge() + direction
            * this.getInferredEdge().getLength())) / 2d;

    final Vector beliefMean = belief.getRawState();
    final double e = mean - Or.times(beliefMean).getElement(0);
    final Vector a = beliefMean.plus(W.getColumn(0).scale(e));

    assert StatisticsUtil
        .isPosSemiDefinite((gov.sandia.cognition.math.matrix.mtj.DenseMatrix) R);

    final MultivariateGaussian prediction =
        belief.getGlobalStateBelief().clone();
    prediction.setMean(a);
    prediction.setCovariance(R);

    return prediction;
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
    final PathEdge other = (PathEdge) obj;
    if (distToStartOfEdge == null) {
      if (other.distToStartOfEdge != null) {
        return false;
      }
    } else if (!distToStartOfEdge.equals(other.distToStartOfEdge)) {
      return false;
    }
    if (edge == null) {
      if (other.edge != null) {
        return false;
      }
    } else if (!edge.equals(other.edge)) {
      return false;
    }
    if (isBackward == null) {
      if (other.isBackward != null) {
        return false;
      }
    } else if (!isBackward.equals(other.isBackward)) {
      return false;
    }
    return true;
  }

  /**
   * Returns a state on the edge that's been truncated within the given
   * tolerance. The relative parameters set to true will return a state relative
   * to the edge (i.e. removing the distance to start).
   * 
   * @param state
   * @param tolerance
   * @param relative
   * @return the state on the edge or null if it's
   */
  public Vector getCheckedStateOnEdge(Vector state, double tolerance,
    boolean relative) {
    Preconditions.checkState(!isNullEdge());
    Preconditions.checkArgument(tolerance >= 0d);
    Preconditions.checkArgument(state.getDimensionality() == 2);

    final Vector newState = state.clone();
    final double distance = newState.getElement(0);
    final double direction = isBackward ? -1d : 1d;
    final double posDistAdj =
        direction * distance - Math.abs(distToStartOfEdge);
    final double overEndDist = posDistAdj - this.edge.getLength();
    if (overEndDist > 0d) {
      if (overEndDist > tolerance) {
        return null;
      } else {
        newState.setElement(0, direction * this.edge.getLength()
            + (relative ? 0d : distToStartOfEdge));
      }
    } else if (posDistAdj < 0d) {
      if (posDistAdj < -tolerance) {
        return null;
      } else {
        newState.setElement(0, (relative ? 0d : distToStartOfEdge));
      }
    }

    if (relative)
      newState.setElement(0, direction * posDistAdj);

    return newState;
  }

  public Double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  public Geometry getGeometry() {
    return this.edge.getGeometry();
  }

  public InferenceGraphEdge getInferredEdge() {
    return edge;
  }

  public double getLength() {
    return this.getGeometry().getLength();
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((distToStartOfEdge == null) ? 0 : distToStartOfEdge
                .hashCode());
    result = prime * result + ((edge == null) ? 0 : edge.hashCode());
    result =
        prime * result
            + ((isBackward == null) ? 0 : isBackward.hashCode());
    return result;
  }

  public Boolean isBackward() {
    return isBackward;
  }

  public boolean isNullEdge() {
    return this == nullPathEdge;
  }

  public boolean isOnEdge(double distance) {
    final double direction = this.isBackward ? -1d : 1d;
    final double posDistToStart = Math.abs(distToStartOfEdge);
    final double posDistOffset =
        direction * distance - posDistToStart;

    if (posDistOffset - edge.getLength() > 1e-7d) {
      return false;
    } else if (posDistOffset < 0d) {
      return false;
    }

    return true;
  }

  @Override
  public String toString() {
    if (this.isNullEdge()) {
      return "PathEdge [empty edge]";
    } else {
      final double distToStart =
          distToStartOfEdge == 0d && this.isBackward ? -0d
              : distToStartOfEdge.longValue();
      return "PathEdge [edge=" + edge.getEdgeId() + " ("
          + edge.getLength().longValue() + ")" + ", distToStart="
          + distToStart + "]";
    }
  }

  public static PathEdge getEdge(InferenceGraphEdge infEdge,
    double distToStart, Boolean isBackward) {
    Preconditions.checkArgument(isBackward != Boolean.TRUE
        || distToStart <= 0d);

    PathEdge edge;
    if (infEdge.isNullEdge() || isBackward == null) {
      edge = PathEdge.getNullPathEdge();
    } else {
      edge = new PathEdge(infEdge, distToStart, isBackward);
    }
    return edge;
  }

  public static PathEdge getNullPathEdge() {
    return nullPathEdge;
  }

  @Override
  public PathEdge clone() {
    PathEdge clone = (PathEdge) super.clone();
    clone.distToStartOfEdge = distToStartOfEdge;
    clone.edge = edge;
    clone.isBackward = isBackward;
    return clone;
  }

  @Override
  public ProbabilityFunction<PathEdge> getProbabilityFunction() {
  }

  @Override
  public PathEdge sample(Random random) {
  }

  @Override
  public ArrayList<? extends PathEdge> sample(Random random,
    int numSamples) {
  }

  @Override
  public RecursiveBayesianEstimatorPredictor<PathEdge, PathEdge>
      getBayesianEstimatorPredictor() {
  }

}
