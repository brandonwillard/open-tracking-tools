package org.opentrackingtools.distributions;

import java.util.ArrayList;
import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

/**
 * A distribution over the vector "state" of a PathState, i.e. [location,
 * velocity]. The distribution is MultivariateGaussian, where the mean takes the
 * place of the vector state. Transformations between paths apply equally to
 * PathStateDistributions, such that the Gaussian's covariance is project up in
 * dimension when going on-road to off, and vice versa.
 * 
 * @author bwillard
 * 
 */
public class PathStateDistribution extends PathState implements
    ComputableDistribution<PathState>,
    BayesianEstimableDistribution<PathState, PathStateDistribution>,
    Cloneable {

  private static final long serialVersionUID = -31238492416118648L;

  protected MultivariateGaussian localStateBelief;
  protected MultivariateGaussian globalStateBelief;
  protected MultivariateGaussian rawStateBelief;
  protected MultivariateGaussian groundBelief;

  protected PathStateDistribution(Path path,
    MultivariateGaussian state) {
    super(path, state.getMean());

    this.path = path;
    this.rawStateBelief = state.clone();
    this.globalStateBelief = state.clone();

    /*
     * Now make sure the result is on this path.
     */
    if (!path.isNullPath()) {
      this.globalStateBelief.getMean().setElement(
          0,
          path.clampToPath(this.globalStateBelief.getMean()
              .getElement(0)));
    }
  }

  @Override
  public PathStateDistribution clone() {
    final PathStateDistribution clone =
        (PathStateDistribution) super.clone();
    clone.rawStateBelief = ObjectUtil.cloneSmart(this.rawStateBelief);
    clone.localStateBelief =
        ObjectUtil.cloneSmart(this.localStateBelief);
    clone.globalStateBelief =
        ObjectUtil.cloneSmart(this.globalStateBelief);
    clone.groundBelief = ObjectUtil.cloneSmart(this.groundBelief);
    return clone;
  }

  @Override
  public int compareTo(PathState o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.path, o.getPath());
    if (o instanceof PathStateDistribution)
      comparator.append(this.rawStateBelief,
          ((PathStateDistribution) o).rawStateBelief);
    return comparator.toComparison();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!super.equals(obj)) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final PathStateDistribution other = (PathStateDistribution) obj;
    if (rawStateBelief == null) {
      if (other.rawStateBelief != null) {
        return false;
      }
    } else if (!rawStateBelief.equals(other.rawStateBelief)) {
      return false;
    }
    return true;
  }

  public Matrix getCovariance() {
    return globalStateBelief.getCovariance();
  }

  @Override
  public PathEdge getEdge() {
    if (!isOnRoad())
      return Iterables.getOnlyElement(this.path.getPathEdges());

    if (edge == null) {
      Preconditions.checkState(!path.isNullPath());
      this.edge =
          path.getEdgeForDistance(
              this.getGlobalState().getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public Vector getGlobalState() {
    return globalStateBelief.getMean();
  }

  public MultivariateGaussian getGlobalStateBelief() {
    return globalStateBelief;
  }

  public MultivariateGaussian getGroundBelief() {

    if (!this.isOnRoad())
      return this.globalStateBelief;

    if (this.groundBelief == null) {
      this.groundBelief =
          PathUtils.getGroundBeliefFromRoad(this.globalStateBelief,
              this.getEdge(), true);
    }

    return this.groundBelief;
  }

  @Override
  public Vector getGroundState() {
    return getGroundBelief().getMean();
  }

  @Override
  public Vector getLocalState() {
    return getLocalStateBelief().getMean();
  }

  public MultivariateGaussian getLocalStateBelief() {
    if (this.localStateBelief != null)
      return this.localStateBelief;
    if (this.path.isNullPath()) {
      this.localStateBelief = this.globalStateBelief;
    } else {
      final Vector mean =
          Preconditions.checkNotNull(this.getEdge()
              .getCheckedStateOnEdge(
                  this.globalStateBelief.getMean(),
                  AbstractRoadTrackingFilter
                      .getEdgeLengthErrorTolerance(), true));
      this.localStateBelief = this.globalStateBelief.clone();
      this.localStateBelief.setMean(mean);
    }
    return this.localStateBelief;
  }

  @Override
  public Vector getRawState() {
    return rawStateBelief.getMean();
  }

  public MultivariateGaussian getRawStateBelief() {
    return rawStateBelief;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((rawStateBelief == null) ? 0 : rawStateBelief
                .hashCode());
    return result;
  }

  public double logLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter) {
    return PathStateDistribution.logLikelihood(obs,
        filter.getObsCovar(), this);
  }

  public double priorPredictiveLogLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter) {

    final Matrix measurementCovariance = filter.getObsCovar();
    final Matrix Q =
        AbstractRoadTrackingFilter.getOg()
            .times(this.getGroundBelief().getCovariance())
            .times(AbstractRoadTrackingFilter.getOg().transpose());
    Q.plusEquals(measurementCovariance);

    final double result =
        PathStateDistribution.logLikelihood(obs, Q, this);
    return result;
  }

  public PathStateDistribution getTruncatedPathStateBelief() {

    if (!this.isOnRoad())
      return this;

    final Path newPath = this.path.getPathTo(this.getEdge());

    return new PathStateDistribution(newPath, this.rawStateBelief);
  }

  @Override
  public PathState getTruncatedPathState() {
    return getTruncatedPathStateBelief();
  }

  public static PathStateDistribution getPathStateBelief(
    @Nonnull Path path, @Nonnull MultivariateGaussian state) {

    Preconditions.checkArgument(!path.isNullPath()
        || state.getInputDimensionality() == 4);

    final MultivariateGaussian adjState =
        PathUtils.checkAndGetConvertedBelief(state, path);

    return new PathStateDistribution(path, adjState);
  }

  public static double logLikelihood(Vector obs, PathState state,
    AbstractRoadTrackingFilter filter) {
    return PathStateDistribution.logLikelihood(obs,
        filter.getObsCovar(), state);
  }

  /**
   * Returns the log likelihood for given state and observation.<br>
   * 
   * Note: When in a road-state we break down the likelihood into its parallel
   * and perpendicular components. This way the parallel (distance-along) is
   * evaluated correctly, since if we just used the ground-state likelihood
   * things like loops with considerable movement would be just as likely as
   * staying in the same place (for any velocity!).
   * 
   * @param obs
   * @param belief
   * @param edge
   * @return
   */
  public static double logLikelihood(Vector obs, Matrix obsCov,
    PathState state) {
    final double result;
    final Vector groundState = state.getGroundState();
    result =
        StatisticsUtil.logEvaluateNormal(obs,
            AbstractRoadTrackingFilter.getOg().times(groundState),
            obsCov);

    return result;
  }

  @Override
  public PathState sample(Random random) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ArrayList<? extends PathState> sample(Random random,
    int numSamples) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public
      RecursiveBayesianEstimatorPredictor<PathState, PathStateDistribution>
      getBayesianEstimatorPredictor() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ProbabilityFunction<PathState> getProbabilityFunction() {
    // TODO Auto-generated method stub
    return null;
  }
}
