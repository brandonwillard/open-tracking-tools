package org.opentrackingtools.distributions;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.Set;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.PathUtils.PathEdgeProjection;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

/**
 * A distribution over the vector "state" of a PathState, i.e. the distribution 
 * of location/velocity over a given path edge. 
 * The distribution is MultivariateGaussian, where the mean takes the
 * place of the vector state. Transformations between paths apply equally to
 * PathStateDistributions, such that the Gaussian's covariance is project up in
 * dimension when going on-road to off, and vice versa.
 * 
 * @author bwillard
 * 
 */
public class PathStateDistribution 
  extends TruncatedRoadGaussian 
  implements Cloneable, Comparable<PathStateDistribution> {

  private static final long serialVersionUID = -31238492416118648L;

  protected MultivariateGaussian groundBelief;
  protected PathState pathState;
  
  public PathStateDistribution(Path path, MultivariateGaussian dist) {
    super(dist, Double.POSITIVE_INFINITY, 0d);
    this.pathState = new PathState(path, dist.getMean());
    this.setMean(pathState);
  }

  public PathStateDistribution(PathState pathState, Matrix covar) {
    super(pathState, covar, Double.POSITIVE_INFINITY, 0d);
    this.pathState = pathState;
  }

  @Override
  public PathStateDistribution clone() {
    final PathStateDistribution clone =
        (PathStateDistribution) super.clone();
    clone.pathState = ObjectUtil.cloneSmart(this.pathState);
    clone.groundBelief = ObjectUtil.cloneSmart(this.groundBelief);
    return clone;
  }

  @Override
  public int compareTo(PathStateDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.pathState, o.pathState);
    comparator.append(this.getCovariance().toArray(), o.getCovariance().toArray());
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
    if (pathState == null) {
      if (other.pathState != null) {
        return false;
      }
    } else if (!pathState.equals(other.pathState)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getGroundBelief() {

    if (!pathState.isOnRoad())
      return this;

    if (this.groundBelief == null) {
      this.groundBelief =
          PathUtils.getGroundBeliefFromRoad(this,
              pathState.getEdge(), true);
    }

    return this.groundBelief;
  }

  public MultivariateGaussian getLocalStateBelief() {
    if (pathState.isOnRoad()) {
      return this;
    } else {
      final Vector mean =
          Preconditions.checkNotNull(pathState.getEdge()
              .getCheckedStateOnEdge(
                  this.getMean(),
                  MotionStateEstimatorPredictor
                      .getEdgeLengthErrorTolerance(), true));
      MultivariateGaussian localStateBelief = super.clone();
      localStateBelief.setMean(mean);
      return localStateBelief;
    }
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((pathState == null) ? 0 : pathState.hashCode());
    return result;
  }

  @Override
  public PathState sample(Random random) {
    final Vector stateSample = this.sample(random);
    PathState result = new PathState(this.pathState.getPath(), stateSample);
    return result;
  }

  public ArrayList<Vector> sample(Random random, int numSamples) {
    List<Vector> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(new PathState(pathState.getPath(), sample(random)));
    }
    return (ArrayList<Vector>) samples;
  }

  public PathStateDistribution getRelatableState(PathStateDistribution stateBelief) {

    final Path path = this.pathState.getPath();
    final PathState onThisPath = this.pathState.getRelatableState(stateBelief.getPathState());
    final Matrix covar;
    if (path.isNullPath()) {
      covar = stateBelief.getGroundBelief().getCovariance();
    } else {
      if (!stateBelief.getPathState().isOnRoad()) {
        /*
         * Project covar to edge 
         */
        final boolean pathIsBackward = path.isBackward();
        PathEdgeProjection proj =
            PathUtils.getRoadProjection(stateBelief.getMean(),
                path.getGeometry(), pathIsBackward, pathIsBackward ? onThisPath
                    .getEdge().getGeometry().reverse() : onThisPath
                    .getEdge().getGeometry(), onThisPath.getEdge()
                    .getDistToStartOfEdge());
        final Matrix C = stateBelief.getCovariance();
        covar =
            proj.getProjMatrix().transpose().times(C)
                .times(proj.getProjMatrix());
      } else {
        covar = stateBelief.getCovariance();
      }
    }
    final PathStateDistribution newBelief =
        stateBelief.clone();
    newBelief.setMean(onThisPath.getGlobalState());
    newBelief.setCovariance(covar);
    return newBelief;
  }

  public PathState getPathState() {
    return this.pathState;
  }
}
