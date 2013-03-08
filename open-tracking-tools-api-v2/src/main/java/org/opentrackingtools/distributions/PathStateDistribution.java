package org.opentrackingtools.distributions;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.AbstractDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.PathUtils.PathEdgeProjection;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;

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
    extends AbstractDistribution<PathState>
    implements ClosedFormComputableDistribution<PathState>, Comparable<PathStateDistribution> {

  public static class PDF extends PathStateDistribution implements ProbabilityFunction<PathState> {

    protected MultivariateGaussian.PDF gaussianPdf; 
    
    public PDF(PathStateDistribution pathStateDistribution) {
      super(pathStateDistribution);
      this.gaussianPdf = pathStateDistribution.distribution.getProbabilityFunction();
    }

    @Override
    public Double evaluate(PathState input) {
      return this.gaussianPdf.evaluate(input);
    }

    @Override
    public Vector convertToVector() {
      return this.gaussianPdf.convertToVector();
    }

    @Override
    public void convertFromVector(Vector parameters) {
      this.gaussianPdf.convertFromVector(parameters);
    }

    @Override
    public double logEvaluate(PathState input) {
      return this.gaussianPdf.logEvaluate(input);
    }

  }

  private static final long serialVersionUID = -31238492416118648L;

  public PDF getProbabilityFunction() {
    return new PDF(this);
  }

  public PathState getMean() {
    return pathState;
  }

  public void setMean(Vector mean) {
    distribution.setMean(mean);
    pathState = new PathState(this.path, mean);
  }

  protected MultivariateGaussian groundBelief;
  protected PathState pathState;
  protected MultivariateGaussian distribution;
  protected Path path;
  
  public PathStateDistribution(Path path, MultivariateGaussian dist) {
    this.path = path;
    this.distribution = dist;
    this.pathState = new PathState(path, dist.getMean());
  }

  public PathStateDistribution(
    PathStateDistribution pathStateDistribution) {
    this.pathState = pathStateDistribution.pathState;
    this.path = pathStateDistribution.path;
    this.distribution = pathStateDistribution.distribution;
    this.groundBelief = pathStateDistribution.groundBelief;
  }

  @Override
  public PathStateDistribution clone() {
    final PathStateDistribution clone =
        (PathStateDistribution) super.clone();
    clone.path = ObjectUtil.cloneSmart(path);
    clone.pathState = ObjectUtil.cloneSmart(this.pathState);
    clone.distribution = this.distribution.clone();
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
    if (distribution == null) {
      if (other.distribution != null) {
        return false;
      }
    } else if (!distribution.equals(other.distribution)) {
      return false;
    }
    return true;
  }

  public MultivariateGaussian getGroundBelief() {

    if (!pathState.isOnRoad())
      return this.distribution;

    if (this.groundBelief == null) {
      this.groundBelief =
          PathUtils.getGroundBeliefFromRoad(this.distribution,
              pathState.getEdge(), true);
    }

    return this.groundBelief;
  }

  public MultivariateGaussian getLocalStateBelief() {
    if (pathState.isOnRoad()) {
      return this.distribution;
    } else {
      final Vector mean =
          Preconditions.checkNotNull(pathState.getEdge()
              .getCheckedStateOnEdge(
                  this.distribution.getMean(),
                  MotionStateEstimatorPredictor
                      .getEdgeLengthErrorTolerance(), true));
      MultivariateGaussian localStateBelief = this.distribution.clone();
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

  public ArrayList<PathState> sample(Random random, int numSamples) {
    List<PathState> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(new PathState(pathState.getPath(), sample(random)));
    }
    return (ArrayList<PathState>) samples;
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
    newBelief.distribution.setMean(onThisPath.getGlobalState());
    newBelief.distribution.setCovariance(covar);
    return newBelief;
  }

  public Matrix getCovariance() {
    return this.distribution.getCovariance();
  }

  public PathState getPathState() {
    return this.pathState;
  }

  @Override
  public Vector convertToVector() {
    return this.distribution.convertToVector();
  }

  @Override
  public void convertFromVector(Vector parameters) {
    this.distribution.convertFromVector(parameters);
  }

  public MultivariateGaussian getMotionStateDistribution() {
    return this.distribution;
  }
}
