package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.AbstractDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.PathUtils.PathEdgeProjection;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;

/**
 * A distribution over the vector "state" of a PathState, i.e. the distribution
 * of location/velocity over a given path edge. The distribution is
 * MultivariateGaussian, where the mean takes the place of the vector state.
 * Transformations between paths apply equally to PathStateDistributions, such
 * that the Gaussian's covariance is project up in dimension when going on-road
 * to off, and vice versa.
 * 
 * @author bwillard
 * 
 */
public class PathStateDistribution extends
    AbstractDistribution<PathState> implements
    ClosedFormComputableDistribution<PathState>,
    Comparable<PathStateDistribution> {

  public static class PDF extends PathStateDistribution implements
      ProbabilityFunction<PathState> {

    /**
     * 
     */
    private static final long serialVersionUID = 6617680787425670505L;
    protected MultivariateGaussian.PDF gaussianPdf;

    public PDF(PathStateDistribution pathStateDistribution) {
      super(pathStateDistribution);
      this.gaussianPdf =
          pathStateDistribution.distribution.getProbabilityFunction();
    }

    @Override
    public void convertFromVector(Vector parameters) {
      this.gaussianPdf.convertFromVector(parameters);
    }

    @Override
    public Vector convertToVector() {
      return this.gaussianPdf.convertToVector();
    }

    @Override
    public Double evaluate(PathState input) {
      return this.gaussianPdf.evaluate(input);
    }

    @Override
    public double logEvaluate(PathState input) {
      return this.gaussianPdf.logEvaluate(input);
    }

  }

  private static final long serialVersionUID = -31238492416118648L;

  protected MultivariateGaussian distribution;

  protected MultivariateGaussian groundBelief = null;

  protected Path path;

  protected PathState pathState;

  public PathStateDistribution(Path path, MultivariateGaussian dist) {
    Preconditions.checkArgument(path.isNullPath() || dist.getInputDimensionality() == 2);
    Preconditions.checkArgument(!path.isNullPath() || dist.getInputDimensionality() == 4);
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
    clone.path = ObjectUtil.cloneSmart(this.path);
    clone.pathState = ObjectUtil.cloneSmart(this.pathState);
    clone.distribution = this.distribution.clone();
    clone.groundBelief = ObjectUtil.cloneSmart(this.groundBelief);
    return clone;
  }

  @Override
  public int compareTo(PathStateDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.pathState, o.pathState);
    comparator.append(this.getCovariance().toArray(), o
        .getCovariance().toArray());
    return comparator.toComparison();
  }

  @Override
  public void convertFromVector(Vector parameters) {
    this.distribution.convertFromVector(parameters);
  }

  @Override
  public Vector convertToVector() {
    return this.distribution.convertToVector();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!super.equals(obj)) {
      return false;
    }
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final PathStateDistribution other = (PathStateDistribution) obj;
    if (this.pathState == null) {
      if (other.pathState != null) {
        return false;
      }
    } else if (!this.pathState.equals(other.pathState)) {
      return false;
    }
    if (this.distribution == null) {
      if (other.distribution != null) {
        return false;
      }
    } else if (!this.distribution.equals(other.distribution)) {
      return false;
    }
    return true;
  }

  public Matrix getCovariance() {
    return this.distribution.getCovariance();
  }

  public MultivariateGaussian getGroundBelief() {

    if (!this.pathState.isOnRoad()) {
      return this.distribution;
    }

    if (this.groundBelief == null) {
      this.groundBelief =
          PathUtils.getGroundBeliefFromRoad(this.distribution,
              this.pathState.getEdge(), true);
    }

    return this.groundBelief;
  }

  public MultivariateGaussian getLocalStateBelief() {
    if (this.pathState.isOnRoad()) {
      return this.distribution;
    } else {
      final Vector mean =
          Preconditions.checkNotNull(this.pathState.getEdge()
              .getCheckedStateOnEdge(
                  this.distribution.getMean(),
                  MotionStateEstimatorPredictor
                      .getEdgeLengthErrorTolerance(), true));
      final MultivariateGaussian localStateBelief =
          this.distribution.clone();
      localStateBelief.setMean(mean);
      return localStateBelief;
    }
  }

  @Override
  public PathState getMean() {
    return this.pathState;
  }

  public MultivariateGaussian getMotionStateDistribution() {
    return this.distribution;
  }

  public PathState getPathState() {
    return this.pathState;
  }

  @Override
  public PDF getProbabilityFunction() {
    return new PDF(this);
  }

  public PathStateDistribution getRelatableState(
    PathStateDistribution stateBelief) {

    final Path path = this.pathState.getPath();
    final PathState onThisPath =
        this.pathState.getRelatableState(stateBelief.getPathState());
    final Matrix covar;
    if (path.isNullPath()) {
      covar = stateBelief.getGroundBelief().getCovariance();
    } else {
      if (!stateBelief.getPathState().isOnRoad()) {
        /*
         * Project covar to edge 
         */
        final PathEdgeProjection proj =
            PathUtils.getRoadProjection(stateBelief.
                getMotionStateDistribution().getMean(), 
                path.getGeometry(), onThisPath.getEdge().getLine(), 
                onThisPath.getEdge().getDistToStartOfEdge());
        final Matrix C = stateBelief.getCovariance();
        covar =
            proj.getProjMatrix().transpose().times(C)
                .times(proj.getProjMatrix());
      } else {
        covar = stateBelief.getCovariance();
      }
    }
    final PathStateDistribution newBelief = stateBelief.clone();
    newBelief.distribution.setMean(onThisPath.getMotionState());
    newBelief.distribution.setCovariance(covar);
    return newBelief;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((this.pathState == null) ? 0 : this.pathState
                .hashCode());
    return result;
  }

  @Override
  public PathState sample(Random random) {
    final Vector stateSample = this.sample(random);
    final PathState result =
        new PathState(this.pathState.getPath(), stateSample);
    return result;
  }

  @Override
  public ArrayList<PathState> sample(Random random, int numSamples) {
    final List<PathState> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(new PathState(this.pathState.getPath(), this
          .sample(random)));
    }
    return (ArrayList<PathState>) samples;
  }

  /**
   * Updates the internal distribution to correspond to the argument projected
   * onto the current edge.
   * 
   * @param newGroundDist
   */
  public void
      setGroundDistribution(MultivariateGaussian newGroundDist) {

    Preconditions.checkArgument(newGroundDist
        .getInputDimensionality() == 4);

    if (!this.path.isNullPath()) {
      this.distribution =
          PathUtils.getRoadBeliefFromGround(newGroundDist,
              this.pathState.getEdge(), true);
      this.pathState =
          new PathState(this.pathState.getPath(),
              this.distribution.getMean());
    }

    this.groundBelief = newGroundDist;
  }

  public void setMean(Vector mean) {
    this.distribution.setMean(mean);
    this.groundBelief = null;
    this.pathState = new PathState(this.path, mean);
  }
  
  public PathStateDistribution convertToPath(Path newPath) {
    final PathState onThisPath = 
        this.pathState.convertToPath(newPath);
    final Matrix covar;
    if (newPath.isNullPath()) {
      covar = this.getGroundBelief().getCovariance();
    } else {
      if (!this.pathState.isOnRoad()) {
        /*
         * Project covar to edge 
         */
        PathEdgeProjection proj = 
            PathUtils.getRoadProjection(this.pathState.getGroundState(), 
                path.getGeometry(), onThisPath.getEdge().getLine(), 
                onThisPath.getEdge().getDistToStartOfEdge());
        final Matrix C = this.getCovariance();
        covar = proj.getProjMatrix().transpose().times(C)
                .times(proj.getProjMatrix());
      } else {
        covar = this.getCovariance();
      }
    }
    final MultivariateGaussian newBelief = 
        this.distribution.clone();
    newBelief.setMean(onThisPath.getMotionState());
    newBelief.setCovariance(covar);
    return new PathStateDistribution(newPath, newBelief);
  }
  
}
