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
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.PathUtils.PathEdgeProjection;
import org.opentrackingtools.util.SvdMatrix;

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
          pathStateDistribution.motionDistribution.getProbabilityFunction();
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

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.append("distribution", motionDistribution);
    builder.append("path", path);
    return builder.toString();
  }

  protected MultivariateGaussian motionDistribution;

  protected MultivariateGaussian groundDistribution = null;

  protected Path path;

  protected PathState pathState;

  protected MultivariateGaussian edgeDistribution = null;

  public PathStateDistribution(Path path, MultivariateGaussian dist) {
    Preconditions.checkArgument(path.isNullPath() || dist.getInputDimensionality() == 2);
    Preconditions.checkArgument(!path.isNullPath() || dist.getInputDimensionality() == 4);
    this.path = path;
    this.motionDistribution = dist;
    this.pathState = new PathState(path, dist.getMean());
  }

  public PathStateDistribution(
    PathStateDistribution pathStateDistribution) {
    this.pathState = pathStateDistribution.pathState;
    this.path = pathStateDistribution.path;
    this.motionDistribution = pathStateDistribution.motionDistribution;
    this.groundDistribution = pathStateDistribution.groundDistribution;
    this.edgeDistribution = pathStateDistribution.edgeDistribution;
  }

  @Override
  public PathStateDistribution clone() {
    final PathStateDistribution clone =
        (PathStateDistribution) super.clone();
    clone.path = ObjectUtil.cloneSmart(this.path);
    clone.pathState = ObjectUtil.cloneSmart(this.pathState);
    clone.motionDistribution = this.motionDistribution.clone();
    clone.groundDistribution = ObjectUtil.cloneSmart(this.groundDistribution);
    clone.edgeDistribution = ObjectUtil.cloneSmart(this.edgeDistribution);
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
    this.motionDistribution.convertFromVector(parameters);
    this.groundDistribution = null;
    this.edgeDistribution = null;
    this.pathState = new PathState(this.path, this.motionDistribution.getMean());
  }

  @Override
  public Vector convertToVector() {
    return this.motionDistribution.convertToVector();
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
    if (this.motionDistribution == null) {
      if (other.motionDistribution != null) {
        return false;
      }
    } else if (!this.motionDistribution.equals(other.motionDistribution)) {
      return false;
    }
    return true;
  }

  public Matrix getCovariance() {
    return this.motionDistribution.getCovariance();
  }

  public MultivariateGaussian getGroundDistribution() {

    if (!this.pathState.isOnRoad()) {
      return this.motionDistribution;
    }

    if (this.groundDistribution == null) {
      this.groundDistribution =
          PathUtils.getGroundBeliefFromRoad(this.motionDistribution,
              this.pathState.getEdge(), false, true);
    }

    return this.groundDistribution;
  }

  public MultivariateGaussian getEdgeDistribution() {
    if (!this.pathState.isOnRoad()) {
      return this.motionDistribution;
    } else {
      if (this.edgeDistribution == null) {
        final Vector mean =
            Preconditions.checkNotNull(this.pathState.getEdge()
                .getCheckedStateOnEdge(
                    this.motionDistribution.getMean(),
                    MotionStateEstimatorPredictor
                        .getEdgeLengthErrorTolerance(), true));
        final MultivariateGaussian localStateBelief =
            this.motionDistribution.clone();
        localStateBelief.setMean(mean);
        this.edgeDistribution = localStateBelief;
      }
      return edgeDistribution;
    }
  }

  @Override
  public PathState getMean() {
    return this.pathState;
  }

  public MultivariateGaussian getMotionDistribution() {
    return this.motionDistribution;
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
      covar = stateBelief.getGroundDistribution().getCovariance();
    } else {
      if (!stateBelief.getPathState().isOnRoad()) {
        /*
         * Project covar to edge 
         */
        final PathEdgeProjection proj =
            PathUtils.getRoadProjection(stateBelief.
                getMotionDistribution().getMean(), 
                path.getGeometry(), onThisPath.getEdge().getLine(), 
                onThisPath.getEdge().getDistToStartOfEdge());
        final Matrix C = stateBelief.getCovariance();
        covar = PathUtils.getProjectedCovariance(proj, C, true);
      } else {
        covar = stateBelief.getCovariance();
      }
    }
    final PathStateDistribution newBelief = new PathStateDistribution(path,
        new TruncatedRoadGaussian(onThisPath.getMotionState().clone(), new SvdMatrix(covar)));
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
      setGroundDistribution(MultivariateGaussian newGroundDist, Vector prevState, double timeDiff) {

    Preconditions.checkArgument(newGroundDist
        .getInputDimensionality() == 4);

    if (!this.path.isNullPath()) {
      this.motionDistribution =
          PathUtils.getRoadBeliefFromGround(newGroundDist,
              this.pathState.getEdge(), true, prevState, timeDiff);
    } else {
      this.motionDistribution = newGroundDist;
    }
    this.pathState = 
        new PathState(this.pathState.getPath(),
            this.motionDistribution.getMean());

    this.groundDistribution = newGroundDist;
    this.edgeDistribution = null;
  }

  public void setMean(Vector mean) {
    this.motionDistribution.setMean(mean);
    this.edgeDistribution = null;
    this.groundDistribution = null;
    this.pathState = new PathState(this.path, mean);
  }
  
  public PathStateDistribution convertToPath(Path newPath) {
    final PathState onThisPath = 
        this.pathState.convertToPath(newPath);
    final Matrix covar;
    if (newPath.isNullPath()) {
      covar = this.getGroundDistribution().getCovariance();
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
        covar = PathUtils.getProjectedCovariance(proj, C, true);
      } else {
        covar = this.getCovariance();
      }
    }
    final MultivariateGaussian newBelief = 
        this.motionDistribution.clone();
    newBelief.setMean(onThisPath.getMotionState());
    newBelief.setCovariance(covar);
    return new PathStateDistribution(newPath, newBelief);
  }
  
}
