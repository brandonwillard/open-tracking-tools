package org.opentrackingtools.paths.impl;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.estimators.AbstractRoadTrackingFilter;
import org.opentrackingtools.paths.AbstractPathState;
import org.opentrackingtools.paths.InferredPath;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.paths.PathStateBelief;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

public class SimplePathStateBelief extends AbstractPathState implements PathStateBelief {

  private static final long serialVersionUID =
      -31238492416118648L;

  protected MultivariateGaussian localStateBelief;
  protected MultivariateGaussian globalStateBelief;
  protected MultivariateGaussian rawStateBelief;
  protected MultivariateGaussian groundBelief;

  protected SimplePathStateBelief(InferredPath path,
    MultivariateGaussian state) {

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
  public SimplePathStateBelief clone() {
    final SimplePathStateBelief clone =
        (SimplePathStateBelief) super.clone();
    clone.rawStateBelief =
        ObjectUtil.cloneSmart(this.rawStateBelief);
    clone.localStateBelief =
        ObjectUtil.cloneSmart(this.localStateBelief);
    clone.globalStateBelief =
        ObjectUtil.cloneSmart(this.globalStateBelief);
    clone.groundBelief =
        ObjectUtil.cloneSmart(this.groundBelief);
    return clone;
  }

  @Override
  public int compareTo(PathState o) {
    final CompareToBuilder comparator =
        new CompareToBuilder();
    comparator.append(this.path, o.getPath());
    if (o instanceof SimplePathStateBelief)
      comparator
          .append(this.rawStateBelief, 
              ((SimplePathStateBelief) o).rawStateBelief);
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
    final SimplePathStateBelief other = (SimplePathStateBelief) obj;
    if (rawStateBelief == null) {
      if (other.rawStateBelief != null) {
        return false;
      }
    } else if (!rawStateBelief.equals(other.rawStateBelief)) {
      return false;
    }
    return true;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#getCovariance()
   */
  @Override
  public Matrix getCovariance() {
    return globalStateBelief.getCovariance();
  }

  @Override
  public PathEdge getEdge() {
    if (!isOnRoad())
      return Iterables.getOnlyElement(this.path.getPathEdges());
          
    if (edge == null) {
      Preconditions.checkState(!path.isNullPath());
      this.edge = path.getEdgeForDistance(
                  this.getGlobalState().getElement(0), false);
    }
    return Preconditions.checkNotNull(this.edge);
  }

  @Override
  public Vector getGlobalState() {
    return globalStateBelief.getMean();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#getGlobalStateBelief()
   */
  @Override
  public MultivariateGaussian getGlobalStateBelief() {
    return globalStateBelief;
  }

  @Override
  public MultivariateGaussian getGroundBelief() {
    
    if (!this.isOnRoad())
      return this.globalStateBelief;
    
    if (this.groundBelief == null) {
      this.groundBelief = PathUtils.getGroundBeliefFromRoad(this.globalStateBelief, 
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

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#getLocalStateBelief()
   */
  @Override
  public MultivariateGaussian getLocalStateBelief() {
    if (this.localStateBelief != null)
      return this.localStateBelief;
    if (this.path.isNullPath()) {
      this.localStateBelief = this.globalStateBelief;
    } else {
      final Vector mean =
          Preconditions
              .checkNotNull(this.getEdge()
                  .getCheckedStateOnEdge(
                      this.globalStateBelief.getMean(),
                      AbstractRoadTrackingFilter
                          .getEdgeLengthErrorTolerance(),
                      true));
      this.localStateBelief = this.globalStateBelief.clone();
      this.localStateBelief.setMean(mean);
    }
    return this.localStateBelief;
  }

  @Override
  public Vector getRawState() {
    return rawStateBelief.getMean();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#getRawStateBelief()
   */
  @Override
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
            + ((rawStateBelief == null) ? 0
                : rawStateBelief.hashCode());
    return result;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#logLikelihood(gov.sandia.cognition.math.matrix.Vector, org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter)
   */
  @Override
  public double logLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter) {
    return SimplePathStateBelief.logLikelihood(obs,
        filter.getObsCovar(), this);
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.impl.PathStateBelief#priorPredictiveLogLikelihood(gov.sandia.cognition.math.matrix.Vector, org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter)
   */
  @Override
  public double priorPredictiveLogLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter) {

    final Matrix measurementCovariance =
        filter.getObsCovar();
    final Matrix Q =
        AbstractRoadTrackingFilter
            .getOg()
            .times(this.getGroundBelief().getCovariance())
            .times(
                AbstractRoadTrackingFilter.getOg()
                    .transpose());
    Q.plusEquals(measurementCovariance);

    final double result =
        SimplePathStateBelief.logLikelihood(obs, Q, this);
    return result;
  }

  /**
   * Returns a version of this state with a path ending
   * at the current edge.
   * @return
   */
  @Override
  public PathStateBelief getTruncatedPathStateBelief() {
    
    if (!this.isOnRoad())
      return this;
    
    final InferredPath newPath = 
        this.path.getPathTo(this.getEdge());
    
    return new SimplePathStateBelief(newPath, this.rawStateBelief);
  }
  
  @Override
  public PathState getTruncatedPathState() {
    return getTruncatedPathStateBelief();
  }
  
  public static SimplePathStateBelief getPathStateBelief(
    @Nonnull InferredPath path,
    @Nonnull MultivariateGaussian state) {
    
    Preconditions.checkArgument(!path.isNullPath()
        || state.getInputDimensionality() == 4);
    
    final MultivariateGaussian adjState = PathUtils.checkAndGetConvertedBelief(state, path);
    
    return new SimplePathStateBelief(path, adjState);
  }

  public static double logLikelihood(Vector obs,
    PathState state,
    AbstractRoadTrackingFilter filter) {
    return SimplePathStateBelief.logLikelihood(obs,
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
  public static double logLikelihood(Vector obs, Matrix obsCov, PathState state) {
    final double result;
    //    if (state.isOnRoad()) {
    //      final PathEdge lastEdge = Iterables.getLast(
    //          state.getPath().getEdges(), null);
    //      final Geometry lastEdgeGeom = lastEdge.isBackward() ?
    //          lastEdge.getGeometry().reverse() : lastEdge.getGeometry();
    //      final MultivariateGaussian obsProjPar = 
    //          AbstractRoadTrackingFilter.getRoadObservation(obs, 
    //              obsCov, state.getPath(), lastEdge);
    //      
    //      final LocationIndexedLine lilPath = new LocationIndexedLine(
    //          lastEdgeGeom);
    //      final LinearLocation obsLoc = LengthLocationMap.
    //          getLocation(lastEdgeGeom, obsProjPar.getMean().getElement(0));
    ////      final LineSegment segment = obsLoc.getSegment(lastEdgeGeom);
    //      final Coordinate obsLocCoord = lilPath.extractPoint(obsLoc);
    //      final Vector obsGroundLoc = 
    //         GeoUtils.getVector(obsLocCoord); 
    //      final Vector perpUnitVec = obs.minus(obsGroundLoc).unitVector();
    //      final double obsProjCovPer = perpUnitVec.times(obsCov)
    //          .dotProduct(perpUnitVec);
    //     final Matrix measurementCov = MatrixFactory.getDefault()
    //                  .copyArray(new double[][] {{obsProjCovPer}});
    //     final Vector measurementMean = VectorFactory.getDefault()
    //                  .createVector1D(obs.euclideanDistance(obsGroundLoc));
    //      /*
    //       * TODO FIXME XXX: This decomp needs to be done properly.
    //       * I.e. the perpendicular component should be conditional
    //       * on the parallel, and/or (?) actually projected perpendicular.
    //       */
    //      result =
    //          // parallel component
    //          StatisticsUtil.logEvaluateNormal(obsProjPar.getMean(),
    //              AbstractRoadTrackingFilter.getOr().times(state.getGlobalState()),
    //              obsProjPar.getCovariance()) 
    //              
    //                // perpendicular component
    //              + StatisticsUtil.logEvaluateNormal(
    //                  VectorFactory.getDefault().createVector1D(0d), 
    //                  measurementMean , measurementCov);
    //    } else {
    final Vector groundState = state.getGroundState();
    result =
        StatisticsUtil.logEvaluateNormal(
            obs,
            AbstractRoadTrackingFilter.getOg().times(
                groundState), obsCov);

    //    }
    return result;
  }
}
