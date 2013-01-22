package org.opentrackingtools.impl.graph.paths;

import java.util.List;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import javax.annotation.Nonnull;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.impl.statistics.StatisticsUtil;
import org.opentrackingtools.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;

import com.google.common.base.Preconditions;
import com.google.common.collect.Lists;

public class PathStateBelief extends AbstractPathState
    implements Comparable<PathStateBelief> {

  private static final long serialVersionUID =
      -31238492416118648L;

  private MultivariateGaussian localStateBelief;
  private MultivariateGaussian globalStateBelief;
  private MultivariateGaussian rawStateBelief;
  private MultivariateGaussian groundBelief;

  protected PathStateBelief(InferredPath path,
    MultivariateGaussian state) {

    this.path = path;
    this.rawStateBelief = state.clone();
    this.globalStateBelief = state.clone();

    /*
     * Now make sure the result is on this path.
     */
    if (!path.isEmptyPath()) {
      this.globalStateBelief.getMean().setElement(
          0,
          path.clampToPath(this.globalStateBelief.getMean()
              .getElement(0)));
    }
  }

  @Override
  public PathStateBelief clone() {
    final PathStateBelief clone =
        (PathStateBelief) super.clone();
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
  public int compareTo(PathStateBelief o) {
    final CompareToBuilder comparator =
        new CompareToBuilder();
    comparator.append(this.path, o.path);
    comparator
        .append(this.rawStateBelief, o.rawStateBelief);
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
    final PathStateBelief other = (PathStateBelief) obj;
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
    if (edge == null) {
      this.edge =
          path.isEmptyPath() ? PathEdge.getEmptyPathEdge()
              : path.getEdgeForDistance(globalStateBelief
                  .getMean().getElement(0), false);
    }
    return this.edge;
  }

  @Override
  public Vector getGlobalState() {
    return globalStateBelief.getMean();
  }

  public MultivariateGaussian getGlobalStateBelief() {
    return globalStateBelief;
  }

  public MultivariateGaussian getGroundBelief() {
    if (this.groundBelief == null) {
      final MultivariateGaussian newBelief =
          this.globalStateBelief.clone();
      AbstractRoadTrackingFilter.convertToGroundBelief(
          newBelief, this.getEdge(), true);
      this.groundBelief = newBelief;
      return newBelief;
    } else {
      return this.groundBelief;
    }
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
    if (this.path.isEmptyPath()) {
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
      this.localStateBelief =
          new MultivariateGaussian(mean,
              this.globalStateBelief.getCovariance());
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
            + ((rawStateBelief == null) ? 0
                : rawStateBelief.hashCode());
    return result;
  }

  public double logLikelihood(Vector obs,
    AbstractRoadTrackingFilter filter) {
    return PathStateBelief.logLikelihood(obs,
        filter.getObsCovar(), this);
  }

  /**
   * Returns the log likelihood for the
   * {@link AbstractRoadTrackingFilter#getObservationBelief(MultivariateGaussian, PathEdge)
   * observation belief} .
   * 
   * @param obs
   * @param belief
   * @return
   */
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
        PathStateBelief.logLikelihood(obs, Q, this);
    return result;
  }

  /**
   * Returns a version of this state with a path ending
   * at the current edge.
   * @return
   */
  public PathStateBelief getTruncatedPath() {
    final List<PathEdge> newEdges = Lists.newArrayList();
    for (PathEdge edge1 : this.getPath().getEdges()) {
      newEdges.add(edge1);
      if (edge1.equals(this.getEdge())) {
        break;
      }
    }
    final InferredPath newPath = InferredPath.getInferredPath(newEdges, 
        this.getPath().getIsBackward());
    return PathStateBelief.getPathStateBelief(newPath, this.rawStateBelief);
  }
  
  public static PathStateBelief getPathStateBelief(
    @Nonnull InferredPath path,
    @Nonnull MultivariateGaussian state) {
    Preconditions.checkArgument(!path.isEmptyPath()
        || state.getInputDimensionality() == 4);
    return new PathStateBelief(path, state);
  }

  public static PathStateBelief getPathStateBelief(
    PathState newPathState, Matrix covariance) {
    Preconditions.checkState(newPathState.getRawState()
        .getDimensionality() == covariance.getNumColumns());
    final PathStateBelief result =
        getPathStateBelief(
            newPathState.path,
            new MultivariateGaussian(newPathState
                .getRawState(), covariance));
    result.localStateBelief =
        new MultivariateGaussian(
            newPathState.getLocalState(), covariance);
    return result;
  }

  public static double logLikelihood(Vector obs,
    AbstractPathState state,
    AbstractRoadTrackingFilter filter) {
    return PathStateBelief.logLikelihood(obs,
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
  public static double logLikelihood(Vector obs,
    Matrix obsCov, AbstractPathState state) {
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
