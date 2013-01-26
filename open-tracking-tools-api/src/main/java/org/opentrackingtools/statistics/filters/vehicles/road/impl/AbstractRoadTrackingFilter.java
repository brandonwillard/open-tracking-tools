package org.opentrackingtools.statistics.filters.vehicles.road.impl;

import gov.sandia.cognition.math.matrix.AbstractMatrix;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.math.signals.LinearDynamicalSystem;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.filters.impl.AdjKalmanFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public abstract class AbstractRoadTrackingFilter<T extends AbstractRoadTrackingFilter<T>>
    extends AbstractCloneableSerializable implements
    Comparable<T> {

  public static class PathEdgeProjection {

    private final Matrix projMatrix;
    private final Vector offset;
    private final Vector positiveState;
    private final Entry<Matrix, Vector> otherProjection;

    public PathEdgeProjection(
      Entry<Matrix, Vector> projPair, Vector posState,
      Entry<Matrix, Vector> otherProj) {
      this.projMatrix = projPair.getKey();
      this.offset = projPair.getValue();
      this.positiveState = posState;
      this.otherProjection = otherProj;
    }

    public Vector getOffset() {
      return offset;
    }

    public Entry<Matrix, Vector> getOtherProjection() {
      return otherProjection;
    }

    public Vector getPositiveState() {
      return positiveState;
    }

    public Matrix getProjMatrix() {
      return projMatrix;
    }

    @Override
    public String toString() {
      return "PathEdgeProjection [projMatrix=" + projMatrix
          + ", offset=" + offset + ", otherProjection="
          + otherProjection + "]";
    }

  }

  /**
   * 
   */
  private static final long serialVersionUID =
      -3818533301279461087L;

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
  protected AdjKalmanFilter groundFilter;

  /**
   * State movement and observation model for the road-state.
   */
  protected LinearDynamicalSystem roadModel;

  /**
   * The filter that applies movement and updates prior distributions for the
   * road model.
   */
  protected AdjKalmanFilter roadFilter;

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
  protected static Matrix Og;

  /**
   * Extracts the distance from a road state.
   */
  protected static Matrix Or;

  /**
   * Extracts the velocity vector from a ground state.
   */
  protected static Matrix Vg;

  /**
   * Extracts the velocity from a road state.
   */
  protected static Vector Vr;

  /**
   * This matrix converts (x, y, vx, vy) to (x, vx, y, vy).
   */
  protected static Matrix U;

  static {
    Vg =
        MatrixFactory.getDenseDefault()
            .copyArray(
                new double[][] { { 0, 1, 0, 0 },
                    { 0, 0, 0, 1 } });

    Vr =
        VectorFactory.getDenseDefault().createVector2D(0,
            1d);

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

  protected final static Vector zeros2D = VectorFactory
      .getDefault().copyValues(0, 0);
  
  @Nonnull
  protected transient InferenceGraph graph;

  @Override
  public AbstractRoadTrackingFilter<T> clone() {
    final AbstractRoadTrackingFilter<T> clone =
        (AbstractRoadTrackingFilter<T>) super.clone();
    clone.currentTimeDiff = this.currentTimeDiff;
    clone.groundFilter = this.groundFilter.clone();
    clone.groundModel = this.groundModel.clone();
    clone.obsCovar = this.obsCovar.clone();
    clone.offRoadStateTransCovar =
        this.offRoadStateTransCovar.clone();
    clone.onRoadStateTransCovar =
        this.onRoadStateTransCovar.clone();
    clone.prevTimeDiff = this.prevTimeDiff;
    clone.Qg = this.Qg.clone();
    clone.Qr = this.Qr.clone();
    clone.roadFilter = this.roadFilter.clone();
    clone.roadModel = this.roadModel.clone();
    
    // XXX no cloning here
    clone.graph = this.graph;
    
    return clone;
  }

  public MultivariateGaussian createInitialLearnedObject() {
    return new MultivariateGaussian(groundFilter.getModel()
        .getState(), groundFilter.getModelCovariance());
  }

  public Matrix getCovarianceFactor(boolean isRoad) {
    return getCovarianceFactor(this.currentTimeDiff, isRoad);
  }

  public Matrix getCovarianceFactorLeftInv(boolean isRoad) {
    return getCovarianceFactorLeftInv(this.currentTimeDiff,
        isRoad);
  }

  public double getCurrentTimeDiff() {
    return currentTimeDiff;
  }

  public AdjKalmanFilter getGroundFilter() {
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

  public AdjKalmanFilter getRoadFilter() {
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
    final PathStateBelief stateBelief) {
    final MultivariateGaussian projBelief =
        stateBelief.getGroundBelief().clone();

    final Matrix Q =
        Og.times(projBelief.getCovariance()).times(
            Og.transpose());
    Q.plusEquals(this.groundFilter
        .getMeasurementCovariance());

    final MultivariateGaussian res =
        new MultivariateGaussian(Og.times(projBelief
            .getMean()), Q);
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
  public PathStateBelief measure(PathStateBelief priorPathStateBelief,
    Vector observation, PathEdge edge) {

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
          AbstractRoadTrackingFilter.getRoadObservation(
              observation, this.obsCovar, priorPathStateBelief.getPath(),
              edge);
      //              Iterables.getLast(belief.getPath().getEdges()));

      this.roadFilter.setMeasurementCovariance(obsProj
          .getCovariance());
      updatedBelief = priorPathStateBelief.getGlobalStateBelief().clone();
      this.roadFilter.measure(updatedBelief,
          obsProj.getMean());

      final PathStateBelief tmpBelief =
          priorPathStateBelief.getPath().getStateBeliefOnPath(updatedBelief);
      
      /*
       * Use the path actually traveled.
       * TODO: create a "getSubpath" in InferredPath
       */
      result = tmpBelief.getTruncatedPathStateBelief();

    } else {
      updatedBelief = priorPathStateBelief.getGlobalStateBelief().clone();
      this.groundFilter.measure(updatedBelief, observation);
      result =
          priorPathStateBelief.getPath().getStateBeliefOnPath(updatedBelief);
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
  public PathStateBelief predict(
    PathStateBelief currentBelief, InferredPath path) {

    Preconditions.checkNotNull(path);
    MultivariateGaussian newBelief;
    if (path.isNullPath()) {
      if (!currentBelief.isOnRoad()) {
        /*-
         * Predict free-movement
         */
        newBelief =
            currentBelief.getGlobalStateBelief().clone();
        groundFilter.predict(newBelief);
      } else {
        /*-
         * Going off-road
         */
        newBelief = currentBelief.getGroundBelief().clone();

        /*-
         * After this conversion, our covariance matrix will
         * have a shape that reflects the direction of the 
         * edge it was on, so we need to compensate for that,
         * since we don't believe that going off-road is more 
         * likely to put us in the area of the road (perhaps
         * the opposite).
         * For now we will simply reset the covariance.
         * (sample from a prior, if one exists?)
         */
        //        currentBelief.setCovariance(
        //            createStateCovarianceMatrix(this.currentTimeDiff, this.Qg, false));

        groundFilter.predict(newBelief);
      }
    } else {

      if (!currentBelief.isOnRoad()) {
        /*-
         * Project a current location onto the path, then 
         * project movement along the path.
         */
        newBelief =
            currentBelief.getLocalStateBelief().clone();
        convertToRoadBelief(newBelief, path,
            Iterables.getFirst(path.getPathEdges(), null),
            true);
      } else {
        final PathStateBelief newBeliefOnPath =
            path.getStateBeliefOnPath(currentBelief);
        newBelief = newBeliefOnPath.getGlobalStateBelief();
      }
      roadFilter.predict(newBelief);
    }

    return path.getStateBeliefOnPath(newBelief);
  }

  public void setCurrentTimeDiff(double currentTimeDiff) {
    if (currentTimeDiff != prevTimeDiff) {
      groundFilter
          .setModelCovariance(createStateCovarianceMatrix(
              currentTimeDiff, Qg, false));
      roadFilter
          .setModelCovariance(createStateCovarianceMatrix(
              currentTimeDiff, Qr, true));

      groundModel.setA(createStateTransitionMatrix(
          currentTimeDiff, false));
      roadModel.setA(createStateTransitionMatrix(
          currentTimeDiff, true));
    }
    this.prevTimeDiff = this.currentTimeDiff;
    this.currentTimeDiff = currentTimeDiff;
  }

  @Override
  public String toString() {
    return "StandardRoadTrackingFilter ["
        + "groundFilterCov="
        + groundFilter.getModelCovariance()
        + ", roadFilterCov="
        + roadFilter.getModelCovariance()
        + ", onRoadStateVariance=" + onRoadStateTransCovar
        + ", offRoadStateVariance="
        + offRoadStateTransCovar + ", obsVariance="
        + obsCovar + ", currentTimeDiff=" + currentTimeDiff
        + "]";
  }

  public static PathState
      convertToRoadState(Vector state, InferredPath path,
        boolean useAbsVelocity) {

    final PathEdgeProjection projPair =
        getRoadProjection(state, path, null);

    if (projPair == null)
      return path.getStateOnPath(state);

    final Vector projState =
        projPair
            .getProjMatrix()
            .transpose()
            .times(
                projPair.getPositiveState().minus(
                    projPair.getOffset()));

    if (useAbsVelocity) {
      final double absVelocity =
          VectorFactory.getDenseDefault()
              .copyVector(Vg.times(state)).norm2();
      projState.setElement(1,
          Math.signum(projState.getElement(1))
              * absVelocity);
    }

    /*
     * Since this projection was working wrt. positive movement,
     * we need to reconvert.
     */
    if (path.isBackward()) {
      projState.scaleEquals(-1d);
    }

    assert path.isOnPath(projState.getElement(0));

    return path.getStateOnPath(projState,
        AbstractRoadTrackingFilter
            .getEdgeLengthErrorTolerance());
  }

  public static Vector convertToGroundState(
    Vector locVelocity, PathEdge edge,
    boolean useAbsVelocity) {
    final PathEdgeProjection projPair =
        getGroundProjection(locVelocity, edge, false);

    if (projPair == null)
      return locVelocity;

    final Vector projMean =
        projPair.getProjMatrix()
            .times(projPair.getPositiveState())
            .plus(projPair.getOffset());

    if (useAbsVelocity) {
      final double absVelocity =
          Math.abs(locVelocity.getElement(1));
      if (absVelocity > 0d) {
        final Vector velocities =
            VectorFactory.getDenseDefault().copyVector(
                Vg.times(projMean));
        velocities.scaleEquals(absVelocity
            / velocities.norm2());
        projMean.setElement(1, velocities.getElement(0));
        projMean.setElement(3, velocities.getElement(1));
      }
    }

    assert isIsoMapping(locVelocity, projMean, edge);

    return projMean;
  }

  public static void convertToGroundBelief(
    MultivariateGaussian belief, PathEdge edge,
    boolean useAbsVelocity) {

    // TODO XXX FIXME debug, remove.
    final MultivariateGaussian beliefBefore =
        belief.clone();

    convertToGroundBelief(belief, edge, false,
        useAbsVelocity);

    assert Preconditions.checkNotNull(beliefBefore
        .getInputDimensionality() == 4
        || getRoadBeliefFromGround(belief, edge,
            useAbsVelocity).getMean()
            .minus(beliefBefore.getMean()).isZero(1e-4)
        ? true : null);
  }

  /**
   * This version simply adds the offset from the passed
   * PathEdge.
   * @param belief
   * @param edge
   * @param useAbsVelocity
   * @return
   */
  public static MultivariateGaussian
      getRoadBeliefFromGround(MultivariateGaussian belief,
        PathEdge edge, boolean useAbsVelocity) {
    Preconditions.checkArgument(belief
        .getInputDimensionality() == 4);
    final MultivariateGaussian tmpMg =
        getRoadBeliefFromGround(belief, 
            edge.isBackward() ? edge.getGeometry().reverse() : edge.getGeometry(), 
            edge.isBackward(), null, 0, true);
    tmpMg.getMean().setElement(
        0,
        tmpMg.getMean().getElement(0)
            + edge.getDistToStartOfEdge());
    return tmpMg;
  }

  public static PathEdgeProjection convertToGroundBelief(
    MultivariateGaussian belief, PathEdge edge,
    boolean allowExtensions, boolean useAbsVelocity) {

    final PathEdgeProjection projPair =
        getGroundProjection(belief.getMean(), edge,
            allowExtensions);

    if (projPair == null)
      return null;

    final Matrix C = belief.getCovariance();
    final Matrix projCov =
        projPair.getProjMatrix().times(C)
            .times(projPair.getProjMatrix().transpose());

    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) projCov);

    final Vector projMean =
        projPair.getProjMatrix()
            .times(projPair.getPositiveState())
            .plus(projPair.getOffset());

    if (useAbsVelocity) {
      final double absVelocity =
          Math.abs(belief.getMean().getElement(1));
      if (absVelocity > 0d) {
        final Vector velocities =
            VectorFactory.getDenseDefault().copyVector(
                Vg.times(projMean));
        velocities.scaleEquals(absVelocity
            / velocities.norm2());
        projMean.setElement(1, velocities.getElement(0));
        projMean.setElement(3, velocities.getElement(1));
      }
    }

    assert Preconditions.checkNotNull(isIsoMapping(
        belief.getMean(), projMean, edge) ? true : null);

    belief.setMean(projMean);
    belief.setCovariance(projCov);

    return projPair;
  }

  public static PathEdgeProjection getGroundProjection(
    Vector locVelocity, PathEdge edge,
    boolean allowExtensions) {

    Preconditions.checkArgument(locVelocity
        .getDimensionality() == 2
        || locVelocity.getDimensionality() == 4);

    if (locVelocity.getDimensionality() == 4)
      return null;

    Preconditions.checkArgument(allowExtensions
        || edge.isOnEdge(locVelocity.getElement(0)));

    Preconditions.checkArgument(!edge.isNullEdge());

    final Vector positiveMean;
    if (locVelocity.getElement(0) < 0d
        || (locVelocity.getElement(0) == 0d && edge
            .isBackward() == Boolean.TRUE)) {
      /*
       * We're going all positive here, since we should've been using
       * the reversed geometry if negative.
       */
      final Vector posMeanTmp = locVelocity.clone();

      double posLocation =
          Math.max(0d,
              locVelocity.getElement(0) + edge.getLength()
                  + Math.abs(edge.getDistToStartOfEdge()));

      /*
       * In cases where large negative movements past an edge are made,
       * we need to adjust the excess to be positive.
       */
      if (allowExtensions && posLocation < 0d) {
        posLocation =
            edge.getLength()
                + Math.abs(locVelocity.getElement(0));
      }

      posMeanTmp.setElement(0, posLocation);
      positiveMean = posMeanTmp;
    } else if (locVelocity.getElement(0) > 0d
        || (locVelocity.getElement(0) == 0d && edge
            .isBackward() == Boolean.FALSE)) {

      positiveMean = locVelocity.clone();

      assert edge.getDistToStartOfEdge() >= 0d;

      positiveMean.setElement(
          0,
          Math.max(
              0d,
              positiveMean.getElement(0)
                  - edge.getDistToStartOfEdge()));
    } else {
      throw new IllegalStateException();
    }

    assert positiveMean.getElement(0) >= 0d
        && (allowExtensions || positiveMean.getElement(0) <= edge
            .getLength() + 1);

    final Geometry geom = edge.getGeometry();
    final Entry<LineSegment, Double> segmentDist =
        getSegmentAndDistanceToStart(geom,
            positiveMean.getElement(0));
    final double absTotalPathDistanceToStartOfSegment =
        Math.abs(segmentDist.getValue());
    final double absTotalPathDistanceToEndOfSegment =
        absTotalPathDistanceToStartOfSegment
            + segmentDist.getKey().getLength();
    final Entry<Matrix, Vector> projPair =
        AbstractRoadTrackingFilter.posVelProjectionPair(
            segmentDist.getKey(),
            absTotalPathDistanceToStartOfSegment);

    if (!allowExtensions) {
      //    assert (Math.abs(belief.getMean().getElement(0)) <= absTotalPathDistanceToEndOfSegment 
      //        && Math.abs(belief.getMean().getElement(0)) >= absTotalPathDistanceToStartOfSegment);

      /*
       * Truncate, to keep it on the edge.
       */
      if (positiveMean.getElement(0) > absTotalPathDistanceToEndOfSegment) {
        positiveMean.setElement(0,
            absTotalPathDistanceToEndOfSegment);
      } else if (positiveMean.getElement(0) < absTotalPathDistanceToStartOfSegment) {
        positiveMean.setElement(0,
            absTotalPathDistanceToStartOfSegment);
      }
    }

    // TODO: missing other projection
    return new PathEdgeProjection(projPair, positiveMean,
        null);
  }

  public Vector sampleStateTransDist(Vector state,
    Random rng) {
    final int dim = state.getDimensionality();
    final Matrix sampleCovChol =
        StatisticsUtil.rootOfSemiDefinite(dim == 4 ? this
            .getQg() : this.getQr());
    final Vector qSmpl =
        MultivariateGaussian.sample(VectorFactory
            .getDenseDefault().createVector(dim / 2),
            sampleCovChol, rng);
    final Vector stateSmpl =
        state.plus(this.getCovarianceFactor(dim == 2)
            .times(qSmpl));
    return stateSmpl;
  }

  /**
   * Returns the projection onto the given path, or, if a non-null edge is
   * given, onto that edge.
   * 
   * <b>Important</b>: See
   * {@link AbstractRoadTrackingFilter#getRoadProjection(Vector, SimpleInferredPath, SimplePathEdge)}
   * about projection details.
   * 
   * @param belief
   * @param path
   * @param pathEdge
   * @param useAbsVelocity
   * @return
   */
  public static void convertToRoadBelief(
    MultivariateGaussian belief, InferredPath path,
    @Nullable PathEdge pathEdge, boolean useAbsVelocity) {
    
    MultivariateGaussian projBelief = getRoadBeliefFromGround(belief, 
        path.getGeometry(), path.isBackward(), pathEdge.getGeometry(), 
        pathEdge.getDistToStartOfEdge(), useAbsVelocity);
    
    belief.setMean(projBelief.getMean());
    belief.setCovariance(projBelief.getCovariance());
  }

  public static MultivariateGaussian getRoadBeliefFromGround(
    MultivariateGaussian belief, Geometry pathGeometry,
    boolean pathIsBackwards, @Nullable Geometry edgeGeometry, 
    double edgeDistanceToStartOnPath, boolean useAbsVelocity) {
    
    Preconditions.checkArgument(belief
        .getInputDimensionality() == 4);
    
    final PathEdgeProjection projPair =
        getRoadProjection(belief.getMean(), pathGeometry, 
            pathIsBackwards, edgeGeometry, edgeDistanceToStartOnPath);

    if (projPair == null)
      return null;

    final Matrix C = belief.getCovariance().clone();
    final Matrix projCov =
        projPair.getProjMatrix().transpose().times(C)
            .times(projPair.getProjMatrix());

    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) projCov);

    final Vector projMean =
        projPair
            .getProjMatrix()
            .transpose()
            .times(
                projPair.getPositiveState().minus(
                    projPair.getOffset()));

    if (useAbsVelocity) {
//      final double absVelocity =
//          VectorFactory.getDenseDefault()
//              .copyVector(Vg.times(belief.getMean()))
//              .norm2();
//      final double projVelocity = Math.signum(projMean.getElement(1))
//                  * absVelocity;
//      projMean.setElement(1, projVelocity);
    }

    /*
     * Since this projection was working wrt. positive movement,
     * if we're on a path moving backward, we need to flip the signs.
     * otherwise, predictions will advance in the opposite direction!
     */
    if (pathIsBackwards) {
      projMean.scaleEquals(-1d);
    }

    assert LengthLocationMap.getLocation(
        pathGeometry, projMean.getElement(0)) != null;

    MultivariateGaussian result = new MultivariateGaussian(
        projMean, projCov);

    return result;
  }

  protected static boolean isIsoMapping(
    @Nonnull Vector from, @Nonnull Vector to,
    @Nonnull PathEdge pathEdge) {

    /*
     * XXX TODO FIXME: this is temporary!  we should be testing
     * with full paths!!
     */
    final Vector adjFrom;
    if (Math.abs(pathEdge.getDistToStartOfEdge()) > 0d) {
      adjFrom = from.clone();
      adjFrom.setElement(0, adjFrom.getElement(0)
          - pathEdge.getDistToStartOfEdge());
    } else {
      adjFrom = from;
    }

    final boolean isBackward = pathEdge.isBackward();

    final Vector inversion;
    if (to.getDimensionality() == 2) {
      inversion = convertToGroundState(to, pathEdge, true);
    } else {

      final InferredPath invPath =
          SimpleInferredPath.getInferredPath(Collections
              .singletonList(SimplePathEdge.getEdge(
                  pathEdge.getInferredEdge(), 0d,
                  isBackward)), isBackward);
      inversion =
          convertToRoadState(to, invPath, true)
              .getLocalState();
    }
    final boolean result =
        inversion.equals(adjFrom,
            AbstractRoadTrackingFilter
                .getEdgeLengthErrorTolerance());
    return result;
  }

  private static boolean isIsoMapping(Vector from,
    Vector to, InferredPath path) {
    final Vector inversion = invertProjection(to, path);
    final boolean result =
        inversion.equals(from, AbstractRoadTrackingFilter
            .getEdgeLengthErrorTolerance());
    return result;
  }

  public static void convertToRoadBelief(
    MultivariateGaussian belief, InferredPath path,
    boolean useAbsVelocity) {
    convertToRoadBelief(belief, path, null,
        useAbsVelocity);
  }

  public static PathEdgeProjection getRoadProjection(
    Vector locVelocity, InferredPath path, PathEdge pathEdge) {
    return getRoadProjection(locVelocity, path.getGeometry(), path.isBackward(),
        pathEdge.getGeometry(), pathEdge.getDistToStartOfEdge());
  }
  
  /**
   * <b>Attention</b>: When we're projected onto a vertex of the LineString then
   * we can have multiple transformations. <br>
   * More importantly, we can lose velocity by performing this projection (how
   * much dependents on the orientation of the edge(s) being we're projecting
   * onto)! However, we can preserve it by simply using the velocity
   * projection's direction and the velocity magnitude of the original 4D
   * vector. This must be done by the user with the results of this method. <br>
   * <br>
   * The returned projection is always in the positive direction.
   * 
   * @param locVelocity
   * @param path
   * @param pathEdge
   * @return
   */
  public static PathEdgeProjection getRoadProjection(
    Vector locVelocity, Geometry pathGeometry, 
    boolean pathIsBackwards, Geometry edgeGeometry, 
    double edgeDistanceToStartOnPath) {
    
    Preconditions.checkArgument(locVelocity.getDimensionality() == 2
        || locVelocity.getDimensionality() == 4);
    Preconditions.checkArgument(edgeGeometry == null
        || pathGeometry.contains(edgeGeometry));

    if (locVelocity.getDimensionality() == 2)
      return null;


    final Vector m = locVelocity.clone();

    /*
     * We snap to the line and find the segment of interest.
     * When we're given a non-null distanceThreshold, we attempt
     * to snap a location that is between the start of the edge 
     * for the distance threshold and the actual distance threshold.
     */
    final LinearLocation lineLocation;
    final Coordinate currentPos =
        GeoUtils.makeCoordinate(Og.times(m));
    final double distanceToStartOfSegmentOnGeometry;
    final LineSegment pathLineSegment;
    if (edgeGeometry != null) {
      final Geometry edgeGeom =
          pathIsBackwards ? edgeGeometry.reverse() : edgeGeometry;
      final LocationIndexedLine locIndex =
          new LocationIndexedLine(edgeGeom);
      final LinearLocation tmpLocation =
          locIndex.project(currentPos);
      final LengthIndexedLine tmpLengthIdx =
          new LengthIndexedLine(edgeGeom);
      final double lengthOnEdge =
          tmpLengthIdx.indexOf(tmpLocation
              .getCoordinate(edgeGeom));

      /*
       * Due to some really weird behavior with indexAfter in JTS,
       * we're doing this for the edge first, then computing the distance
       * up to it on the path and using that with the full path geom.
       */
      pathLineSegment = tmpLocation.getSegment(edgeGeom);
      final LengthIndexedLine lengthIndex =
          new LengthIndexedLine(edgeGeom);
      distanceToStartOfSegmentOnGeometry =
          Math.abs(edgeDistanceToStartOnPath)
              + lengthIndex.indexOf(pathLineSegment.p0);

      lineLocation =
          LengthLocationMap.getLocation(pathGeometry,
              Math.abs(edgeDistanceToStartOnPath)
                  + lengthOnEdge);
    } else {
      final LocationIndexedLine locIndex =
          new LocationIndexedLine(pathGeometry);
      lineLocation = locIndex.project(currentPos);
      /*
       * Get the segment we're projected onto, and the distance offset
       * of the path.
       */
      pathLineSegment = lineLocation.getSegment(pathGeometry);

      final LengthIndexedLine lengthIndex =
          new LengthIndexedLine(pathGeometry);
      distanceToStartOfSegmentOnGeometry =
          lengthIndex.indexOf(pathLineSegment.p0);
    }

    final Coordinate pointOnLine =
        lineLocation.getCoordinate(pathGeometry);
    m.setElement(0, pointOnLine.x);
    m.setElement(2, pointOnLine.y);

    final Entry<Matrix, Vector> projPair =
        AbstractRoadTrackingFilter.posVelProjectionPair(
            pathLineSegment,
            distanceToStartOfSegmentOnGeometry);

    return new PathEdgeProjection(projPair, m, null);
  }

  protected static Matrix createStateCovarianceMatrix(
    double timeDiff, Matrix Q, boolean isRoad) {

    final Matrix A_half =
        getCovarianceFactor(timeDiff, isRoad);
    final Matrix A =
        A_half.times(Q).times(A_half.transpose());

    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) A);

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
        MatrixFactory.getDefault().createMatrix(dim * 2,
            dim);
    A_half.setElement(0, 0, Math.pow(timeDiff, 2) / 2d);
    A_half.setElement(1, 0, timeDiff);
    if (dim == 2) {
      A_half.setElement(2, 1, Math.pow(timeDiff, 2) / 2d);
      A_half.setElement(3, 1, timeDiff);
    }

    return A_half;
  }

  public static Matrix getCovarianceFactorLeftInv(
    double timeDiff, boolean isRoad) {

    final int dim;
    if (!isRoad) {
      dim = 2;
    } else {
      dim = 1;
    }
    final Matrix A_half =
        MatrixFactory.getDefault().createMatrix(dim,
            dim * 2);
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

  /**
   * Returns the lineSegment in the geometry of the edge and the
   * distance-to-start of the segment on the entire path. The line segment is in
   * the direction of the edge's geometry, and the distance-to-start has the
   * same sign as the direction of movement.
   * 
   * @param edge
   * @param distanceAlong
   * @return
   */
  public static Entry<LineSegment, Double>
      getSegmentAndDistanceToStart(Geometry geometry,
        double distAlongGeometry) {

    Preconditions.checkArgument(distAlongGeometry >= 0d);
    final LengthIndexedLine lengthIdxLine =
        new LengthIndexedLine(geometry);

    final LinearLocation lineLocation =
        LengthLocationMap.getLocation(geometry,
            distAlongGeometry);
    final LineSegment lineSegment =
        lineLocation.getSegment(geometry);
    final Coordinate startOfSegmentCoord = lineSegment.p0;
    final double positiveDistToStartOfSegmentOnGeometry =
        lengthIdxLine.indexOf(startOfSegmentCoord);

    double distanceToStartOfSegmentOnPath;
    distanceToStartOfSegmentOnPath =
        positiveDistToStartOfSegmentOnGeometry;

    return Maps.immutableEntry(lineSegment,
        distanceToStartOfSegmentOnPath);
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  public static Matrix getU() {
    return U;
  }

  public static Vector getZeros2d() {
    return zeros2D;
  }

  public static Vector invertProjection(Vector dist,
    InferredPath path) {
    Preconditions.checkArgument(!path.isNullPath());
    Preconditions
        .checkArgument(dist.getDimensionality() == 2
            || dist.getDimensionality() == 4);

    if (dist.getDimensionality() == 2) {
      /*
       * Convert to ground-coordinates
       */
      final PathEdgeProjection proj =
          getGroundProjection(dist,
              path.getEdgeForDistance(dist.getElement(0),
                  true), false);

      final Vector projMean =
          proj.getProjMatrix()
              .times(proj.getPositiveState())
              .plus(proj.getOffset());

      return projMean;

    } else {
      /*
       * Convert to road-coordinates
       */
      final PathEdgeProjection proj =
          getRoadProjection(dist, path, null);

      final Vector projMean =
          proj.getProjMatrix()
              .transpose()
              .times(
                  proj.getPositiveState().minus(
                      proj.getOffset()));

      return projMean;
    }
  }

  public static void invertProjection(
    MultivariateGaussian dist, InferredPath path,
    boolean useAbsVelocity) {
    Preconditions.checkArgument(!path.isNullPath());
    Preconditions.checkArgument(dist
        .getInputDimensionality() == 2
        || dist.getInputDimensionality() == 4);

    if (dist.getInputDimensionality() == 2) {
      /*
       * Convert to ground-coordinates
       */
      convertToGroundBelief(
          path.getStateBeliefOnPath(dist).getGlobalStateBelief(),
          path.getEdgeForDistance(dist.getMean().getElement(0), false),
          useAbsVelocity);
    } else {
      /*
       * Convert to road-coordinates
       */
      convertToRoadBelief(dist, path, useAbsVelocity);

    }
  }

  /**
   * Note: it's very important that the position be "normalized" relative to the
   * edge w.r.t. the velocity. That way, when we predict the next location, the
   * start position isn't relative to the wrong end of the edge, biasing our
   * measurements by the length of the edge in the wrong direction. E.g. {35,
   * -1} -> {-5, -1} for 30sec time diff., then relative to the origin edge we
   * will mistakenly evaluate a loop-around. Later, when evaluating likelihoods
   * on paths/path-edges, we'll need to re-adjust locally when path directions
   * don't line up.
   */
  @Deprecated
  public static void normalizeBelief(Vector mean,
    PathEdge edge) {

    Preconditions.checkArgument(edge.isOnEdge(mean
        .getElement(0)));

    final double desiredDirection;
    if (edge.getDistToStartOfEdge() == 0d) {
      desiredDirection = Math.signum(mean.getElement(1));
      /*
       * When the velocity is zero there's not way to normalize, other
       * than pure convention.
       */
      if (desiredDirection == 0d)
        return;
    } else {
      desiredDirection =
          Math.signum(edge.getDistToStartOfEdge());
    }

    if (Math.signum(mean.getElement(0)) != desiredDirection) {
      final double totalPosLength =
          edge.getInferredEdge().getLength()
              + Math.abs(edge.getDistToStartOfEdge());
      double newPosLocation =
          totalPosLength - Math.abs(mean.getElement(0));
      if (newPosLocation < 0d)
        newPosLocation = 0d;
      else if (newPosLocation > totalPosLength)
        newPosLocation = totalPosLength;

      final double newLocation =
          desiredDirection * newPosLocation;
      assert Double.compare(Math.abs(newLocation),
          totalPosLength) <= 0;
      //assert edge.isOnEdge(newLocation);

      mean.setElement(0, newLocation);
    }
  }

  /**
   * TODO FIXME associate these values with segments? Returns the matrix and
   * offset vector for projection onto the given edge. distEnd is the distance
   * from the start of the path to the end of the given edge. NOTE: These
   * results are only in the positive direction. Convert on your end.
   */
  static protected Entry<Matrix, Vector>
      posVelProjectionPair(LineSegment lineSegment,
        double distToStartOfLine) {

    final Vector start = GeoUtils.getVector(lineSegment.p0);
    final Vector end = GeoUtils.getVector(lineSegment.p1);

    final double length = start.euclideanDistance(end);

    final double distToStart = Math.abs(distToStartOfLine);

    final Vector P1 = end.minus(start).scale(1 / length);
    final Vector s1 = start.minus(P1.scale(distToStart));

    final Matrix P =
        MatrixFactory.getDefault().createMatrix(4, 2);
    P.setColumn(0, P1.stack(zeros2D));
    P.setColumn(1, zeros2D.stack(P1));

    final Vector a = s1.stack(zeros2D);

    return Maps.immutableEntry(U.times(P), U.times(a));
  }

  protected void setQr(Matrix qr) {
    Preconditions.checkArgument(qr.getNumColumns() == 1);
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) qr);
    Qr = qr;
    // TODO should set trans covar?
  }

  protected void setQg(Matrix qg) {
    Preconditions.checkArgument(qg.getNumColumns() == 2);
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) qg);
    Qg = qg;
    // TODO should set trans covar?
  }

  protected void setOnRoadStateTransCovar(
    Matrix onRoadStateVariance) {
    Preconditions.checkArgument(onRoadStateVariance
        .getNumColumns() == 2);
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) onRoadStateVariance);
    this.onRoadStateTransCovar =
        onRoadStateVariance.clone();
    this.roadFilter.setModelCovariance(onRoadStateVariance);
  }

  protected void setOffRoadStateTransCovar(
    Matrix offRoadStateVariance) {
    Preconditions.checkArgument(offRoadStateVariance
        .getNumColumns() == 4);
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) offRoadStateVariance);
    this.offRoadStateTransCovar =
        offRoadStateVariance.clone();
    this.groundFilter
        .setModelCovariance(offRoadStateVariance);
  }

  protected void setObsCovar(Matrix obsVariance) {
    Preconditions
        .checkArgument(obsVariance.getNumColumns() == 2);
    Preconditions.checkState(!obsVariance.isZero());
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) obsVariance);
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
    result =
        prime
            * result
            + ((Qg == null) ? 0 : ((AbstractMatrix) Qg)
                .hashCode());
    result =
        prime
            * result
            + ((Qr == null) ? 0 : ((AbstractMatrix) Qr)
                .hashCode());
    long temp;
    temp = Double.doubleToLongBits(currentTimeDiff);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    result =
        prime
            * result
            + ((obsCovar == null) ? 0
                : ((AbstractMatrix) obsCovar).hashCode());
    result =
        prime
            * result
            + ((offRoadStateTransCovar == null) ? 0
                : ((AbstractMatrix) offRoadStateTransCovar)
                    .hashCode());
    result =
        prime
            * result
            + ((onRoadStateTransCovar == null) ? 0
                : ((AbstractMatrix) onRoadStateTransCovar)
                    .hashCode());
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
    final AbstractRoadTrackingFilter other =
        (AbstractRoadTrackingFilter) obj;
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
    if (Double.doubleToLongBits(currentTimeDiff) != Double
        .doubleToLongBits(other.currentTimeDiff)) {
      return false;
    }
    if (obsCovar == null) {
      if (other.obsCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) obsCovar)
        .equals((other.obsCovar))) {
      return false;
    }
    if (offRoadStateTransCovar == null) {
      if (other.offRoadStateTransCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) offRoadStateTransCovar)
        .equals((other.offRoadStateTransCovar))) {
      return false;
    }
    if (onRoadStateTransCovar == null) {
      if (other.onRoadStateTransCovar != null) {
        return false;
      }
    } else if (!((AbstractMatrix) onRoadStateTransCovar)
        .equals((other.onRoadStateTransCovar))) {
      return false;
    }
    if (Double.doubleToLongBits(prevTimeDiff) != Double
        .doubleToLongBits(other.prevTimeDiff)) {
      return false;
    }
    return true;
  }

  /**
   * Transforms the observation and observation covariance to road coordinates
   * for the given edge and path.
   * 
   * @param obs
   * @param path
   * @param edge
   * @return
   */
  public static MultivariateGaussian getRoadObservation(
    Vector obs, Matrix obsCov, InferredPath path,
    PathEdge edge) {

    Preconditions
        .checkState(obs.getDimensionality() == 2
            && obsCov.getNumColumns() == 2
            && obsCov.isSquare());

    final Matrix obsCovExp =
        AbstractRoadTrackingFilter.getOg().transpose()
            .times(obsCov)
            .times(AbstractRoadTrackingFilter.getOg());
    final MultivariateGaussian obsProjBelief =
        new MultivariateGaussian(AbstractRoadTrackingFilter
            .getOg().transpose().times(obs), obsCovExp);
    AbstractRoadTrackingFilter.convertToRoadBelief(
        obsProjBelief, path, edge, true);

    final Vector y =
        AbstractRoadTrackingFilter.getOr().times(
            obsProjBelief.getMean());
    final Matrix Sigma =
        AbstractRoadTrackingFilter
            .getOr()
            .times(obsProjBelief.getCovariance())
            .times(
                AbstractRoadTrackingFilter.getOr()
                    .transpose());
    return new MultivariateGaussian(y, Sigma);
  }

  public abstract void update(VehicleState state,
    GpsObservation obs, PathStateBelief updatedBelief,
    PathStateBelief pathAdjustedPriorBelief, Random rng);

  public static Matrix getVg() {
    return Vg;
  }

  public static Vector getVr() {
    return Vr;
  }

  public static MultivariateGaussian getRoadBeliefFromGround(
    MultivariateGaussian belief, InferredEdge edge,
    boolean useAbsVelocity) {
    Preconditions.checkArgument(belief
        .getInputDimensionality() == 4);
    final MultivariateGaussian tmpMg =
        getRoadBeliefFromGround(belief, 
            edge.getGeometry(), false, null, 0, true);
    return tmpMg;
  }

}
