package org.opentrackingtools.graph.paths.util;

import java.util.Collections;
import java.util.Map.Entry;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseMatrix;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import org.apache.commons.lang3.ArrayUtils;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.distributions.impl.AdjMultivariateGaussian;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;
import com.vividsolutions.jts.operation.linemerge.LineMerger;
import com.vividsolutions.jts.util.AssertionFailedException;

public class PathUtils {

  public static class PathMergeResults {
    final Geometry path;
    final boolean toIsReversed;
  
    public PathMergeResults(Geometry path,
      boolean toIsReversed) {
      this.path = path;
      this.toIsReversed = toIsReversed;
    }
  
    public Geometry getPath() {
      return path;
    }
  
    public boolean isToIsReversed() {
      return toIsReversed;
    };
  
  }

  public static class PathEdgeProjection {
  
    private final Matrix projMatrix;
    private final Vector offset;
  
    public PathEdgeProjection(
      Matrix projMatrix, 
      Vector offset) {
      this.projMatrix = projMatrix;
      this.offset = offset;
    }
  
    public Vector getOffset() {
      return offset;
    }
  
    public Matrix getProjMatrix() {
      return projMatrix;
    }
  
    @Override
    public String toString() {
      return "PathEdgeProjection [projMatrix=" + projMatrix
          + ", offset=" + offset + "]";
    }
  
  }

  /**
   * Checks that the state is either on- or off-road, depending
   * on the given path, and converts if necessary.
   * 
   * @param state
   * @param path
   * @return
   */
  public static Vector checkAndGetConvertedState(Vector state,
    InferredPath path) {
    
    final Vector adjState;
    if (path.isNullPath() && state.getDimensionality() != 4) {
      
      final double dist = 
          AbstractRoadTrackingFilter.getOr().times(state).getElement(0);
      final PathEdge edge = path.getEdgeForDistance(dist, false);
      
      adjState = getGroundStateFromRoad(state, edge, true);
  
    } else if (!path.isNullPath() && state.getDimensionality() != 2) {
      
      adjState = getRoadStateFromGround(state, path, true);
    } else {
      adjState = state;
    }
    
    return adjState;
  }

  /**
   * See {@link #checkAndGetConvertedState(Vector, InferredPath)}
   * @param belief
   * @param path
   * @return
   */
  public static MultivariateGaussian checkAndGetConvertedBelief(MultivariateGaussian belief,
    InferredPath path) {
    
    final MultivariateGaussian adjBelief;
    if (path.isNullPath() && belief.getInputDimensionality() != 4) {
      
      final double dist = 
          AbstractRoadTrackingFilter.getOr().times(belief.getMean()).getElement(0);
      final PathEdge edge = path.getEdgeForDistance(dist, false);
      
      adjBelief = getGroundBeliefFromRoad(belief, edge, true);
  
    } else if (!path.isNullPath() && belief.getInputDimensionality() != 2) {
      
      adjBelief = getRoadBeliefFromGround(belief, path, true);
    } else {
      if (!path.isNullPath()) {
        Preconditions.checkState(
          path.isOnPath(
              AbstractRoadTrackingFilter.getOr()
              .times(belief.getMean()).getElement(0)));
      }
      adjBelief = belief;
    }
    
    Preconditions.checkState(belief.getMean().getDimensionality()
        == belief.getCovariance().getNumColumns()
        && belief.getCovariance().isSquare());
    
    return adjBelief;
  }
  
  public static Geometry getMovementPath(VehicleState state) {
    if (state.getBelief().isOnRoad()
        && state.getParentState() != null) {
      final Geometry fullPath = state.getBelief().getPath()
          .getGeometry();
      final PathState prevStateOnPath = 
          state.getBelief().getPath().getStateOnPath(state.getBelief());
      final double distStart = prevStateOnPath.getGlobalState()
          .getElement(0);
      final double distEnd = state.getBelief().getGlobalState()
          .getElement(0);
      final LinearLocation startLoc = LengthLocationMap.
          getLocation(fullPath, distStart);
      final LinearLocation endLoc = LengthLocationMap.
          getLocation(fullPath, distEnd);
      LocationIndexedLine lil = new LocationIndexedLine(fullPath);
      return lil.extractLine(startLoc, endLoc);
    } else {
      return JTSFactoryFinder.getGeometryFactory()
          .createPoint(new Coordinate());
    }
  }
  
  /**
   * This projects ground-coordinates belief to the closest location on the path.
   * @param belief
   * @param useAbsVelocity
   * @return
   */
  public static MultivariateGaussian
      getRoadBeliefFromGround(MultivariateGaussian belief,
        InferredPath path, boolean useAbsVelocity) {
    Preconditions.checkArgument(belief
        .getInputDimensionality() == 4);
    final MultivariateGaussian tmpMg =
        getRoadBeliefFromGround(belief, path.getGeometry(),
            path.isBackward(), null, 0, useAbsVelocity);
    return tmpMg;
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
            edge.isBackward(), null, 0, useAbsVelocity);
    tmpMg.getMean().setElement(
        0,
        tmpMg.getMean().getElement(0)
            + edge.getDistToStartOfEdge());
    return tmpMg;
  }

  public static void convertToGroundBelief(
    MultivariateGaussian belief, PathEdge edge,
    boolean allowExtensions, boolean useAbsVelocity) {
  
    final PathEdgeProjection projPair =
        getGroundProjection(belief.getMean(), edge,
            allowExtensions);
  
    if (projPair == null)
      return;
  
    final Matrix C = belief.getCovariance();
    final Matrix projCov =
        projPair.getProjMatrix().times(C)
            .times(projPair.getProjMatrix().transpose());
  
    assert StatisticsUtil
        .isPosSemiDefinite((DenseMatrix) projCov);
  
    final Vector posState = edge.isBackward() ? belief.getMean().scale(-1d)
        : belief.getMean();
    
    final Vector projMean =
        projPair.getProjMatrix()
            .times(posState)
            .plus(projPair.getOffset());
  
    if (useAbsVelocity) {
      final double absVelocity =
          Math.abs(AbstractRoadTrackingFilter.getVr().times(belief.getMean()).getElement(0));
      if (absVelocity > 0d) {
        final Vector velocities =
            VectorFactory.getDenseDefault().copyVector(
                AbstractRoadTrackingFilter.getVg().times(projMean));
        velocities.scaleEquals(absVelocity
            / velocities.norm2());
        projMean.setElement(1, velocities.getElement(0));
        projMean.setElement(3, velocities.getElement(1));
      }
    }
  
    belief.setMean(projMean);
    belief.setCovariance(projCov);
  }

  /**
   * Returns a projection in the path direction
   * (so use this with positive, path-directed vectors).
   * @param locVelocity
   * @param edge
   * @param allowExtensions
   * @return
   */
  public static PathEdgeProjection getGroundProjection(
    Vector locVelocity, PathEdge edge,
    boolean allowExtensions) {
  
    final Geometry geom;
    double distance = AbstractRoadTrackingFilter.getOr()
        .times(locVelocity).getElement(0);
    if (edge.isBackward()) { 
      geom = edge.getGeometry().reverse();
      distance *= -1d;
    } else {
      geom = edge.getGeometry();
    }
    
    final Entry<LineSegment, Double> segmentDist =
        getSegmentAndDistanceToStart(geom, distance);
    
    final double absTotalPathDistanceToStartOfSegment =
        Math.abs(segmentDist.getValue()) + Math.abs(edge.getDistToStartOfEdge());
  
    return PathUtils.posVelProjectionPair(
            segmentDist.getKey(),
            absTotalPathDistanceToStartOfSegment);
  }

  /**
   * Returns the projection onto the given path, or, if a non-null edge is
   * given, onto that edge.
   * 
   * <b>Important</b>: See
   * {@link PathUtils#getRoadProjection(Vector, SimpleInferredPath, SimplePathEdge)}
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
        path.getGeometry(), path.isBackward(), 
        pathEdge.isBackward() ? pathEdge.getGeometry().reverse() : pathEdge.getGeometry(), 
        pathEdge.getDistToStartOfEdge(), useAbsVelocity);
    
    belief.setMean(projBelief.getMean());
    belief.setCovariance(projBelief.getCovariance());
  }

  public static Vector getRoadStateFromGround(
      Vector state, InferredPath path, boolean useAbsVelocity) {
    return getRoadStateFromGround(state, path.getGeometry(),
        path.isBackward(), null, 0, useAbsVelocity); 
  }
  
  public static Vector getRoadStateFromGround(
      Vector state, Geometry pathGeometry,
      boolean pathIsBackwards, @Nullable Geometry edgeGeometry, 
      double edgeDistanceToStartOnPath, boolean useAbsVelocity) {
      
    Preconditions.checkArgument(state
        .getDimensionality() == 4);
    
    final PathEdgeProjection projPair =
        getRoadProjection(state, pathGeometry, 
            pathIsBackwards, edgeGeometry, edgeDistanceToStartOnPath);

    if (projPair == null)
      return null;

    final Vector projMean =
        projPair
            .getProjMatrix()
            .transpose()
            .times(
                state.minus(
                    projPair.getOffset()));
    
    /*
     * When there is no exactly orthogonal line to the edge
     * we must clip the result.
     */
    final LengthIndexedLine lil =
        new LengthIndexedLine(pathGeometry);
    final double clampedIndex =
        lil.clampIndex(projMean.getElement(0));
    projMean.setElement(0, clampedIndex);
    
    if (pathIsBackwards)
      projMean.scaleEquals(-1d);

    if (useAbsVelocity) {
      final double absVelocity =
          VectorFactory.getDenseDefault()
              .copyVector(AbstractRoadTrackingFilter.getVg()
                  .times(state))
              .norm2();
      final double projVelocity = Math.signum(projMean.getElement(1))
                  * absVelocity;
      projMean.setElement(1, projVelocity);
    }

    assert LengthLocationMap.getLocation(
        pathGeometry, projMean.getElement(0)) != null;

    return projMean;
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
                belief.getMean().minus(
                    projPair.getOffset()));
    
    /*
     * When there is no exactly orthogonal line to the edge
     * we must clip the result.
     */
    final LengthIndexedLine lil =
        new LengthIndexedLine(pathGeometry);
    final double clampedIndex =
        lil.clampIndex(projMean.getElement(0));
    projMean.setElement(0, clampedIndex);
    
    if (pathIsBackwards)
      projMean.scaleEquals(-1d);

    if (useAbsVelocity) {
      final double absVelocity =
          VectorFactory.getDenseDefault()
              .copyVector(AbstractRoadTrackingFilter.getVg()
                  .times(belief.getMean()))
              .norm2();
      final double projVelocity = Math.signum(projMean.getElement(1))
                  * absVelocity;
      projMean.setElement(1, projVelocity);
    }

    assert LengthLocationMap.getLocation(
        pathGeometry, projMean.getElement(0)) != null;

    
    MultivariateGaussian result = belief.clone();
    result.setMean(projMean);
    result.setCovariance(projCov);

    return result;
  }

  public static boolean isIsoMapping(
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
      inversion = getGroundStateFromRoad(to, pathEdge, true);
    } else {
  
      final InferredPath invPath =
          SimpleInferredPath.getInferredPath(Collections
              .singletonList(SimplePathEdge.getEdge(
                  pathEdge.getInferredEdge(), 
                  pathEdge.getGeometry(), 0d,
                  isBackward)), isBackward);
      inversion =
          invPath.getStateOnPath(to)
              .getLocalState();
    }
    final boolean result =
        inversion.equals(adjFrom,
            AbstractRoadTrackingFilter
                .getEdgeLengthErrorTolerance());
    return result;
  }

  public static void convertToRoadBelief(
    MultivariateGaussian belief, InferredPath path,
    boolean useAbsVelocity) {
    convertToRoadBelief(belief, path, null,
        useAbsVelocity);
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
//    Preconditions.checkArgument(edgeGeometry == null
//        || pathGeometry.contains(edgeGeometry));
  
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
        GeoUtils.makeCoordinate(AbstractRoadTrackingFilter.getOg().times(m));
    final double distanceToStartOfSegmentOnGeometry;
    final LineSegment pathLineSegment;
    if (edgeGeometry != null) {
      final Geometry edgeGeom = edgeGeometry;
//          pathIsBackwards ? edgeGeometry.reverse() : edgeGeometry;
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
  
    return PathUtils.posVelProjectionPair(
            pathLineSegment,
            distanceToStartOfSegmentOnGeometry);
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
          edge.getLength()
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
  public static PathEdgeProjection
      posVelProjectionPair(LineSegment lineSegment,
        double distToStartOfLine) {
    
    Preconditions.checkState(lineSegment.getLength() > 0d);
  
    final Vector start = GeoUtils.getVector(lineSegment.p0);
    final Vector end = GeoUtils.getVector(lineSegment.p1);
  
    final double length = start.euclideanDistance(end);
  
    final double distToStart = Math.abs(distToStartOfLine);
  
    final Vector P1 = end.minus(start).scale(1 / length);
    final Vector s1 = start.minus(P1.scale(distToStart));
  
    final Matrix P =
        MatrixFactory.getDefault().createMatrix(4, 2);
    P.setColumn(0, P1.stack(AbstractRoadTrackingFilter.getZeros2d()));
    P.setColumn(1, AbstractRoadTrackingFilter.getZeros2d().stack(P1));
  
    final Vector a = s1.stack(AbstractRoadTrackingFilter.getZeros2d());
  
    return new PathEdgeProjection(AbstractRoadTrackingFilter.getU().times(P), 
        AbstractRoadTrackingFilter.getU().times(a));
  }

  public static MultivariateGaussian getRoadBeliefFromGround(
    MultivariateGaussian belief, InferredEdge edge,
    boolean useAbsVelocity) {
    Preconditions.checkArgument(belief
        .getInputDimensionality() == 4);
    final MultivariateGaussian tmpMg =
        getRoadBeliefFromGround(belief, 
            edge.getGeometry(), false, null, 0, useAbsVelocity);
    return tmpMg;
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
        new AdjMultivariateGaussian(AbstractRoadTrackingFilter
            .getOg().transpose().times(obs), obsCovExp);
    convertToRoadBelief(
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
    return new AdjMultivariateGaussian(y, Sigma);
  }

  public static MultivariateGaussian getGroundBeliefFromRoad(
    MultivariateGaussian belief, PathEdge edge, boolean useAbsVelocity) {
    final MultivariateGaussian newBelief = belief.clone();
    convertToGroundBelief(newBelief, edge, useAbsVelocity, useAbsVelocity);
    return newBelief;
  }
  
  public static Vector getGroundStateFromRoad(
    Vector locVelocity, PathEdge edge,
    boolean useAbsVelocity) {
    
    final PathEdgeProjection projPair =
        getGroundProjection(locVelocity, edge, false);
  
    if (projPair == null)
      return locVelocity;
  
    final Vector posState = edge.isBackward() ? 
        locVelocity.scale(-1d) : locVelocity;
    
    final Vector projMean =
        projPair.getProjMatrix()
            .times(posState)
            .plus(projPair.getOffset());
  
    if (useAbsVelocity) {
      final double absVelocity =
          Math.abs(AbstractRoadTrackingFilter.getVr().times(locVelocity).getElement(0));
      if (absVelocity > 0d) {
        final Vector velocities =
            VectorFactory.getDenseDefault().copyVector(
              AbstractRoadTrackingFilter.getVg().times(projMean));
        velocities.scaleEquals(absVelocity
            / velocities.norm2());
        projMean.setElement(1, velocities.getElement(0));
        projMean.setElement(3, velocities.getElement(1));
      }
    }
  
    return projMean;
  }

  /**
   * Converts a state to the same direction as the path.  If the opposite
   * direction of the state isn't on the path, null is returned.
   * 
   * @param state
   * @param path
   * @return
   */
  public static Vector adjustForOppositeDirection(Vector state, InferredPath path) {
    final Vector newState = state.clone();
    final double distance = newState.getElement(0);
    final double direction = Math.signum(path.getTotalPathDistance());
    final double overTheEndDist =
        direction * distance - Math.abs(path.getTotalPathDistance());
    
    if (overTheEndDist > 0d) {
      if (overTheEndDist 
          > AbstractRoadTrackingFilter.getEdgeLengthErrorTolerance()) {
        return null;
      } else {
        newState.setElement(0, path.getTotalPathDistance());
      }
    } else if (direction * distance < 0d) {
      if (direction * distance 
          < AbstractRoadTrackingFilter.getEdgeLengthErrorTolerance()) {
        return null;
      } else {
        newState.setElement(0, direction * 0d);
      }
    }
    
    return newState;
  }

  /**
   * Returns the path connecting the two passed geoms and distances. XXX:
   * assumes that from end and to start edges overlap.
   * 
   * @param from
   * @param distFrom
   * @param inTo
   * @param inDistTo
   * @return
   */
  public static PathUtils.PathMergeResults mergePaths(Geometry from,
    double distFrom, Geometry to, double distTo) {
  
    /*
     * Make sure we're only dealing with the length of the
     * path that was traveled.  This assumes that the
     * to-path starts before the from-path's distance,
     * which it should per normal path-state propagation.
     */
    Geometry intersections = from.intersection(to);
    final LineMerger lm = new LineMerger();
    lm.add(intersections);
    intersections =
        JTSFactoryFinder.getGeometryFactory()
            .buildGeometry(lm.getMergedLineStrings());
    Geometry endIntersection = null;
    for (int i = intersections.getNumGeometries() - 1; i > -1; i--) {
      final Geometry match = intersections.getGeometryN(i);
      if (match instanceof LineString) {
        endIntersection = match;
        break;
      }
    }
  
    if (endIntersection == null)
      return null;
  
    /*
     * Always use the last intersection found so that
     * we can compute positive movement.
     */
    final LengthIndexedLine fromLil =
        new LengthIndexedLine(from);
    final double[] fromLocs =
        fromLil.indicesOf(endIntersection);
  
    /*
     * Now, we need to know if the intersection is going
     * in the same direction on our to-path as our from-path. 
     */
    boolean toIsReversed;
    LocationIndexedLine toLocIdx =
        new LocationIndexedLine(to);
    LinearLocation[] toIntxLocs;
    try {
      toIntxLocs = toLocIdx.indicesOf(endIntersection);
  
      final Geometry intxLineOnTo =
          toLocIdx.extractLine(toIntxLocs[0],
              toIntxLocs[toIntxLocs.length - 1]);
      if (intxLineOnTo.equalsExact(endIntersection)) {
        toIsReversed = false;
      } else {
        final LinearLocation[] toIntxLocsRev =
            toLocIdx.indicesOf(endIntersection.reverse());
        final Geometry intxLineOnToRev =
            toLocIdx.extractLine(toIntxLocsRev[0],
                toIntxLocsRev[toIntxLocsRev.length - 1]);
        if (intxLineOnToRev.reverse().equalsExact(
            endIntersection)) {
          to = to.reverse();
          toIsReversed = true;
          distTo = to.getLength() - distTo;
        } else {
          return null;
        }
      }
  
    } catch (final AssertionFailedException ex) {
      /*
       * FIXME: terrible hack
       */
      to = to.reverse();
      toIsReversed = true;
      distTo = to.getLength() - distTo;
      toLocIdx = new LocationIndexedLine(to);
      toIntxLocs = toLocIdx.indicesOf(endIntersection);
    }
  
    final LengthIndexedLine toLil =
        new LengthIndexedLine(to);
    final double[] toLocs =
        toLil.indicesOf(endIntersection);
  
    /*
     * Now, we cut the paths by the last intersection
     * point in direction of the distance along the path. 
     */
    final int fromEndIdx = fromLocs.length - 1;
    final Geometry fromPart;
    if (distFrom <= fromLocs[fromEndIdx]) {
      fromPart =
          fromLil.extractLine(0, fromLocs[fromEndIdx]);
    } else {
      fromPart =
          fromLil.extractLine(fromLocs[fromEndIdx],
              from.getLength());
    }
  
    /*
     * This can occur when the to-geom covers the from-geom.
     */
    if (fromPart.isEmpty()) {
      final Coordinate[] coords =
          CoordinateArrays.removeRepeatedPoints(to
              .getCoordinates());
      if (toIsReversed)
        CoordinateArrays.reverse(coords);
      return new PathUtils.PathMergeResults(JTSFactoryFinder
          .getGeometryFactory().createLineString(coords),
          toIsReversed);
    }
  
    final int toEndIdx = toLocs.length - 1;
    final Geometry toPart;
    if (distTo <= toLocs[0]) {
      toPart = toLil.extractLine(0, toLocs[toEndIdx]);
    } else {
      toPart =
          toLil.extractLine(toLocs[toEndIdx],
              to.getLength());
    }
  
    if (toPart.isEmpty()) {
      final Coordinate[] coords =
          CoordinateArrays.removeRepeatedPoints(from
              .getCoordinates());
      return new PathUtils.PathMergeResults(JTSFactoryFinder
          .getGeometryFactory().createLineString(coords),
          toIsReversed);
    }
  
    final Coordinate[] merged;
    final Coordinate fromPartStartCoord =
        fromPart.getCoordinates()[0];
    final Coordinate fromPartEndCoord =
        fromPart.getCoordinates()[fromPart.getNumPoints() - 1];
    final Coordinate toPartStartCoord =
        toPart.getCoordinates()[0];
    final Coordinate toPartEndCoord =
        toPart.getCoordinates()[toPart.getNumPoints() - 1];
  
    if (fromPartEndCoord.equals(toPartStartCoord)) {
      merged =
          ArrayUtils.addAll(fromPart.getCoordinates(),
              toPart.getCoordinates());
    } else if (toPartEndCoord.equals(fromPartStartCoord)) {
      merged =
          ArrayUtils.addAll(toPart.getCoordinates(),
              fromPart.getCoordinates());
    } else if (fromPartStartCoord.equals(toPartStartCoord)) {
      final Geometry interTest =
          fromPart.intersection(toPart);
      if (interTest instanceof Point) {
        toIsReversed = !toIsReversed;
        ArrayUtils.reverse(toPart.getCoordinates());
        merged =
            ArrayUtils.addAll(fromPart.getCoordinates(),
                toPart.getCoordinates());
      } else {
        if (fromPart.getLength() > toPart.getLength()) {
          merged = fromPart.getCoordinates();
        } else {
          merged = toPart.getCoordinates();
        }
      }
    } else if (fromPartEndCoord.equals(toPartEndCoord)) {
      toIsReversed = !toIsReversed;
      ArrayUtils.reverse(toPart.getCoordinates());
      merged =
          ArrayUtils.addAll(fromPart.getCoordinates(),
              toPart.getCoordinates());
    } else {
      return null;
    }
  
    final Coordinate[] coords =
        CoordinateArrays.removeRepeatedPoints(merged);
    return new PathUtils.PathMergeResults(JTSFactoryFinder
        .getGeometryFactory().createLineString(coords),
        toIsReversed);
  
  }

  /**
   * This method returns the start component index for a LinearLocation on a
   * multi-component/geometry geom when the location is on the end of the
   * component before.
   * 
   * @param loc
   * @param geom
   * @return
   */
  static public int geomIndexOf(LinearLocation loc,
    Geometry geom) {
    final Geometry firstComponent =
        geom.getGeometryN(loc.getComponentIndex());
    final int adjIndex;
    if (firstComponent.getNumPoints() - 1 == loc
        .getSegmentIndex()
        && loc.getComponentIndex() + 1 < geom
            .getNumGeometries()) {
      adjIndex = loc.getComponentIndex() + 1;
    } else {
      adjIndex = loc.getComponentIndex();
    }
  
    return adjIndex;
  }

  public static Vector stateDiff(PathState fromState, PathState toState, 
    boolean useRaw) {
    
    if (toState.isOnRoad() && fromState.isOnRoad()) {
  
      final InferredPath toPath = toState.getPath();
      final PathEdge toFirstEdge =
          Iterables.getFirst(toPath.getPathEdges(), null);
      final PathEdge toLastEdge =
          Iterables.getLast(toPath.getPathEdges());
      final Geometry toFirstActualGeom =
          toPath.isBackward() ? toFirstEdge
              .getGeometry().reverse() : toFirstEdge
              .getGeometry();
      final Geometry toLastActualGeom;
      if (toPath.getPathEdges().size() > 1) {
        toLastActualGeom =
            toPath.isBackward() ? toLastEdge
                .getGeometry().reverse() : toLastEdge
                .getGeometry();
      } else {
        toLastActualGeom =
            JTSFactoryFinder.getGeometryFactory()
                .createLineString(new Coordinate[0]);
      }
  
      final PathEdge fromFirstEdge =
          Iterables.getFirst(fromState.getPath().getPathEdges(),
              null);
      final PathEdge fromLastEdge =
          Iterables.getLast(fromState.getPath().getPathEdges(),
              null);
      final InferredPath fromPath = fromState.getPath();
      final Geometry fromFirstActualGeom =
          fromPath.isBackward() ? fromFirstEdge
              .getGeometry().reverse() : fromFirstEdge
              .getGeometry();
  
      final Geometry fromLastActualGeom;
      if (fromPath.getPathEdges().size() > 1) {
        fromLastActualGeom =
            fromPath.isBackward() ? fromLastEdge
                .getGeometry().reverse() : fromLastEdge
                .getGeometry();
      } else {
        fromLastActualGeom =
            JTSFactoryFinder.getGeometryFactory()
                .createLineString(new Coordinate[0]);
      }
  
      final Vector result;
      final double distanceMax;
  
      final Vector toStateVec =
          useRaw ? toState.getRawState() :toState 
              .getGlobalState();
      final Vector fromStateVec =
          useRaw ? fromState.getRawState() : fromState
              .getGlobalState();
          
      if (fromLastActualGeom
          .equalsExact(toFirstActualGeom)) {
        /*
         * Head-to-tail
         */
        result =
            PathUtils.headToTailDiff(
                toStateVec,
                toPath.isBackward(),
                toFirstEdge.getGeometry(),
                fromStateVec,
                fromState.getEdge().getDistToStartOfEdge(),
                fromLastEdge.getGeometry());
  
        distanceMax =
            Math.abs(fromPath.getTotalPathDistance())
                + Math
                    .abs(toPath.getTotalPathDistance())
                - fromLastActualGeom.getLength();
  
      } else if (fromFirstActualGeom
          .equalsExact(toFirstActualGeom)) {
        /*
         * Same start, same path-directions.
         */
        final Vector fromVec;
        if (toPath.isBackward() == fromPath
            .isBackward()) {
          fromVec = fromStateVec;
        } else {
          fromVec = fromStateVec.scale(-1d);
        }
  
        result = toStateVec.minus(fromVec);
  
        distanceMax =
            Math.max(Math.abs(fromPath
                .getTotalPathDistance()), Math
                .abs(toPath.getTotalPathDistance()));
        
      } else if (InferredPath.biDirComp.compare(fromLastActualGeom.getCoordinates(),
          toFirstActualGeom.getCoordinates()) == 0) {
        /*
         * Head-to-tail, but in opposite path directions.
         */
        result =
            PathUtils.headToTailRevDiff(toState, fromState, useRaw);
  
        distanceMax =
            Math.abs(fromPath.getTotalPathDistance())
                + Math
                    .abs(toPath.getTotalPathDistance())
                - fromLastActualGeom.getLength();
  
  
      } else if (InferredPath.biDirComp.compare(fromFirstActualGeom.getCoordinates(), 
          toFirstActualGeom.getCoordinates()) == 0) {
        /*
         * Going in opposite path-directions from the same
         * starting location. 
         */
        final double adjustedLocation =
            -1d
                * (Math.abs(fromStateVec.getElement(0)) - fromFirstEdge
                    .getLength());
        final double distDiff =
            (toPath.isBackward() ? -1d : 1d)
                * (Math.abs(toStateVec.getElement(0)) - adjustedLocation);
  
        final double toVel = toStateVec.getElement(1);
        final double fromVel =
            (fromFirstEdge.getGeometry().equalsExact(
                toFirstEdge.getGeometry()) ? 1d : -1d)
                * fromStateVec.getElement(1);
        final double velDiff = toVel - fromVel;
  
        result =
            VectorFactory.getDenseDefault().createVector2D(
                distDiff, velDiff);
  
        distanceMax =
            Math.max(Math.abs(fromPath
                .getTotalPathDistance()), Math
                .abs(toPath.getTotalPathDistance()));
  
      } else if (fromFirstActualGeom.equalsExact(
          toLastActualGeom)) {
        /*
         * Head-to-tail, reversed from-to
         */
        final Vector fromVec;
        if (toPath.isBackward() == fromPath
            .isBackward()) {
          fromVec = fromStateVec.clone();
        } else {
          fromVec = fromStateVec.scale(-1d);
        }
        fromVec.setElement(0, fromVec.getElement(0)
            + toLastEdge.getDistToStartOfEdge());
        
  
        result = toStateVec.minus(fromVec);
  
        distanceMax =
            Math.max(Math.abs(fromPath
                .getTotalPathDistance()), Math
                .abs(toPath.getTotalPathDistance()));
        
      } else {
        throw new IllegalStateException();
      }
  
      /*
       * Distance upper-bound requirement
       */
      assert Preconditions.checkNotNull((useRaw || Math
          .abs(result.getElement(0)) - distanceMax <= 1d)
          ? true : null);
  
      /*
       * Distance/velocity lower-bound requirement
       */
      assert Preconditions.checkNotNull(Math.min(
          toState.getGroundState()
              .minus(fromState.getGroundState())
              .norm2Squared()
              - result.norm2Squared(), 0d) <= 1d ? true
          : null);
  
      return result;
    } else {
      return toState.getGroundState().minus(
          fromState.getGroundState());
    }
  }

  public static Vector headToTailRevDiff(
    PathState thisState,
    PathState otherState, boolean useRaw) {
    /*
     * Flip the other state around so that it's
     * going the same direction as this state.
     */
    final double thisDir =
        thisState.getPath().isBackward() ? -1d : 1d;
    final double otherDir =
        otherState.getPath().isBackward() ? -1d : 1d;
    final Vector otherStateVec =
        useRaw ? otherState.getRawState() : otherState
            .getGlobalState();
    final Vector thisStateVec =
        useRaw ? thisState.getRawState() : thisState
            .getGlobalState();
    final double otherDist =
        (thisState.getPath().isBackward() ? -1d : 1d)
            * (Math.abs(otherState.getPath()
                .getTotalPathDistance()) - Math
                .abs(otherStateVec.getElement(0)));
    
    /*
     * Normed velocities (normed means sign 
     * is positive for motion in the direction of geom).
     */
    final double otherVelNormRev =
        -1d * otherDir * otherStateVec.getElement(1);
    final double thisVelNorm =
        thisDir * thisStateVec.getElement(1);
    final double relVelDiff =
        thisDir * (thisVelNorm - otherVelNormRev);
    return VectorFactory.getDenseDefault().createVector2D(
        thisStateVec.getElement(0) - otherDist, relVelDiff);
  }

  public static Vector
      headToTailDiff(Vector toState,
        boolean toStateIsBackward,
        Geometry toStartEdgeGeom, Vector fromState,
        double fromStateDistToStart,
        Geometry fromLastEdgeGeom) {
  
    final double toStateSign =
        toStateIsBackward ? -1d : 1d;
  
    /*
     * The following distance is fromState's
     * distance along the path but flipped to correspond to
     * movement opposite of toState, and the origin
     * is set to the start of toState's path. 
     */
    final double fromFlipDist =
        Math.abs(fromState.getElement(0))
            - Math.abs(fromStateDistToStart);
  
    final double toDist =
        Math.abs(toState.getElement(0));
    final double lengthDiff =
        toStateSign * (toDist - fromFlipDist);
  
    final double toVel = toState.getElement(1);
  
    final double fromVel =
        (fromLastEdgeGeom.equalsExact(toStartEdgeGeom)
            ? 1d : -1d) * fromState.getElement(1);
  
    final double velocityDiff = toVel - fromVel;
  
    return VectorFactory.getDefault().createVector2D(
        lengthDiff, velocityDiff);
  }

}
