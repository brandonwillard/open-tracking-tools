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

import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.SimplePathEdge;
import org.opentrackingtools.graph.paths.impl.SimpleInferredPath;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter.PathEdgeProjection;
import org.opentrackingtools.statistics.impl.StatisticsUtil;
import org.opentrackingtools.util.GeoUtils;

import com.google.common.base.Preconditions;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineSegment;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;

public class PathUtils {

  /**
   * Checks that the state is either on- or off-road, depending
   * on the given path, and converts if necessary.
   * 
   * @param state
   * @param path
   * @return
   */
  public static Vector checkAndConvertState(Vector state,
    InferredPath path) {
    
    final Vector adjState;
    if (path.isNullPath() && state.getDimensionality() != 4) {
      
      final double dist = 
          AbstractRoadTrackingFilter.getOr().times(state).getElement(0);
      final PathEdge edge = path.getEdgeForDistance(dist, false);
      
      adjState = getGroundStateFromRoad(state, edge, false);
  
    } else if (!path.isNullPath() && state.getDimensionality() != 2) {
      
      adjState = getRoadStateFromGround(state, path, false);
    } else {
      adjState = state;
    }
    
    return adjState;
  }

  /**
   * See {@link #checkAndConvertState(Vector, InferredPath)}
   * @param belief
   * @param path
   * @return
   */
  public static MultivariateGaussian checkAndConvertBelief(MultivariateGaussian belief,
    InferredPath path) {
    
    final MultivariateGaussian adjBelief;
    if (path.isNullPath() && belief.getInputDimensionality() != 4) {
      
      final double dist = 
          AbstractRoadTrackingFilter.getOr().times(belief.getMean()).getElement(0);
      final PathEdge edge = path.getEdgeForDistance(dist, false);
      
      adjBelief = getGroundBeliefFromRoad(belief, edge, false);
  
    } else if (!path.isNullPath() && belief.getInputDimensionality() != 2) {
      
      adjBelief = getRoadBeliefFromGround(belief, path, false);
    } else {
      if (!path.isNullPath()) {
        Preconditions.checkState(
          path.isOnPath(
              AbstractRoadTrackingFilter.getOr()
              .times(belief.getMean()).getElement(0)));
      }
      adjBelief = belief;
    }
    
    return adjBelief;
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
            path.isBackward(), null, 0, false);
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
            edge.isBackward(), null, 0, true);
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
                AbstractRoadTrackingFilter.getVg().times(projMean));
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
        PathUtils.posVelProjectionPair(
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
        path.getGeometry(), path.isBackward(), pathEdge.getGeometry(), 
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
                projPair.getPositiveState().minus(
                    projPair.getOffset()));

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
                projPair.getPositiveState().minus(
                    projPair.getOffset()));

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
                  pathEdge.getInferredEdge(), 0d,
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
        GeoUtils.makeCoordinate(AbstractRoadTrackingFilter.getOg().times(m));
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
        PathUtils.posVelProjectionPair(
            pathLineSegment,
            distanceToStartOfSegmentOnGeometry);
  
    return new PathEdgeProjection(projPair, m, null);
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
  public static Entry<Matrix, Vector>
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
    P.setColumn(0, P1.stack(AbstractRoadTrackingFilter.getZeros2d()));
    P.setColumn(1, AbstractRoadTrackingFilter.getZeros2d().stack(P1));
  
    final Vector a = s1.stack(AbstractRoadTrackingFilter.getZeros2d());
  
    return Maps.immutableEntry(AbstractRoadTrackingFilter.getU().times(P), 
        AbstractRoadTrackingFilter.getU().times(a));
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
    return new MultivariateGaussian(y, Sigma);
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
              AbstractRoadTrackingFilter.getVg().times(projMean));
        velocities.scaleEquals(absVelocity
            / velocities.norm2());
        projMean.setElement(1, velocities.getElement(0));
        projMean.setElement(3, velocities.getElement(1));
      }
    }
  
    assert isIsoMapping(locVelocity, projMean, edge);
  
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

}
