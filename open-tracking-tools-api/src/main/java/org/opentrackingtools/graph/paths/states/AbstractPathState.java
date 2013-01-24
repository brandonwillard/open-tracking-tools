package org.opentrackingtools.graph.paths.states;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import org.apache.commons.lang3.ArrayUtils;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.opentrackingtools.graph.paths.InferredPath;
import org.opentrackingtools.graph.paths.edges.PathEdge;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;
import com.vividsolutions.jts.operation.linemerge.LineMerger;
import com.vividsolutions.jts.util.AssertionFailedException;

public abstract class AbstractPathState extends
    AbstractCloneableSerializable implements PathState {

  public static class PathMergeResults {
    final Geometry path;
    final boolean toIsReversed;

    protected PathMergeResults(Geometry path,
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

  private static final long serialVersionUID =
      -5009638378510031502L;

  protected InferredPath path;

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getGroundState()
   */
  @Override
  public abstract Vector getGroundState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getGlobalState()
   */
  @Override
  public abstract Vector getGlobalState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getLocalState()
   */
  @Override
  public abstract Vector getLocalState();

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getEdge()
   */
  @Override
  public abstract PathEdge getEdge();

  protected PathEdge edge;

  public AbstractPathState() {
    super();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getPath()
   */
  @Override
  public InferredPath getPath() {
    return path;
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#isOnRoad()
   */
  @Override
  public boolean isOnRoad() {
    return !this.path.isNullPath();
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
  public static PathMergeResults mergePaths(Geometry from,
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
      return new PathMergeResults(JTSFactoryFinder
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
      return new PathMergeResults(JTSFactoryFinder
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
    return new PathMergeResults(JTSFactoryFinder
        .getGeometryFactory().createLineString(coords),
        toIsReversed);

  }

  final boolean useRaw = false;

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#minus(org.opentrackingtools.graph.paths.states.AbstractPathState)
   */
  @Override
  public Vector minus(PathState otherState) {
    if (this.isOnRoad() && otherState.isOnRoad()) {

      final PathEdge thisFirstEdge =
          Iterables.getFirst(this.path.getEdges(), null);
      final Geometry thisFirstActualGeom =
          this.path.getIsBackward() ? thisFirstEdge
              .getGeometry().reverse() : thisFirstEdge
              .getGeometry();

      final PathEdge otherFirstEdge =
          Iterables.getFirst(otherState.getPath().getEdges(),
              null);
      final PathEdge otherLastEdge =
          Iterables.getLast(otherState.getPath().getEdges(),
              null);
      final Geometry otherFirstActualGeom =
          otherState.getPath().getIsBackward() ? otherFirstEdge
              .getGeometry().reverse() : otherFirstEdge
              .getGeometry();

      final Geometry otherLastActualGeom;
      if (otherState.getPath().getEdges().size() > 1) {
        otherLastActualGeom =
            otherState.getPath().getIsBackward() ? otherLastEdge
                .getGeometry().reverse() : otherLastEdge
                .getGeometry();
      } else {
        otherLastActualGeom =
            JTSFactoryFinder.getGeometryFactory()
                .createLineString(new Coordinate[0]);
      }

      final Vector result;
      final double distanceMax;

      final Vector thisStateVec =
          useRaw ? this.getRawState() : this
              .getGlobalState();
      final Vector otherStateVec =
          useRaw ? otherState.getRawState() : otherState
              .getGlobalState();

      if (otherLastActualGeom
          .equalsExact(thisFirstActualGeom)) {
        /*
         * Head-to-tail
         */
        result =
            headToTailDiff(
                thisStateVec,
                this.path.getIsBackward(),
                thisFirstEdge.getGeometry(),
                otherStateVec,
                otherState.getEdge().getDistToStartOfEdge(),
                otherLastEdge.getGeometry());

        distanceMax =
            Math.abs(otherState.getPath().getTotalPathDistance())
                + Math
                    .abs(this.path.getTotalPathDistance())
                - otherLastActualGeom.getLength();

      } else if (otherLastActualGeom
          .equalsTopo(thisFirstActualGeom)) {
        /*
         * Head-to-tail, but in opposite path directions.
         */
        result =
            headToTailRevDiff(this, otherState, useRaw);

        distanceMax =
            Math.abs(otherState.getPath().getTotalPathDistance())
                + Math
                    .abs(this.path.getTotalPathDistance())
                - otherLastActualGeom.getLength();

        //      } else if (otherFirstActualGeom.equalsExact(
        //        thisLastActualGeom)){
        //        
        //        /*
        //         * Tail-to-head
        //         */
        //        result = headToTailDiff(
        //              otherStateVec, 
        //              otherState.path.getIsBackward(),
        //              otherFirstEdge.getGeometry(),
        //              thisStateVec,
        //              this.getEdge().getDistToStartOfEdge(),
        //              thisLastEdge.getGeometry());
        ////            .scale(-1d); 
        //        
        //        distanceMax = 
        //            Math.abs(otherState.path.getTotalPathDistance())
        //            + Math.abs(this.path.getTotalPathDistance())
        //            - otherFirstActualGeom.getLength(); 
        //        
        //      } else if (otherFirstActualGeom.equalsTopo(
        //        thisLastActualGeom)){
        //        
        //        /*
        //         * Tail-to-head, but moving opposite path-directions.
        //         * TODO FIXME what to do?
        //         */
        //        if (otherFirstEdge.isOnEdge(
        //            otherStateVec.getElement(0))) {
        //          result = headToTailDiff(
        //              otherStateVec, 
        //              otherState.path.getIsBackward(),
        //              otherFirstEdge.getGeometry(),
        //              thisStateVec,
        //              this.getEdge().getDistToStartOfEdge(),
        //              thisLastEdge.getGeometry());
        ////              .scale(-1d); 
        //        } else {
        //          throw new IllegalStateException();
        //        }
        //        
        //        distanceMax = 
        //            Math.abs(otherState.path.getTotalPathDistance())
        //            + Math.abs(this.path.getTotalPathDistance())
        //            - otherFirstActualGeom.getLength(); 

      } else if (otherFirstActualGeom
          .equalsExact(thisFirstActualGeom)) {
        /*
         * Same start, same path-directions.
         */
        final Vector otherVec;
        if (this.path.getIsBackward() == otherState.getPath()
            .getIsBackward()) {
          otherVec = otherStateVec;
        } else {
          otherVec = otherStateVec.scale(-1d);
        }

        result = thisStateVec.minus(otherVec);

        distanceMax =
            Math.max(Math.abs(otherState.getPath()
                .getTotalPathDistance()), Math
                .abs(this.path.getTotalPathDistance()));

      } else if (otherFirstActualGeom
          .equalsTopo(thisFirstActualGeom)) {
        /*
         * Going in opposite path-directions from the same
         * starting location. 
         */
        final double adjustedLocation =
            -1d
                * (Math.abs(otherStateVec.getElement(0)) - otherFirstEdge
                    .getLength());
        final double distDiff =
            (this.path.getIsBackward() ? -1d : 1d)
                * (Math.abs(thisStateVec.getElement(0)) - adjustedLocation);

        final double thisVel = thisStateVec.getElement(1);
        final double thatVel =
            (otherFirstEdge.getGeometry().equalsExact(
                thisFirstEdge.getGeometry()) ? 1d : -1d)
                * otherStateVec.getElement(1);
        final double velDiff = thisVel - thatVel;

        result =
            VectorFactory.getDenseDefault().createVector2D(
                distDiff, velDiff);

        distanceMax =
            Math.max(Math.abs(otherState.getPath()
                .getTotalPathDistance()), Math
                .abs(this.path.getTotalPathDistance()));

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
          this.getGroundState()
              .minus(otherState.getGroundState())
              .norm2Squared()
              - result.norm2Squared(), 0d) <= 1d ? true
          : null);

      return result;
    } else {
      return this.getGroundState().minus(
          otherState.getGroundState());
    }
  }

  private static Vector headToTailRevDiff(
    PathState thisState,
    PathState otherState, boolean useRaw) {
    /*
     * Flip the other state around so that it's
     * going the same direction as this state.
     */
    final double thisDir =
        thisState.getPath().getIsBackward() ? -1d : 1d;
    final double otherDir =
        otherState.getPath().getIsBackward() ? -1d : 1d;
    final Vector otherStateVec =
        useRaw ? otherState.getRawState() : otherState
            .getGlobalState();
    final Vector thisStateVec =
        useRaw ? thisState.getRawState() : thisState
            .getGlobalState();
    final double otherDist =
        (thisState.getPath().getIsBackward() ? -1d : 1d)
            * (Math.abs(otherState.getPath()
                .getTotalPathDistance()) - Math
                .abs(otherStateVec.getElement(0)));
    //        (otherState.getEdge().getLength()
    //        + Math.abs(otherState.getEdge().getDistToStartOfEdge())
    //        - Math.abs(otherStateVec.getElement(0)));
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

  private static Vector
      headToTailDiff(Vector thisState,
        boolean thisStateIsBackward,
        Geometry thisStartEdgeGeom, Vector otherState,
        double otherStateDistToStart,
        Geometry otherLastEdgeGeom) {

    final double thisStateSign =
        thisStateIsBackward ? -1d : 1d;

    /*
     * The following distance is otherState's
     * distance along the path but flipped to correspond to
     * movement opposite of thisState, and the origin
     * is set to the start of thisState's path. 
     */
    final double thatFlipDist =
        Math.abs(otherState.getElement(0))
            - Math.abs(otherStateDistToStart);

    final double thisDist =
        Math.abs(thisState.getElement(0));
    final double lengthDiff =
        thisStateSign * (thisDist - thatFlipDist);

    final double thisVel = thisState.getElement(1);

    final double thatVel =
        (otherLastEdgeGeom.equalsExact(thisStartEdgeGeom)
            ? 1d : -1d) * otherState.getElement(1);

    final double velocityDiff = thisVel - thatVel;

    return VectorFactory.getDefault().createVector2D(
        lengthDiff, velocityDiff);
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

  @Override
  public AbstractPathState clone() {
    final AbstractPathState clone =
        (AbstractPathState) super.clone();
    clone.edge = this.edge;
    clone.path = this.path;
    return clone;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + ((path == null) ? 0 : path.hashCode());
    result =
        prime
            * result
            + ((getRawState() == null) ? 0
                : ((AbstractVector) getRawState())
                    .hashCode());
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
    final AbstractPathState other = (AbstractPathState) obj;
    if (path == null) {
      if (other.path != null) {
        return false;
      }
    } else if (!path.equals(other.path)) {
      return false;
    }

    if (getRawState() == null) {
      if (other.getRawState() != null) {
        return false;
      }
    } else if (!((AbstractVector) getRawState())
        .equals((other.getRawState()))) {
      return false;
    }
    return true;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("AbstractPathState [path=").append(path)
        .append(", state=").append(this.getGlobalState())
        .append("]");
    return builder.toString();
  }

  /* (non-Javadoc)
   * @see org.opentrackingtools.graph.paths.states.PathState#getRawState()
   */
  @Override
  abstract public Vector getRawState();

}