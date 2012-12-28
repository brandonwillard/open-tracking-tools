package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.AbstractVector;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.geotools.geometry.jts.coordinatesequence.CoordinateSequences;
import org.jaitools.jts.CoordinateSequence2D;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;
import org.openplans.tools.tracking.impl.util.EdgeOverlayOp;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Objects;
import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.CoordinateArrays;
import com.vividsolutions.jts.geom.CoordinateSequence;
import com.vividsolutions.jts.geom.CoordinateSequenceComparator;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.geom.Point;
import com.vividsolutions.jts.linearref.LengthIndexedLine;
import com.vividsolutions.jts.linearref.LengthLocationMap;
import com.vividsolutions.jts.linearref.LinearGeometryBuilder;
import com.vividsolutions.jts.linearref.LinearLocation;
import com.vividsolutions.jts.linearref.LocationIndexedLine;
import com.vividsolutions.jts.operation.linemerge.LineMergeGraph;
import com.vividsolutions.jts.operation.linemerge.LineMerger;
import com.vividsolutions.jts.operation.linemerge.LineSequencer;
import com.vividsolutions.jts.util.AssertionFailedException;

public abstract class AbstractPathState extends
    AbstractCloneableSerializable {

  public static class PathMergeResults {
    final Geometry path;
    final boolean toIsReversed;
    
    protected PathMergeResults(Geometry path, boolean toIsReversed) {
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

  private static final long serialVersionUID = -5009638378510031502L;
  
  protected InferredPath path;

  public abstract Vector getGroundState();

  /**
   * The state relative to its path if it's on a road,
   * otherwise the off-road state.
   * @return
   */
  public abstract Vector getGlobalState();
  
  /**
   * The state relative to its edge if it's on a road,
   * otherwise the off-road state.
   * @return
   */
  public abstract Vector getLocalState();
  
  public abstract PathEdge getEdge();

  protected PathEdge edge;

  public AbstractPathState() {
    super();
  }

  public InferredPath getPath() {
    return path;
  }

  public boolean isOnRoad() {
    return !this.path.isEmptyPath();
  }
  
  /**
   * Returns the path connecting the two passed geoms
   * and distances.
   * XXX: assumes that from end and to start edges overlap.
   * @param from
   * @param distFrom
   * @param inTo
   * @param inDistTo
   * @return
   */
  public static PathMergeResults mergePaths(Geometry from, double distFrom, 
    Geometry to, double distTo) {
    
    /*
     * Make sure we're only dealing with the length of the
     * path that was traveled.  This assumes that the
     * to-path starts before the from-path's distance,
     * which it should per normal path-state propagation.
     */
    Geometry intersections = from.intersection(to);
    LineMerger lm = new LineMerger();
    lm.add(intersections);
    intersections = JTSFactoryFinder.getGeometryFactory().buildGeometry(
        lm.getMergedLineStrings()); 
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
    final LengthIndexedLine fromLil = new LengthIndexedLine(from);
    double[] fromLocs = fromLil.indicesOf(endIntersection);
    
    /*
     * Now, we need to know if the intersection is going
     * in the same direction on our to-path as our from-path. 
     */
    boolean toIsReversed;
    LocationIndexedLine toLocIdx = new LocationIndexedLine(to);
    LinearLocation[] toIntxLocs;
    try {
      toIntxLocs = toLocIdx.indicesOf(endIntersection);
      
      Geometry intxLineOnTo = toLocIdx.extractLine(toIntxLocs[0], 
        toIntxLocs[toIntxLocs.length - 1]);
      if (intxLineOnTo.equalsExact(endIntersection)) {
        toIsReversed = false;
      } else { 
        LinearLocation[] toIntxLocsRev = toLocIdx.indicesOf(endIntersection.reverse());
        Geometry intxLineOnToRev = toLocIdx.extractLine(toIntxLocsRev[0], 
            toIntxLocsRev[toIntxLocsRev.length - 1]);
        if (intxLineOnToRev.reverse().equalsExact(endIntersection)) {
          to = to.reverse();
          toIsReversed = true;
          distTo = to.getLength() - distTo;
        } else {
          return null;
        }
      } 
    
    } catch (AssertionFailedException ex) {
      /*
       * FIXME: terrible hack
       */
      to = to.reverse();
      toIsReversed = true;
      distTo = to.getLength() - distTo;
      toLocIdx = new LocationIndexedLine(to);
      toIntxLocs = toLocIdx.indicesOf(endIntersection);
    }
    
    LengthIndexedLine toLil = new LengthIndexedLine(to);
    double[] toLocs = toLil.indicesOf(endIntersection);   
    
    /*
     * Now, we cut the paths by the last intersection
     * point in direction of the distance along the path. 
     */
    final int fromEndIdx = fromLocs.length - 1;
    final Geometry fromPart;
    if (distFrom <= fromLocs[fromEndIdx]) {
      fromPart = fromLil.extractLine(0, fromLocs[fromEndIdx]);
    } else {
      fromPart = fromLil.extractLine(fromLocs[fromEndIdx], 
          from.getLength());
    }
    
    /*
     * This can occur when the to-geom covers the from-geom.
     */
    if (fromPart.isEmpty()) {
      Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(
         to.getCoordinates());
      if (toIsReversed)
        CoordinateArrays.reverse(coords);
      return new PathMergeResults(
          JTSFactoryFinder.getGeometryFactory().createLineString(
          coords), toIsReversed);
    }
    
    final int toEndIdx = toLocs.length - 1;
    final Geometry toPart;
    if (distTo <= toLocs[0]) {
      toPart = toLil.extractLine(0, toLocs[toEndIdx]);
    } else {
      toPart = toLil.extractLine(toLocs[toEndIdx], 
          to.getLength());
    }
    
    if (toPart.isEmpty()) {
      Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(
          from.getCoordinates());
      return new PathMergeResults(
          JTSFactoryFinder.getGeometryFactory().createLineString(
          coords), toIsReversed);
    }

    final Coordinate[] merged;
    final Coordinate fromPartStartCoord = fromPart.getCoordinates()
        [0];
    final Coordinate fromPartEndCoord = fromPart.getCoordinates()
        [fromPart.getNumPoints() - 1];
    final Coordinate toPartStartCoord = toPart.getCoordinates()
        [0];
    final Coordinate toPartEndCoord = toPart.getCoordinates()
        [toPart.getNumPoints() - 1];
    
    if (fromPartEndCoord.equals(toPartStartCoord)) {
      merged = ArrayUtils.addAll(fromPart.getCoordinates(), 
          toPart.getCoordinates());
    } else if (toPartEndCoord.equals(fromPartStartCoord)){
      merged = ArrayUtils.addAll(toPart.getCoordinates(), 
          fromPart.getCoordinates());
    } else if (fromPartStartCoord.equals(toPartStartCoord)) {
      final Geometry interTest = fromPart.intersection(toPart);
      if (interTest instanceof Point) {
        toIsReversed = !toIsReversed;
        ArrayUtils.reverse(toPart.getCoordinates());
        merged = ArrayUtils.addAll(fromPart.getCoordinates(), 
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
      merged = ArrayUtils.addAll(fromPart.getCoordinates(), 
          toPart.getCoordinates());
    } else {
      return null;
    }
    
    Coordinate[] coords = CoordinateArrays.removeRepeatedPoints(
        merged);
    return new PathMergeResults(
        JTSFactoryFinder.getGeometryFactory().createLineString(
        coords), toIsReversed);
          
  }
  
  final boolean useRaw = false;
  
  /**
   * 
   * Yields the difference between distance and velocities
   * between this state and otherState.
   * 
   * Important: The allowed movements for differencing are 
   * <ul>
   * <li>
   * Both states starting at the same origin, or
   * </li>
   * <li>
   * this state leaving from the other one's location.
   * </li>
   * </ul>
   * Also, raw state vectors (not truncated to the path) are used.
   * @param otherState
   * @return
   */
  public Vector minus(AbstractPathState otherState) {
    if (this.isOnRoad() && otherState.isOnRoad()) {
      
      PathEdge thisFirstEdge = Iterables.getFirst(
          this.path.getEdges(), null);
      final Geometry thisFirstActualGeom = 
          this.path.getIsBackward() ?
          thisFirstEdge.getGeometry().reverse() :
          thisFirstEdge.getGeometry();
      PathEdge thisLastEdge = 
          Iterables.getLast(this.path.getEdges(), null);
      
      final Geometry thisLastActualGeom;
      if (this.path.getEdges().size() > 1) {
        thisLastActualGeom = 
            this.path.getIsBackward() ?
            thisLastEdge.getGeometry().reverse() :
            thisLastEdge.getGeometry();
      } else {
        thisLastActualGeom = JTSFactoryFinder.getGeometryFactory().createLineString(
            new Coordinate[0]);
      }
          
      PathEdge otherFirstEdge = Iterables.getFirst(
          otherState.path.getEdges(), null);
      PathEdge otherLastEdge = 
          Iterables.getLast(otherState.path.getEdges(), null);
      final Geometry otherFirstActualGeom = 
          otherState.path.getIsBackward() ?
          otherFirstEdge.getGeometry().reverse() :
          otherFirstEdge.getGeometry();
          
      final Geometry otherLastActualGeom;
      if (otherState.path.getEdges().size() > 1) {
        otherLastActualGeom = 
          otherState.path.getIsBackward() ?
          otherLastEdge.getGeometry().reverse() :
          otherLastEdge.getGeometry();
      } else {
        otherLastActualGeom = JTSFactoryFinder.getGeometryFactory().createLineString(
            new Coordinate[0]);
      }
      
      final Vector result;
      final double distanceMax;
      
      final Vector thisStateVec =  useRaw ?
          this.getRawState() : this.getGlobalState();
      final Vector otherStateVec = useRaw ?
          otherState.getRawState() : otherState.getGlobalState();
      
      if (otherLastActualGeom.equalsExact(
          thisFirstActualGeom)) {
        /*
         * Head-to-tail
         */
        result = headToTailDiff(
              thisStateVec, 
              this.path.getIsBackward(),
              thisFirstEdge.getGeometry(),
              otherStateVec,
              otherState.getEdge().getDistToStartOfEdge(),
              otherLastEdge.getGeometry());
        
        distanceMax = 
            Math.abs(otherState.path.getTotalPathDistance())
            + Math.abs(this.path.getTotalPathDistance())
            - otherLastActualGeom.getLength(); 
        
      } else if (otherLastActualGeom.equalsTopo(
        thisFirstActualGeom)) {
        /*
         * Head-to-tail, but in opposite path directions.
         */
        result = headToTailRevDiff(this, otherState, useRaw);
        
        distanceMax = 
            Math.abs(otherState.path.getTotalPathDistance())
            + Math.abs(this.path.getTotalPathDistance())
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
        
      } else if (otherFirstActualGeom.equalsExact(
            thisFirstActualGeom)) {
        /*
         * Same start, same path-directions.
         */
        final Vector otherVec;
        if (this.path.getIsBackward() 
            == otherState.path.getIsBackward()) {
          otherVec = otherStateVec;
        } else {
          otherVec = otherStateVec.
              scale(-1d);
        }
        
        result = thisStateVec.minus(otherVec);
        
        distanceMax = Math.max(
            Math.abs(otherState.path.getTotalPathDistance()),
            Math.abs(this.path.getTotalPathDistance())); 
        
      } else if (otherFirstActualGeom.equalsTopo(
          thisFirstActualGeom)) {
        /*
         * Going in opposite path-directions from the same
         * starting location. 
         */
        final double adjustedLocation =
            -1d * (Math.abs(otherStateVec.getElement(0))
              - otherFirstEdge.getLength());
        final double distDiff = 
            (this.path.getIsBackward() ? -1d : 1d) *
            (Math.abs(thisStateVec.getElement(0))
              - adjustedLocation);
        
        double thisVel = thisStateVec.getElement(1);
        double thatVel = (otherFirstEdge.getGeometry().equalsExact(
            thisFirstEdge.getGeometry()) ? 1d : -1d) 
            * otherStateVec.getElement(1);
        final double velDiff = thisVel - thatVel;
        
        result = VectorFactory.getDenseDefault().createVector2D(
            distDiff, velDiff);
        
        distanceMax = Math.max(
            Math.abs(otherState.path.getTotalPathDistance()),
            Math.abs(this.path.getTotalPathDistance())); 
        
      } else {
        throw new IllegalStateException();
      }
      
      /*
       * Distance upper-bound requirement
       */
      assert Preconditions.checkNotNull(
          (useRaw || 
          Math.abs(result.getElement(0)) - distanceMax
            <= 1d) ? true : null);
        
      /*
       * Distance/velocity lower-bound requirement
       */
      assert Preconditions.checkNotNull(
         Math.min(this.getGroundState()
             .minus(otherState.getGroundState())
          .norm2Squared() - result.norm2Squared(), 0d)
            <= 1d ? true : null);
      
      return result;
    } else {
      return this.getGroundState().minus(otherState.getGroundState());
    }
  }
  
  private static Vector headToTailRevDiff(AbstractPathState thisState,
    AbstractPathState otherState, boolean useRaw) {
    /*
     * Flip the other state around so that it's
     * going the same direction as this state.
     */
    final double thisDir = thisState.path.getIsBackward()
        ? -1d : 1d;     
    final double otherDir = otherState.path.getIsBackward()
        ? -1d : 1d;     
    final Vector otherStateVec = useRaw? otherState.getRawState()
        : otherState.getGlobalState();
    final Vector thisStateVec = useRaw? thisState.getRawState()
        : thisState.getGlobalState();
    final double otherDist = 
        (thisState.path.getIsBackward() ? -1d : 1d) *
        (Math.abs(otherState.path.getTotalPathDistance())
        - Math.abs(otherStateVec.getElement(0)));
//        (otherState.getEdge().getLength()
//        + Math.abs(otherState.getEdge().getDistToStartOfEdge())
//        - Math.abs(otherStateVec.getElement(0)));
    /*
     * Normed velocities (normed means sign 
     * is positive for motion in the direction of geom).
     */
    final double otherVelNormRev = -1d * 
        otherDir * otherStateVec.getElement(1);
    final double thisVelNorm =
        thisDir * thisStateVec.getElement(1);
    final double relVelDiff = thisDir * (
        thisVelNorm - otherVelNormRev);
    return VectorFactory.getDenseDefault().createVector2D(
        thisStateVec.getElement(0) - otherDist, relVelDiff);
  }
  
  private static Vector headToTailDiff(
    Vector thisState,
    boolean thisStateIsBackward,
    Geometry thisStartEdgeGeom,
    Vector otherState,
    double otherStateDistToStart,
    Geometry otherLastEdgeGeom) {
    
    final double thisStateSign = thisStateIsBackward ? -1d : 1d;
    
    /*
     * The following distance is otherState's
     * distance along the path but flipped to correspond to
     * movement opposite of thisState, and the origin
     * is set to the start of thisState's path. 
     */
    double thatFlipDist = 
        Math.abs(otherState.getElement(0))
        - Math.abs(otherStateDistToStart);
    
    final double thisDist = Math.abs(
        thisState.getElement(0));
    final double lengthDiff = 
        thisStateSign * (thisDist - thatFlipDist);
    
    double thisVel = thisState.getElement(1);
    
    double thatVel = (otherLastEdgeGeom.equalsExact(
        thisStartEdgeGeom) ? 1d : -1d) *  
        otherState.getElement(1);
    
    final double velocityDiff = thisVel - thatVel;
    
    return VectorFactory.getDefault().createVector2D(
        lengthDiff, velocityDiff);
  }

  /**
   * This method returns the start component index 
   * for a LinearLocation on a multi-component/geometry 
   * geom when the location is on the end of the component before.
   * @param loc
   * @param geom
   * @return
   */
  static public int geomIndexOf(LinearLocation loc, Geometry geom) {
    final Geometry firstComponent = geom.getGeometryN(
        loc.getComponentIndex());
    final int adjIndex;
    if (firstComponent.getNumPoints() - 1 == loc.getSegmentIndex()
        && 
        loc.getComponentIndex() + 1 < geom.getNumGeometries()) {
      adjIndex = loc.getComponentIndex() + 1;
    } else {
      adjIndex = loc.getComponentIndex();
    }
    
    return adjIndex;
  }

  @Override
  public AbstractPathState clone() {
    AbstractPathState clone = (AbstractPathState) super.clone();
    clone.edge = this.edge;
    clone.path = this.path;
    return clone;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + ((path == null) ? 0 : path.hashCode());
    result = prime * result + ((getRawState() == null) ? 0 : 
      ((AbstractVector)getRawState()).hashCode());
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
    AbstractPathState other = (AbstractPathState) obj;
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
    } else if (!((AbstractVector)getRawState()).equals(
        ((AbstractVector)other.getRawState()))) {
      return false;
    }
    return true;
  }

  @Override
  public String toString() {
    StringBuilder builder = new StringBuilder();
    builder.append("AbstractPathState [path=").append(path)
    .append(", state=").append(this.getGlobalState())
        .append("]");
    return builder.toString();
  }

  abstract public Vector getRawState();

}