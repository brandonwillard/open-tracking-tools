package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.Vector;

import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;

import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Ordering;
import com.vividsolutions.jts.geom.Geometry;

public class PathEdge implements Comparable<PathEdge> {

  private final InferredEdge edge;
  private final Double distToStartOfEdge;
  private final Boolean isBackward;

  private static PathEdge emptyPathEdge = new PathEdge(
      InferredEdge.getEmptyEdge());

  private PathEdge(InferredEdge edge) {
    this.edge = edge;
    this.distToStartOfEdge = null;
    this.isBackward = null;
  }

  private PathEdge(InferredEdge edge, double distToStartOfEdge,
    Boolean isBackward) {
    Preconditions.checkArgument(!edge.isEmptyEdge());
    Preconditions.checkState(!isBackward || distToStartOfEdge <= 0d);
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
    this.isBackward = isBackward;
  }

  @Override
  public int compareTo(PathEdge o) {
    return ComparisonChain
        .start()
        .compare(this.edge, o.edge)
        .compare(this.distToStartOfEdge, o.distToStartOfEdge,
            Ordering.natural().nullsLast()).result();
  }

  public Geometry getGeometry() {
    return this.edge.getGeometry();
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
    final PathEdge other = (PathEdge) obj;
    if (distToStartOfEdge == null) {
      if (other.distToStartOfEdge != null) {
        return false;
      }
    } else if (!distToStartOfEdge.equals(other.distToStartOfEdge)) {
      return false;
    }
    if (edge == null) {
      if (other.edge != null) {
        return false;
      }
    } else if (!edge.equals(other.edge)) {
      return false;
    }
    if (isBackward == null) {
      if (other.isBackward != null) {
        return false;
      }
    } else if (!isBackward.equals(other.isBackward)) {
      return false;
    }
    return true;
  }

  /**
   * Returns a state on the edge that's been truncated within the given tolerance.
   * The relative parameters set to true will return a state relative to the edge (i.e.
   * removing the distance to start).
   * @param state
   * @param tolerance
   * @param relative 
   * @return the state on the edge or null if it's 
   */
  public Vector getCheckedStateOnEdge(Vector state, double tolerance, boolean relative) {
    Preconditions.checkState(!isEmptyEdge());
    Preconditions.checkArgument(tolerance >= 0d);
    Preconditions.checkArgument(state.getDimensionality() == 2);

    final Vector newState = state.clone();
    final double distance = newState.getElement(0);
    final double direction = isBackward ? -1d : 1d;
    final double posDistAdj =
        direction * distance - Math.abs(distToStartOfEdge);
    final double overEndDist = posDistAdj - this.edge.getLength();
    if (overEndDist > 0d) {
      if (overEndDist > tolerance) {
        return null;
      } else {
        newState.setElement(0, direction * this.edge.getLength()
            + (relative ? 0d : distToStartOfEdge));
      }
    } else if (posDistAdj < 0d) {
      if (posDistAdj < -tolerance) {
        return null;
      } else {
        newState.setElement(0, (relative ? 0d : distToStartOfEdge));
      }
    }

    if (relative)
      newState.setElement(0, direction * posDistAdj);
    
    return newState;
  }

  public Double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  public double getLength() {
    return this.getGeometry().getLength();
  }
  
  public InferredEdge getInferredEdge() {
    return edge;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((distToStartOfEdge == null) ? 0 : distToStartOfEdge
                .hashCode());
    result = prime * result + ((edge == null) ? 0 : edge.hashCode());
    result =
        prime * result
            + ((isBackward == null) ? 0 : isBackward.hashCode());
    return result;
  }

  public Boolean isBackward() {
    return isBackward;
  }

  public boolean isEmptyEdge() {
    return this == emptyPathEdge;
  }

  /**
   * Based on the path that this edge is contained in, 
   * determine if the given distance is on this edge.
   * XXX: It is possible that a given distance is
   * on more than one edge (depends on the value of 
   * {@link AbstractRoadTrackingFilter#getEdgeLengthErrorTolerance()}).
   * 
   * @param distance
   * @return
   */
  public boolean isOnEdge(double distance) {
    final double direction = this.isBackward ? -1d : 1d;
    final double posDistToStart = Math.abs(distToStartOfEdge);
    final double posDistOffset =
        direction * distance - posDistToStart;

    if (posDistOffset - edge.getLength() > 1e-7d) {
      return false;
    } else if (posDistOffset < 0d) {
      return false;
    }

    return true;
  }

  @Override
  public String toString() {
    if (this == emptyPathEdge) {
      return "PathEdge [empty edge]";
    } else {
      final double distToStart =
          distToStartOfEdge == 0d && this.isBackward ? -0d
              : distToStartOfEdge.longValue();
      return "PathEdge [edge=" + edge.getEdgeId() + " ("
          + edge.getLength().longValue() + ")" + ", distToStart="
          + distToStart + "]";
    }
  }

  public static PathEdge getEdge(InferredEdge infEdge,
    double distToStart, Boolean isBackward) {
    Preconditions.checkArgument(isBackward != Boolean.TRUE
        || distToStart <= 0d);

    PathEdge edge;
    if (infEdge.isEmptyEdge() || isBackward == null) {
      edge = PathEdge.getEmptyPathEdge();
    } else {
      edge = new PathEdge(infEdge, distToStart, isBackward);
    }
    return edge;
  }

  public static PathEdge getEmptyPathEdge() {
    return emptyPathEdge;
  }
}
