package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.matrix.Vector;

import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.filters.road_tracking.AbstractRoadTrackingFilter;

import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Ordering;

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

  public Vector getCheckedStateOnEdge(Vector state, double tolerance) {
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
            + distToStartOfEdge);
      }
    } else if (posDistAdj < 0d) {
      if (posDistAdj < tolerance) {
        return null;
      } else {
        newState.setElement(0, distToStartOfEdge);
      }
    }

    return newState;
  }

  public Double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  public InferredEdge getEdge() {
    return edge;
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
   * Based on the path that this edge is contained in, determine if the given
   * distance is on this edge.
   * 
   * @param distance
   * @return
   */
  public boolean isOnEdge(double distance) {
    final double direction = this.isBackward ? -1d : 1d;
    final double posDistToStart = Math.abs(distToStartOfEdge);
    final double posDistOffset =
        direction * distance - posDistToStart;

    if (posDistOffset - edge.getLength() > AbstractRoadTrackingFilter
        .getEdgelengthtolerance()) {
      return false;
    } else if (posDistOffset < -AbstractRoadTrackingFilter
        .getEdgelengthtolerance()) {
      return false;
    }

    return true;
  }

  /*
   * This method is problematic, since it discards/ignores direction.
   */
  //  public static PathEdge getEdge(InferredEdge infEdge) {
  //    PathEdge edge;
  //    if (infEdge.isEmptyEdge()) {
  //      edge = PathEdge.getEmptyPathEdge();
  //    } else {
  //      edge = new PathEdge(infEdge, 0d, false);
  //    }
  //    return edge;
  //  }

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
