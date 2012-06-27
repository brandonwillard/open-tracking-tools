package org.openplans.tools.tracking.impl.graph.paths;

import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;


import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Ordering;

public class PathEdge implements Comparable<PathEdge> {

  private final InferredEdge edge;
  private final Double distToStartOfEdge;

  private static PathEdge emptyPathEdge = new PathEdge(
      InferredEdge.getEmptyEdge());

  private PathEdge(InferredEdge edge) {
    this.edge = edge;
    this.distToStartOfEdge = null;
  }

  private PathEdge(InferredEdge edge, double distToStartOfEdge) {
    Preconditions.checkArgument(!edge.isEmptyEdge());
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
  }

  @Override
  public int compareTo(PathEdge o) {
    return ComparisonChain
        .start()
        .compare(this.edge, o.edge)
        .compare(
            this.distToStartOfEdge, o.distToStartOfEdge,
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
    return true;
  }

  public Double getDistToStartOfEdge() {
    return distToStartOfEdge;
  }

  public InferredEdge getInferredEdge() {
    return edge;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime
        * result
        + ((distToStartOfEdge == null) ? 0 : distToStartOfEdge
            .hashCode());
    result = prime * result + ((edge == null) ? 0 : edge.hashCode());
    return result;
  }

  public boolean isEmptyEdge() {
    return this == emptyPathEdge;
  }



  
  /**
   * Based on the path that this edge is contained in, determine if the
   * given distance is on this edge.
   * @param distance
   * @return
   */
  public boolean isOnEdge(double distance) {
    if (distToStartOfEdge < 0d) {
      if (distance > -edge.getLength() + distToStartOfEdge 
          && distance <= distToStartOfEdge)
        return true;
    } else if (distToStartOfEdge > 0d){
      if (distance < edge.getLength() + distToStartOfEdge 
          && distance >= distToStartOfEdge)
        return true;
    } else {
      if (Math.abs(distance) < edge.getLength() + Math.abs(distToStartOfEdge)
          && Math.abs(distance) >= Math.abs(distToStartOfEdge))
        return true;
    }
    return false;
  }

  @Override
  public String toString() {
    if (this == emptyPathEdge)
      return "PathEdge [empty edge]";
    else
      return "PathEdge [edge=" + edge.getEdgeId()
          + ", distToStartOfEdge=" + distToStartOfEdge + "]";
  }

  public static PathEdge getEdge(InferredEdge infEdge) {
    PathEdge edge;
    if (infEdge.isEmptyEdge()) {
      edge = PathEdge.getEmptyPathEdge();
    } else {
      edge = new PathEdge(infEdge, 0d);
    }
    return edge;
  }

  public static PathEdge getEdge(InferredEdge infEdge,
    double distToStart) {
    PathEdge edge;
    if (infEdge.isEmptyEdge()) {
      edge = PathEdge.getEmptyPathEdge();
    } else {
      edge = new PathEdge(infEdge, distToStart);
    }
    return edge;
  }

  public static PathEdge getEmptyPathEdge() {
    return emptyPathEdge;
  }
}
