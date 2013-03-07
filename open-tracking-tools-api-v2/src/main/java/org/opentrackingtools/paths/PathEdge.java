package org.opentrackingtools.paths;

import java.util.ArrayList;
import java.util.Random;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import org.opentrackingtools.distributions.BayesianEstimableDistribution;
import org.opentrackingtools.distributions.PathEdgeDistribution;
import org.opentrackingtools.distributions.PathEdgeProbabilityFunction;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.RecursiveBayesianEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.util.StatisticsUtil;

import com.google.common.base.Preconditions;
import com.google.common.collect.ComparisonChain;
import com.google.common.collect.Ordering;
import com.vividsolutions.jts.geom.Geometry;

public class PathEdge<E extends InferenceGraphEdge> extends AbstractCloneableSerializable implements
    Comparable<PathEdge<E>> {

  private static final long serialVersionUID = 2615199504616160384L;

  protected E edge;
  protected Double distToStartOfEdge;
  protected Boolean isBackward;

  protected PathEdge() {
    this.edge = null;
    this.distToStartOfEdge = null;
    this.isBackward = null;
  }

  protected PathEdge(E edge,
    Double distToStartOfEdge, Boolean isBackward) {
    Preconditions.checkState((isBackward != Boolean.TRUE)
        || distToStartOfEdge <= 0d);
    this.edge = edge;
    this.distToStartOfEdge = distToStartOfEdge;
    this.isBackward = isBackward;
  }

  @Override
  public int compareTo(PathEdge<E> o) {
    return ComparisonChain
        .start()
        .compare(this.edge, o.getInferredEdge())
        .compare(this.isBackward(), o.isBackward(),
            Ordering.natural().nullsLast())
        .compare(this.distToStartOfEdge, o.getDistToStartOfEdge(),
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
    final PathEdge<E> other = (PathEdge<E>) obj;
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
   * Returns a state on the edge that's been truncated within the given
   * tolerance. The relative parameters set to true will return a state relative
   * to the edge (i.e. removing the distance to start).
   * 
   * @param state
   * @param tolerance
   * @param relative
   * @return the state on the edge or null if it's
   */
  public Vector getCheckedStateOnEdge(Vector state, double tolerance,
    boolean relative) {
    Preconditions.checkState(!isNullEdge());
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

  public Geometry getGeometry() {
    return this.edge.getGeometry();
  }

  public E getInferredEdge() {
    return edge;
  }

  public double getLength() {
    return this.getGeometry().getLength();
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

  public boolean isNullEdge() {
    return this.getInferredEdge() == null;
  }

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
    if (this.isNullEdge()) {
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

  @Override
  public PathEdge<E> clone() {
    PathEdge<E> clone = (PathEdge<E>) super.clone();
    clone.distToStartOfEdge = distToStartOfEdge;
    clone.edge = edge;
    clone.isBackward = isBackward;
    return clone;
  }

}
