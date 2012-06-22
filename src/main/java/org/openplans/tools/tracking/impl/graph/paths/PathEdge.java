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
  
  public double marginalPredictiveLogLikelihood(
    MultivariateGaussian beliefPrediction, double direction) {
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double variance = Or
        .times(beliefPrediction.getCovariance())
        .times(Or.transpose()).getElement(0, 0);
    final double mean = Or.times(beliefPrediction.getMean())
        .getElement(0);
    final double distToEndOfEdge = direction * edge.getLength() + this.distToStartOfEdge;
    final double startDistance = direction > 0d ? this.distToStartOfEdge : distToEndOfEdge; 
    final double endDistance = direction > 0d ? distToEndOfEdge : this.distToStartOfEdge;
    // FIXME use actual log calculations
    final double result = Math.log(UnivariateGaussian.CDF.evaluate(
        endDistance, mean, variance)
        - UnivariateGaussian.CDF.evaluate(
            startDistance, mean, variance));
    
    return result;
  }

  /**
   * This method truncates the given belief over the interval defined by this
   * edge.
   * 
   * @param belief
   */
  public void predict(MultivariateGaussian belief, Observation obs) {

    StandardRoadTrackingFilter.convertToRoadBelief(belief, this);

    /*-
     * TODO really, this should just be the truncated/conditional
     * mean and covariance for the given interval/edge
     */
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double S = Or.times(belief.getCovariance())
        .times(Or.transpose()).getElement(0, 0)
        // + 1d;
        + Math.pow(edge.getLength() / Math.sqrt(12), 2);
    final Matrix W = belief.getCovariance().times(Or.transpose())
        .scale(1 / S);
    final Matrix R = belief.getCovariance().minus(
        W.times(W.transpose()).scale(S));
    /*
     * The mean can be the center-length of the geometry, or something more
     * specific, like the snapped location!
     */
    final double direction = belief.getMean().getElement(0) >= 0d ? 1d : -1d;

    final double mean = (distToStartOfEdge + (distToStartOfEdge + direction
        * edge.getLength())) / 2d;

    // final LocationIndexedLine locIdxLine = isPositive ?
    // edge.getPosLocationIndexedLine()
    // : edge.getNegLocationIndexedLine();
    // final LinearLocation loc = locIdxLine.project(
    // GeoUtils.reverseCoordinates(obs.getObsCoords()));
    // final LengthLocationMap lengthLocLine = isPositive ?
    // edge.getPosLengthLocationMap()
    // : edge.getNegLengthLocationMap();
    // final double mean = (isPositive ? 1d : -1d)
    // * GeoUtils.getAngleDegreesInMeters(lengthLocLine.getLength(loc)) +
    // this.getDistToStartOfEdge();

    final double e = mean - Or.times(belief.getMean()).getElement(0);
    final Vector a = belief.getMean().plus(W.getColumn(0).scale(e));

    belief.setMean(a);
    belief.setCovariance(R);
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
