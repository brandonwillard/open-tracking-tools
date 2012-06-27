package org.openplans.tools.tracking.impl.graph.paths;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.UnivariateGaussian;
import gov.sandia.cognition.util.DefaultWeightedValue;

import java.util.List;
import java.util.Map;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.geotools.geometry.jts.FactoryFinder;
import org.geotools.geometry.jts.JTSFactoryFinder;
import org.openplans.tools.tracking.impl.Observation;
import org.openplans.tools.tracking.impl.VehicleState;
import org.openplans.tools.tracking.impl.graph.InferredEdge;
import org.openplans.tools.tracking.impl.statistics.StandardRoadTrackingFilter;
import org.openplans.tools.tracking.impl.statistics.WrappedWeightedValue;

import com.google.common.base.Preconditions;
import com.google.common.collect.ImmutableCollection;
import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;
import com.google.common.collect.Maps;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryCollection;
import com.vividsolutions.jts.geom.GeometryFactory;

/**
 * Inferred paths are collections of PathEdges that track the distance traveled
 * and the direction (by sign)
 * 
 * @author bwillard
 * 
 */
public class InferredPath implements Comparable<InferredPath> {

  private final ImmutableList<PathEdge> edges;
  private final Double totalPathDistance;
  public List<Integer> edgeIds = Lists.newArrayList();
  
  /*
   * These are the edges used in path finding.
   */
  private InferredEdge startEdge;
  private InferredEdge endEdge;
  
  /*
   * Note: single edges are considered forward
   */
  private Boolean isBackward = null;
  private final Geometry geometry;

  private static InferredPath emptyPath = new InferredPath();

  private InferredPath() {
    this.edges = ImmutableList.of(PathEdge.getEmptyPathEdge());
    this.totalPathDistance = null;
    this.isBackward = null;
    this.geometry = null;
  }

  private InferredPath(ImmutableList<PathEdge> edges, boolean isBackward) {
    Preconditions.checkArgument(edges.size() > 0);
    this.edges = edges;
    this.isBackward = isBackward;

    PathEdge lastEdge = null;
    double absTotalDistance = 0d;
    List<Geometry> geometries = Lists.newArrayList();
    for (final PathEdge edge : edges) {
      
      if (!edge.isEmptyEdge()) {
        if (isBackward) {
          assert (lastEdge == null || lastEdge.getInferredEdge().getStartVertex()
            .equals(edge.getInferredEdge().getEndVertex()));
          
          geometries.add(edge.getInferredEdge().getGeometry().reverse());
          
        } else {
          assert (lastEdge == null || lastEdge.getInferredEdge().getEndVertex()
            .equals(edge.getInferredEdge().getStartVertex()));
          
          geometries.add(edge.getInferredEdge().getGeometry());
        }
        absTotalDistance += edge.getInferredEdge().getLength();
        edgeIds.add(edge.getInferredEdge().getEdgeId());
      }
      
      lastEdge = edge;
    }
    
    if (geometries.size() > 1) {
      // TODO simplify to LineString?
      GeometryFactory factory = JTSFactoryFinder.getGeometryFactory( null );
      GeometryCollection geometryCollection =
           (GeometryCollection) factory.buildGeometry( geometries);
      geometryCollection.union();
      this.geometry = geometryCollection; 
    } else {
      this.geometry = geometries.get(0); 
    }
    
    final double direction = isBackward ? -1d : 1d;
    this.totalPathDistance = direction * absTotalDistance;
  }

  private InferredPath(InferredEdge inferredEdge) {
    Preconditions.checkArgument(!inferredEdge.isEmptyEdge());
    this.edges = ImmutableList.of(PathEdge.getEdge(inferredEdge, 0d));
    this.totalPathDistance = inferredEdge.getLength();
    this.isBackward = Boolean.FALSE;
    this.edgeIds.add(inferredEdge.getEdgeId());
    this.geometry = inferredEdge.getGeometry();
  }

  private InferredPath(PathEdge edge) {
    Preconditions.checkArgument(!edge.isEmptyEdge());
    Preconditions.checkArgument(edge.getDistToStartOfEdge() == 0d);
    this.edges = ImmutableList.of(edge);
    this.totalPathDistance = edge.getInferredEdge().getLength();
    this.isBackward = Boolean.FALSE;
    this.edgeIds.add(edge.getInferredEdge().getEdgeId());
    this.geometry = edge.getInferredEdge().getGeometry();
  }

  @Override
  public int compareTo(InferredPath o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edges.toArray(), o.edges.toArray());
    return comparator.toComparison();
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
    final InferredPath other = (InferredPath) obj;
    if (edges == null) {
      if (other.edges != null) {
        return false;
      }
    } else if (!edges.equals(other.edges)) {
      return false;
    }
    return true;
  }

  public boolean isEmptyPath() {
    return this == emptyPath;
  }
  
  /**
   * Returns the edge that covers the given distance, or null.
   * 
   * @param distance
   * @return
   */
//  public PathEdge getEdgeAtDistance(double distance) {
//    Preconditions.checkArgument(this != emptyPath);
//    Preconditions.checkArgument(distance >= 0d);
//    // TODO pre-compute/improve this
//    for (final PathEdge edge : this.edges) {
//      if (edge.getDistToStartOfEdge() <= distance
//          && distance < edge.getDistToStartOfEdge()
//              // FIXME if you're going to use this, set the direction! 
//              + edge.getInferredEdge().getLength()) {
//        return edge;
//      }
//    }
//
//    return null;
//  }

  public ImmutableList<PathEdge> getEdges() {
    return edges;
  }

  /**
   * XXX: the state must have a prior predictive mean.
   * 
   * @param obs
   * @param state
   * @param edgeToPreBeliefAndLogLik 
   * @return
   */
  public InferredPathEntry getPredictiveLogLikelihood(
    Observation obs, VehicleState state, 
    Map<PathEdge, WrappedWeightedValue<MultivariateGaussian>> edgeToPreBeliefAndLogLik) {

    /*-
     * A prior predictive is created for every path, since, in some instances,
     * we need to project onto an edge and then predict movement.
     */
    final MultivariateGaussian beliefPrediction = state.getBelief()
        .clone();
    final StandardRoadTrackingFilter filter = state
        .getMovementFilter();
    PathEdge prevEdge = PathEdge.getEdge(state.getInferredEdge());
    
    /*
     * Check that we're starting from the edge we're on.
     */
    PathEdge initialEdge = this.edges.get(0);
    if (!initialEdge.isEmptyEdge() && !state.getInferredEdge().isEmptyEdge()) {
      /*
       * Make sure that everything is measured in the same direction of the path,
       * and starting from the beginning of the initial edge.
       */
      StandardRoadTrackingFilter.normalizeBelief(beliefPrediction.getMean(), initialEdge);
      
//      assert initialEdge.getInferredEdge().equals(state.getInferredEdge());
    }

    
    filter.predict(beliefPrediction, this.getEdges().get(0), 
        PathEdge.getEdge(state.getInferredEdge()));
    
    double pathLogLik = Double.NEGATIVE_INFINITY;
    
    List<WrappedWeightedValue<PathEdge>> weightedPathEdges = Lists.newArrayList();
    for (final PathEdge edge : this.getEdges()) {

      final double localLogLik;
      if (edgeToPreBeliefAndLogLik.containsKey(edge)) {
        localLogLik = edgeToPreBeliefAndLogLik.get(edge).getWeight();
      } else {
        /*
         * If we're going off-road, then pass the edge we used to be on.
         */
        final MultivariateGaussian locationPrediction = beliefPrediction
            .clone();
        final double edgePredMarginalLogLik;
        if (edge.isEmptyEdge()) {
          // TODO meh?
          edgePredMarginalLogLik = 0d;
        } else {
          predict(locationPrediction, obs, edge);
          edgePredMarginalLogLik = marginalPredictiveLogLikelihood(edge, beliefPrediction);
        }
  
        final double edgePredTransLogLik = state
            .getEdgeTransitionDist().predictiveLogLikelihood(
                prevEdge.getInferredEdge(), edge.getInferredEdge());
  
        final double localPosVelPredLogLik = filter.logLikelihood(
            obs.getProjectedPoint(), locationPrediction, edge);
        
        assert !Double.isNaN(edgePredMarginalLogLik);
        assert !Double.isNaN(edgePredTransLogLik);
        assert !Double.isNaN(localPosVelPredLogLik);
  
        localLogLik = edgePredMarginalLogLik
            + edgePredTransLogLik + localPosVelPredLogLik;
  
        Preconditions.checkArgument(!Double.isNaN(localLogLik));
  
        /*
         * We're only going to deal with the terminating edge for now.
         */
        edgeToPreBeliefAndLogLik.put(
            edge, new WrappedWeightedValue<MultivariateGaussian>(
                locationPrediction.clone(), localLogLik));
      }

      /*
       * Add likelihood for this edge to the path total
       */
      weightedPathEdges.add(new WrappedWeightedValue<PathEdge>(edge, localLogLik));
      pathLogLik = LogMath.add(pathLogLik, localLogLik);
      prevEdge = edge;
    }

    return new InferredPathEntry(
        this, beliefPrediction, edgeToPreBeliefAndLogLik, filter,
        weightedPathEdges, pathLogLik);
  }
  /**
   * This method truncates the given belief over the interval defined by this
   * edge.
   * 
   * @param belief
   * @param edge2 
   */
  public void predict(MultivariateGaussian belief, Observation obs, PathEdge edge) {

    Preconditions.checkArgument(belief.getInputDimensionality() == 2);
    Preconditions.checkArgument(edges.contains(edge));
    
    /*-
     * TODO really, this should just be the truncated/conditional
     * mean and covariance for the given interval/edge
     */
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double S = Or.times(belief.getCovariance())
        .times(Or.transpose()).getElement(0, 0)
        // + 1d;
        + Math.pow(edge.getInferredEdge().getLength() / Math.sqrt(12), 2);
    final Matrix W = belief.getCovariance().times(Or.transpose())
        .scale(1 / S);
    final Matrix R = belief.getCovariance().minus(
        W.times(W.transpose()).scale(S));
    
    final double mean;
    if (edge.getDistToStartOfEdge() == 0d) {
      /*
       * When this edge is the entire path, choose the midpoint to be
       * the same direction as the movement.
       */
      final double direction = belief.getMean().getElement(0) >= 0d ? 1d : -1d;
      mean = direction * edge.getInferredEdge().getLength() / 2d;
    } else {
      final double direction = edge.getDistToStartOfEdge() > 0d ? 1d : -1d;
      mean = (edge.getDistToStartOfEdge() + (edge.getDistToStartOfEdge() + direction
          * edge.getInferredEdge().getLength())) / 2d;    
      
    }

    /*
     * The mean can be the center-length of the geometry, or something more
     * specific, like the snapped location?
     */
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
    
//    final double direction = distToStartOfEdge > 0d ? 1d : -1d;
//    final double distToEndOfEdge = direction * edge.getLength() + this.distToStartOfEdge;
//    final double startDistance = direction > 0d ? this.distToStartOfEdge : distToEndOfEdge; 
//    final double endDistance = direction > 0d ? distToEndOfEdge : this.distToStartOfEdge;   
//    final double stdDev = Math.sqrt(Or.times(R).times(Or.transpose()).getElement(0, 0));
//
//    final double tmean = a.getElement(0) 
//     + stdDev * (UnivariateGaussian.PDF.evaluate(startDistance, a.getElement(0), stdDev)
//       - UnivariateGaussian.PDF.evaluate(endDistance, a.getElement(0), stdDev))
//       /(UnivariateGaussian.CDF.evaluate(endDistance, a.getElement(0), stdDev)
//           - UnivariateGaussian.CDF.evaluate(startDistance, a.getElement(0), stdDev));

    belief.setMean(a);
    belief.setCovariance(R);
  }
  
  public double marginalPredictiveLogLikelihood(PathEdge edge,
    MultivariateGaussian beliefPrediction) {
    Preconditions.checkArgument(beliefPrediction
        .getInputDimensionality() == 2);
    final Matrix Or = StandardRoadTrackingFilter.getOr();
    final double variance = Or
        .times(beliefPrediction.getCovariance())
        .times(Or.transpose()).getElement(0, 0);
    final double mean = Or.times(beliefPrediction.getMean())
        .getElement(0);
    final double direction = Math.signum(totalPathDistance);
    final double distToEndOfEdge = direction * edge.getInferredEdge().getLength() + edge.getDistToStartOfEdge();
    final double startDistance = direction > 0d ? edge.getDistToStartOfEdge() : distToEndOfEdge; 
    final double endDistance = direction > 0d ? distToEndOfEdge : edge.getDistToStartOfEdge();
    
    // FIXME use actual log calculations
    final double result = Math.log(UnivariateGaussian.CDF.evaluate(
        endDistance, mean, variance)
        - UnivariateGaussian.CDF.evaluate(
            startDistance, mean, variance));
    
    return result;
  }
  
  public Double getTotalPathDistance() {
    return totalPathDistance;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result
        + ((edges == null) ? 0 : edges.hashCode());
    return result;
  }

  @Override
  public String toString() {
    if (this == emptyPath)
      return "InferredPath [empty path]";
    else
      return "InferredPath [edges=" + edgeIds
          + ", totalPathDistance=" + totalPathDistance + "]";
  }

  public static InferredPath getEmptyPath() {
    return emptyPath;
  }

  public static InferredPath getInferredPath(InferredEdge inferredEdge) {
    if (inferredEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(inferredEdge);
  }

  public static InferredPath getInferredPath(List<PathEdge> edges, boolean isBackward) {
    if (edges.size() == 1) {
      PathEdge edge = Iterables.getOnlyElement(edges);
      if (edge.isEmptyEdge())
        return emptyPath;
      else
        return new InferredPath(edge);
    }
    return new InferredPath(ImmutableList.copyOf(edges), isBackward);
  }

  public static InferredPath getInferredPath(PathEdge pathEdge) {
    if (pathEdge.isEmptyEdge())
      return emptyPath;
    else
      return new InferredPath(pathEdge);
  }

  public void setStartEdge(InferredEdge startEdge) {
    this.startEdge = startEdge;
  }

  public InferredEdge getEndEdge() {
    return endEdge;
  }

  public void setEndEdge(InferredEdge endEdge) {
    this.endEdge = endEdge;
  }

  public InferredEdge getStartEdge() {
    return startEdge;
  }

  public Boolean isBackward() {
    return isBackward;
  }

  public void setIsBackward(Boolean isBackward) {
    this.isBackward = isBackward;
  }

  public PathEdge getEdgeForDistance(double distance, boolean clamp) {
    final double direction = Math.signum(totalPathDistance);
    if (direction * distance > Math.abs(totalPathDistance)) {
      return Iterables.getLast(edges);
    } else if (direction * distance < 0d) {
      return Iterables.getFirst(edges, null);
    }
      
    for (PathEdge edge : edges) {
      if (edge.isOnEdge(distance))
        return edge;
    }
    return null;
  }

  public Geometry getGeometry() {
    return this.geometry;
  }

}
