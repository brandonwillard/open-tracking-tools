package org.opentrackingtools.graph.paths;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Map;

import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.paths.edges.PathEdge;
import org.opentrackingtools.graph.paths.edges.impl.EdgePredictiveResults;
import org.opentrackingtools.graph.paths.impl.InferredPathPrediction;
import org.opentrackingtools.graph.paths.states.PathState;
import org.opentrackingtools.graph.paths.states.PathStateBelief;
import org.opentrackingtools.graph.paths.states.impl.SimplePathStateBelief;
import org.opentrackingtools.impl.VehicleState;
import org.opentrackingtools.statistics.filters.vehicles.road.impl.AbstractRoadTrackingFilter;

import com.google.common.collect.ImmutableList;
import com.vividsolutions.jts.geom.Geometry;

public interface InferredPath extends Comparable<InferredPath> {

  public abstract double clampToPath(double distance);

  /**
   * Does the same as {@link #isOnPath(double)} except it returns the trimmed
   * location if it falls within allowable error, and works with/returns a state
   * vector.
   * 
   * @param distance
   * @return
   */
  public abstract PathState getStateOnPath(Vector state,
    double tolerance);

  /**
   * Returns the farthest PathEdge that the given distance could correspond to.
   * The clamp option will clamp the distance to the beginning or end of the
   * path.
   * 
   * @param distance
   * @param clamp
   * @return
   */
  public abstract PathEdge getEdgeForDistance(double distance,
    boolean clamp);

  public abstract ImmutableList<? extends PathEdge> getEdges();

  public abstract Geometry getGeometry();

  public abstract Boolean getIsBackward();

  /**
   * XXX: the state must have a prior predictive mean.
   * 
   * @param obs
   * @param state
   * @param edgeToPreBeliefAndLogLik
   * @return
   */
  public abstract
      InferredPathPrediction
      getPriorPredictionResults(
        GpsObservation obs,
        VehicleState state,
        Map<PathEdge, EdgePredictiveResults> edgeToPreBeliefAndLogLik);

  /**
   * TODO FIXME: make this a generic getStateOnPath (replace the other one).
   * 
   * @see {@link SimpleInferredPath#getStateOnPath(PathState)}
   * @param stateBelief
   * @return
   */
  public abstract PathStateBelief getStateBeliefOnPath(
    PathStateBelief stateBelief);

  /**
   * Converts location component of the mean to a location on this path, if any. <br>
   * Basically, if the state isn't already defined on the path, then we check if
   * the state's edge is the opposite direction to the first edge on this path.
   * If so, we can convert the state to this path's direction.
   * 
   * @param beliefPrediction
   */
  public abstract PathState 
      getStateOnPath(PathState currentState);

  public abstract Double getTotalPathDistance();

  public abstract boolean isNullPath();

  /**
   * Checks if the distance is on the path given some allowable error defined by
   * {@link AbstractRoadTrackingFilter#getEdgeLengthErrorTolerance()}.
   * 
   * @param distance
   * @return
   */
  public abstract boolean isOnPath(double distance);

  public abstract void setIsBackward(Boolean isBackward);

  public abstract void updateEdges(GpsObservation obs,
    MultivariateGaussian globalStateBelief, InferenceGraph inferredGraph);

  public abstract PathStateBelief getStateBeliefOnPath(
    MultivariateGaussian rawStateBelief);

  public abstract PathState getStateOnPath(Vector result);

  public abstract InferredPath getPathTo(PathEdge edge);

}