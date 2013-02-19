package org.opentrackingtools.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.opentrackingtools.GpsObservation;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.statistics.distributions.PathStateDistribution;
import org.opentrackingtools.statistics.distributions.impl.OnOffEdgeTransDirMulti;
import org.opentrackingtools.statistics.estimators.vehicles.impl.AbstractRoadTrackingEstimator;

import com.google.common.base.Preconditions;
import com.google.common.collect.Iterables;

/**
 * This class represents the state of a vehicle, which is made up of the
 * vehicles location, whether it is on an edge, which path it took from its
 * previous location on an edge, and the distributions that determine these.
 * 
 * @author bwillard
 * 
 */
public class VehicleState implements
    ComputableDistribution<GpsObservation>,
    Comparable<VehicleState> {

  public static class PDF extends VehicleState implements
      ProbabilityFunction<GpsObservation> {

    private static final long serialVersionUID =
        879217079360170446L;

    public PDF(VehicleState state) {
      super(state);
    }

    @Override
    public PDF clone() {
      return (PDF) super.clone();
    }

    @Override
    public Double evaluate(GpsObservation input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public VehicleState.PDF getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(GpsObservation input) {
      double logLikelihood = 0d;

      /*
       * Movement.
       */
      logLikelihood +=
          this.belief.logLikelihood(
              input.getProjectedPoint(),
              this.getMovementFilter());
      //          this.getMovementFilter().logLikelihood(
      //              input.getProjectedPoint(), this.belief);

      return logLikelihood;
    }

    @Override
    public GpsObservation sample(Random random) {
      throw new RuntimeException("Not implemented");
    }

    @Override
    public ArrayList<GpsObservation> sample(Random random,
      int numSamples) {
      throw new RuntimeException("Not implemented");
    }

  }

  private static final long serialVersionUID =
      3229140254421801273L;

  /*
   * These members represent the state/parameter samples/sufficient statistics.
   */
  private final AbstractRoadTrackingEstimator movementFilter;

  /**
   * This could be the 4D ground-coordinates dist. for free motion, or the 2D
   * road-coordinates, either way the tracking filter will check. Also, this
   * could be the prior or prior predictive distribution.
   */
  protected final PathStateDistribution belief;

  /*-
   * Edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected final OnOffEdgeTransDirMulti edgeTransitionDist;
  private final GpsObservation observation;
  private VehicleState parentState = null;

  private final InferenceGraph graph;

  // private final int initialHashCode;
  // private final int edgeInitialHashCode;
  // private final int obsInitialHashCode;
  // private final int transInitialHashCode;
  // private final int beliefInitialHashCode;

  private int hash = 0;

  public VehicleState(InferenceGraph inferredGraph,
    GpsObservation observation,
    AbstractRoadTrackingEstimator updatedFilter,
    PathStateDistribution belief,
    OnOffEdgeTransDirMulti edgeTransitionDist,
    VehicleState parentState) {

    Preconditions.checkNotNull(inferredGraph);
    Preconditions.checkNotNull(observation);
    Preconditions.checkNotNull(updatedFilter);
    Preconditions.checkNotNull(belief);

    this.observation = observation;
    this.movementFilter = updatedFilter;
    this.belief = belief.clone();
    this.graph = inferredGraph;

    /*
     * Check that the state's location corresponds
     * to the last edge.
     */
    Preconditions
        .checkState(!belief.isOnRoad()
            || belief.getEdge().equals(
                Iterables.getLast(belief.getPath()
                    .getPathEdges())));

    this.edgeTransitionDist = edgeTransitionDist;

    this.parentState = parentState;
    /*
     * Reset the parent's parent state so that we don't keep these objects
     * forever.
     */

    final double timeDiff;
    if (observation.getPreviousObservation() != null) {
      timeDiff =
          (observation.getTimestamp().getTime() - observation
              .getPreviousObservation().getTimestamp()
              .getTime()) / 1000d;
      this.movementFilter.setCurrentTimeDiff(timeDiff);
    } 

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  public VehicleState(VehicleState other) {
    this.graph = other.graph;
    this.movementFilter = other.movementFilter.clone();
    this.belief = other.belief.clone();
    this.edgeTransitionDist =
        other.edgeTransitionDist.clone();
    this.observation = other.observation;
    this.parentState = other.parentState;

    // DEBUG
    // this.initialHashCode = this.hashCode();
    // this.edgeInitialHashCode = this.edge.hashCode();
    // this.transInitialHashCode = this.edgeTransitionDist.hashCode();
    // this.beliefInitialHashCode =
    // Arrays.hashCode(((DenseVector)this.initialBelief.convertToVector()).getArray());
    // this.obsInitialHashCode = this.observation.hashCode();
  }

  @Override
  public VehicleState clone() {
    return new VehicleState(this);
  }

  @Override
  public int compareTo(VehicleState arg0) {
    return oneStateCompareTo(this, arg0);
  }

  @Override
  public boolean equals(Object obj) {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (!oneStateEquals(this, obj))
      return false;

    final VehicleState other = (VehicleState) obj;
    if (parentState == null) {
      if (other.parentState != null) {
        return false;
      }
    } else if (!oneStateEquals(parentState,
        other.parentState)) {
      return false;
    }

    return true;
  }

  public PathStateDistribution getBelief() {
    return belief;
  }

  public OnOffEdgeTransDirMulti getEdgeTransitionDist() {
    return edgeTransitionDist;
  }

  public InferenceGraph getGraph() {
    return graph;
  }

  /**
   * Returns ground-coordinate mean location
   * 
   * @return
   */
  public Vector getMeanLocation() {
    final Vector v = belief.getGroundState();
    return AbstractRoadTrackingEstimator.getOg().times(v);
  }

  public AbstractRoadTrackingEstimator getMovementFilter() {
    return movementFilter;
  }

  public GpsObservation getObservation() {
    return observation;
  }

  public VehicleState getParentState() {
    return parentState;
  }

  @Override
  public VehicleState.PDF getProbabilityFunction() {
    return new VehicleState.PDF(this);
  }

  @Override
  public int hashCode() {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (hash != 0) {
      return hash;
    } else {
      final int prime = 31;
      int result = 1;
      result = prime * result + oneStateHashCode(this);
      if (this.parentState != null)
        result =
            prime * result
                + oneStateHashCode(this.parentState);
      hash = result;
      return result;
    }
  }

  @Override
  public GpsObservation sample(Random random) {
    throw new RuntimeException("Not implemented");
  }

  @Override
  public ArrayList<GpsObservation> sample(Random random,
    int numSamples) {
    throw new RuntimeException("Not implemented");
  }

  public void setParentState(VehicleState parentState) {
    this.parentState = parentState;
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("belief", belief);
    builder.append("observation", observation);
    return builder.toString();
  }

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4)
      res =
          AbstractRoadTrackingEstimator.getOg().times(vector);
    else
      res =
          AbstractRoadTrackingEstimator.getOr().times(vector);
    return res;
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  private static int oneStateCompareTo(VehicleState t,
    VehicleState o) {
    if (t == o)
      return 0;

    if (t == null) {
      if (o != null)
        return -1;
      else
        return 0;
    } else if (o == null) {
      return 1;
    }

    final CompareToBuilder comparator =
        new CompareToBuilder();
    comparator.append(t.belief, o.belief);
    comparator.append(t.getObservation(),
        o.getObservation());
    comparator.append(t.edgeTransitionDist,
        o.edgeTransitionDist);

    return comparator.toComparison();
  }

  protected static boolean oneStateEquals(Object thisObj,
    Object obj) {
    if (thisObj == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (thisObj.getClass() != obj.getClass()) {
      return false;
    }
    final VehicleState thisState = (VehicleState) thisObj;
    final VehicleState other = (VehicleState) obj;
    if (thisState.belief == null) {
      if (other.belief != null) {
        return false;
      }
    } else if (!thisState.belief.equals(other.belief)) {
      return false;
    }
    if (thisState.edgeTransitionDist == null) {
      if (other.edgeTransitionDist != null) {
        return false;
      }
    } else if (!thisState.edgeTransitionDist
        .equals(other.edgeTransitionDist)) {
      return false;
    }
    if (thisState.movementFilter == null) {
      if (other.movementFilter != null) {
        return false;
      }
    } else if (!thisState.movementFilter
        .equals(other.movementFilter)) {
      return false;
    }
    if (thisState.observation == null) {
      if (other.observation != null) {
        return false;
      }
    } else if (!thisState.observation
        .equals(other.observation)) {
      return false;
    }
    return true;
  }
  
  protected static int oneStateHashCode(VehicleState state) {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((state.belief == null) ? 0 : state.belief
                .hashCode());
    result =
        prime
            * result
            + ((state.edgeTransitionDist == null) ? 0
                : state.edgeTransitionDist.hashCode());
    result =
        prime
            * result
            + ((state.movementFilter == null) ? 0
                : state.movementFilter.hashCode());
    result =
        prime
            * result
            + ((state.observation == null) ? 0
                : state.observation.hashCode());
    return result;
  }

}
