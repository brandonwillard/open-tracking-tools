package org.opentrackingtools.model;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.MultivariateMixtureDensityModel;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransProbabilityFunction;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMultivariateMixtureDensityModel;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.model.TransitionProbMatrix;

/**
 * This class represents the state of a vehicle, which is made up of the
 * vehicles location, whether it is on an edge, which path it took from its
 * previous location on an edge, and the distributions that determine these.
 * 
 * @author bwillard
 * 
 */
public class VehicleState<Observation extends GpsObservation> extends AbstractCloneableSerializable
    implements Comparable<VehicleState<Observation>> {

  private static final long serialVersionUID = 3229140254421801273L;

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4)
      res = MotionStateEstimatorPredictor.getOg().times(vector);
    else
      res = MotionStateEstimatorPredictor.getOr().times(vector);
    return res;
  }

  public
      BayesianParameter<PathState, PathStateMultivariateMixtureDensityModel<PathStateDistribution>, PathStateDistribution>
      getPathStateParam() {
    return pathStateParam;
  }

  public static long getSerialversionuid() {
    return serialVersionUID;
  }

  private static int oneStateCompareTo(
    VehicleState<? extends GpsObservation> t,
    VehicleState<? extends GpsObservation> o) {
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

    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(t.motionStateParam, o.motionStateParam);
    comparator.append(t.getObservation(), o.getObservation());
    comparator.append(t.edgeTransitionParam, o.edgeTransitionParam);

    return comparator.toComparison();
  }

  protected static boolean oneStateEquals(Object thisObj, Object obj) {
    if (thisObj == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (thisObj.getClass() != obj.getClass()) {
      return false;
    }
    final VehicleState<? extends GpsObservation> thisState =
        (VehicleState<? extends GpsObservation>) thisObj;
    final VehicleState<? extends GpsObservation> other =
        (VehicleState<? extends GpsObservation>) obj;
    if (thisState.motionStateParam == null) {
      if (other.motionStateParam != null) {
        return false;
      }
    } else if (!thisState.motionStateParam.equals(other.motionStateParam)) {
      return false;
    }
    if (thisState.edgeTransitionParam == null) {
      if (other.edgeTransitionParam != null) {
        return false;
      }
    } else if (!thisState.edgeTransitionParam
        .equals(other.edgeTransitionParam)) {
      return false;
    }
    if (thisState.onRoadModelCovarianceParam == null) {
      if (other.onRoadModelCovarianceParam != null) {
        return false;
      }
    } else if (!thisState.onRoadModelCovarianceParam
        .equals(other.onRoadModelCovarianceParam)) {
      return false;
    }
    if (thisState.observationCovarianceParam == null) {
      if (other.observationCovarianceParam != null) {
        return false;
      }
    } else if (!thisState.observationCovarianceParam
        .equals(other.observationCovarianceParam)) {
      return false;
    }
    if (thisState.offRoadModelCovarianceParam == null) {
      if (other.offRoadModelCovarianceParam != null) {
        return false;
      }
    } else if (!thisState.offRoadModelCovarianceParam
        .equals(other.offRoadModelCovarianceParam)) {
      return false;
    }
    if (thisState.observation == null) {
      if (other.observation != null) {
        return false;
      }
    } else if (!thisState.observation.equals(other.observation)) {
      return false;
    }
    return true;
  }

  protected static int oneStateHashCode(
    VehicleState<? extends GpsObservation> state) {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((state.motionStateParam == null) ? 0
                : state.motionStateParam.hashCode());
    result =
        prime
            * result
            + ((state.onRoadModelCovarianceParam == null) ? 0
                : state.onRoadModelCovarianceParam.hashCode());
    result =
        prime
            * result
            + ((state.observationCovarianceParam == null) ? 0
                : state.observationCovarianceParam.hashCode());
    result =
        prime
            * result
            + ((state.offRoadModelCovarianceParam == null) ? 0
                : state.offRoadModelCovarianceParam.hashCode());
    result =
        prime
            * result
            + ((state.edgeTransitionParam == null) ? 0
                : state.edgeTransitionParam.hashCode());
    result =
        prime
            * result
            + ((state.observation == null) ? 0 : state.observation
                .hashCode());
    return result;
  }

  /*-
   * These parameters are for the motion state, comprised of the 4D ground-coordinates dist. for free motion or the 2D
   * road-coordinates.  This distribution, however, has nothing to do with paths or edges, only
   * generalized motion on or off of a road. 
   */
  protected BayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam;
  
  /*-
   * These parameters are for the PathState  
   */
  protected BayesianParameter<PathState, PathStateMultivariateMixtureDensityModel<PathStateDistribution>, PathStateDistribution> pathStateParam;

  /*-
   * E.g. GPS error distribution 
   */
  protected BayesianParameter<Matrix, ?, ?> observationCovarianceParam;

  /*-
   * E.g. acceleration error distribution
   */
  protected BayesianParameter<Matrix, ?, ?> onRoadModelCovarianceParam;

  protected BayesianParameter<Matrix, ?, ?> offRoadModelCovarianceParam;

  /*-
   * E.g. edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected BayesianParameter<TransitionProbMatrix, 
      OnOffEdgeTransDistribution, 
      OnOffEdgeTransPriorDistribution> edgeTransitionParam;

  protected Observation observation = null;

  protected VehicleState<Observation> parentState = null;

  protected InferenceGraph graph = null;

  protected int hash = 0;
  public VehicleState(
    InferenceGraph inferredGraph,
    Observation observation,
    BayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam,
    BayesianParameter<PathState, PathStateMultivariateMixtureDensityModel<PathStateDistribution>, PathStateDistribution> pathStateParam,
    BayesianParameter<Matrix, ?, ?> observationCovParam,
    BayesianParameter<Matrix, ?, ?> onRoadModelCovParam,
    BayesianParameter<Matrix, ?, ?> offRoadModelCovParam,
    BayesianParameter<TransitionProbMatrix, 
        OnOffEdgeTransDistribution, 
        OnOffEdgeTransPriorDistribution> edgeTransitionDist,
    VehicleState<Observation> parentState) {

    this.graph = inferredGraph;
    this.observation = observation;

    this.motionStateParam = motionStateParam;
    this.pathStateParam = pathStateParam;
    this.observationCovarianceParam = observationCovParam;
    this.onRoadModelCovarianceParam = onRoadModelCovParam;
    this.offRoadModelCovarianceParam = offRoadModelCovParam;

    this.edgeTransitionParam = edgeTransitionDist;

    this.parentState = parentState;
  }

  public VehicleState(VehicleState<Observation> other) {
    this.graph = other.graph;
    this.motionStateParam = ObjectUtil.cloneSmart(other.motionStateParam);
    this.observationCovarianceParam =
        ObjectUtil.cloneSmart(other.observationCovarianceParam);
    this.onRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(other.onRoadModelCovarianceParam);
    this.offRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(other.offRoadModelCovarianceParam);
    this.edgeTransitionParam = ObjectUtil.cloneSmart(other.edgeTransitionParam);
    this.observation = other.observation;
    this.parentState = other.parentState;
  }

  @Override
  public VehicleState<Observation> clone() {
    return new VehicleState<Observation>(this);
  }

  @Override
  public int compareTo(VehicleState<Observation> arg0) {
    return oneStateCompareTo(this, arg0);
  }

  @Override
  public boolean equals(Object obj) {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (!oneStateEquals(this, obj))
      return false;

    final VehicleState<Observation> other =
        (VehicleState<Observation>) obj;
    if (parentState == null) {
      if (other.parentState != null) {
        return false;
      }
    } else if (!oneStateEquals(parentState, other.parentState)) {
      return false;
    }

    return true;
  }

  public
      BayesianParameter<TransitionProbMatrix, 
          OnOffEdgeTransDistribution, 
          OnOffEdgeTransPriorDistribution>
      getEdgeTransitionParam() {
    return edgeTransitionParam;
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
    final Vector v = ((PathState)motionStateParam.getValue()).getGroundState();
    return MotionStateEstimatorPredictor.getOg().times(v);
  }

  public Observation getObservation() {
    return observation;
  }

  public BayesianParameter<? extends Matrix, ?, ?>
      getObservationCovarianceParam() {
    return observationCovarianceParam;
  }

  public BayesianParameter<? extends Matrix, ?, ?>
      getOffRoadModelCovarianceParam() {
    return offRoadModelCovarianceParam;
  }

  public BayesianParameter<? extends Matrix, ?, ?>
      getOnRoadModelCovarianceParam() {
    return onRoadModelCovarianceParam;
  }

  public VehicleState<Observation> getParentState() {
    return parentState;
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
        result = prime * result + oneStateHashCode(this.parentState);
      hash = result;
      return result;
    }
  }

  public
      void
      setEdgeTransitionParam(
        BayesianParameter<TransitionProbMatrix, 
            OnOffEdgeTransDistribution, 
            OnOffEdgeTransPriorDistribution> edgeTransitionParam) {
    this.edgeTransitionParam = edgeTransitionParam;
  }

  public void setGraph(InferenceGraph graph) {
    this.graph = graph;
  }

  public void setObservation(Observation observation) {
    this.observation = observation;
  }

  public
      void
      setObservationCovarianceParam(
        BayesianParameter<Matrix, ?, ?> observationCovarianceParam) {
    this.observationCovarianceParam =
        observationCovarianceParam;
  }

  public
      BayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian>
      getMotionStateParam() {
    return motionStateParam;
  }

  public
      void
      setMotionStateParam(
        BayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam) {
    this.motionStateParam = motionStateParam;
  }

  public
      void
      setPathStateParam(
        BayesianParameter<PathState, PathStateMultivariateMixtureDensityModel<PathStateDistribution>, PathStateDistribution> pathStateParam) {
    this.pathStateParam = pathStateParam;
  }

  public
      void
      setOffRoadModelCovarianceParam(
        BayesianParameter<Matrix, ?, ?> offRoadModelCovarianceParam) {
    this.offRoadModelCovarianceParam =
        offRoadModelCovarianceParam;
  }

  public
      void
      setOnRoadModelCovarianceParam(
        BayesianParameter<Matrix, ?, ?> onRoadModelCovarianceParam) {
    this.onRoadModelCovarianceParam =
        onRoadModelCovarianceParam;
  }

  public void setParentState(VehicleState<Observation> parentState) {
    this.parentState = parentState;
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("belief", motionStateParam);
    builder.append("observation", observation);
    return builder.toString();
  }

}
