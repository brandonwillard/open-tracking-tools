package org.opentrackingtools.model;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DistributionWithMean;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.Collections;
import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;
import org.opentrackingtools.VehicleStateInitialParameters;
import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.distributions.ScaledInverseGammaCovDistribution;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.paths.Path;
import org.opentrackingtools.paths.PathEdge;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.model.TransitionProbMatrix;

import com.google.common.base.Preconditions;

/**
 * This object represents a joint distribution over the basic components of a
 * vehicle state:
 * <ul>
 * <li>motion state:
 * <ul>
 * <li>location and velocity</li>
 * </ul>
 * </li>
 * <li>path state:
 * <ul>
 * <li>edge and path</li>
 * <li>also encapsulates the motion state conditioned on the path, which can
 * differ from the raw motion state in that the conditioned motion state is more
 * likely to be on a given edge or path</li>
 * </ul>
 * </li>
 * </ul>
 * 
 * @author bwillard
 * 
 */
public class VehicleStateDistribution<Observation extends GpsObservation>
    extends AbstractCloneableSerializable implements
    Comparable<VehicleStateDistribution<Observation>> {

  public static class VehicleStateDistributionFactory<O extends GpsObservation, G extends InferenceGraph> {

    /**
     * This is where an initial off-road state is constructed so that it can be
     * used as a template for the prior initialization of parameters.
     * 
     * @param parameters
     * 
     * @return
     */
    public VehicleStateDistribution<O> createInitialVehicleState(
      VehicleStateInitialParameters parameters, G graph, O obs,
      Random rng, PathEdge pathEdge) {

      /*
       * Create parameters without path or motion state dependencies
       */
      final int obsInitialDof =
          parameters.getObsCovDof() - 1;
      final DistributionWithMean<Matrix> obsCovDistribution =
          new ScaledInverseGammaCovDistribution(2,
              parameters.getObsCovDof(),
              MatrixFactory.getDefault()
              .createDiagonal(parameters.getObsCov())
              .scale(obsInitialDof).getElement(0, 0));
      final SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> observationCovParam =
          SimpleBayesianParameter.create(
              obsCovDistribution.getMean(), new MultivariateGaussian(
                  VectorFactory.getDefault().createVector2D(),
                  obsCovDistribution.getMean()), obsCovDistribution);

      final int onRoadInitialDof = parameters.getOnRoadCovDof() - 1; 
      final DistributionWithMean<Matrix> onRoadCovDistribution =
          new ScaledInverseGammaCovDistribution(1, 
              parameters.getOnRoadCovDof(), 
              MatrixFactory.getDefault()
              .createDiagonal(parameters.getOnRoadStateCov())
              .scale(onRoadInitialDof).getElement(0, 0) );
      final SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> onRoadCovParam =
          SimpleBayesianParameter
              .create(
                  onRoadCovDistribution.getMean(),
                  new MultivariateGaussian(VectorFactory.getDefault()
                      .createVector1D(), onRoadCovDistribution
                      .getMean()), onRoadCovDistribution);

      final int offRoadInitialDof = parameters.getOffRoadCovDof() - 1;
      final DistributionWithMean<Matrix> offRoadCovDistribution =
          new ScaledInverseGammaCovDistribution(2, 
              parameters.getOffRoadCovDof(),
              MatrixFactory.getDefault().createDiagonal(parameters.getOffRoadStateCov())
              .scale(offRoadInitialDof).getElement(0, 0));
      final SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> offRoadCovParam =
          SimpleBayesianParameter
              .create(
                  offRoadCovDistribution.getMean(),
                  new MultivariateGaussian(VectorFactory.getDefault()
                      .createVector1D(), offRoadCovDistribution
                      .getMean()), offRoadCovDistribution);

      final OnOffEdgeTransPriorDistribution initialPriorTransDist =
          new OnOffEdgeTransPriorDistribution(
              parameters.getOnTransitionProbsPrior(),
              parameters.getOffTransitionProbsPrior());
      final TransitionProbMatrix transitionProbMatrix =
          new TransitionProbMatrix(initialPriorTransDist
              .getEdgeMotionTransProbPrior().getMean(),
              initialPriorTransDist.getFreeMotionTransProbPrior()
                  .getMean());

      /*
       * Create incomplete state that is to be filled out depending
       * on the edge.
       */
      final VehicleStateDistribution<O> state =
          new VehicleStateDistribution<O>(graph, obs, null, null,
              observationCovParam, onRoadCovParam, offRoadCovParam,
              null, null);

      final MotionStateEstimatorPredictor motionStateEstimatorPredictor =
          new MotionStateEstimatorPredictor(state, rng,
              parameters.getInitialObsFreq());

      final MultivariateGaussian initialMotionStateDist =
          motionStateEstimatorPredictor.createInitialLearnedObject();
      if (parameters.getInitialMotionState() != null) {
        Preconditions.checkArgument(parameters
            .getInitialMotionState().getDimensionality() == 4);
        initialMotionStateDist.setMean(parameters
            .getInitialMotionState().clone());
      }

      /*
       * Now, handle the off and on road motion and path state creation 
       */
      if (pathEdge.isNullEdge()) {

        final MultivariateGaussian initialObservationState =
            motionStateEstimatorPredictor.getObservationDistribution(
                initialMotionStateDist, PathEdge.nullPathEdge);

        final SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam =
            SimpleBayesianParameter.create(
                initialObservationState.getMean(),
                initialObservationState, initialMotionStateDist);
        state.setMotionStateParam(motionStateParam);

        final PathStateDistribution initialPathStateDist =
            new PathStateDistribution(Path.nullPath,
                initialMotionStateDist);
        final SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution> pathStateParam =
            SimpleBayesianParameter.create(initialPathStateDist
                .getPathState(), new PathStateMixtureDensityModel(
                Collections.singletonList(initialPathStateDist),
                new double[] { 0d }), initialPathStateDist);

        state.setPathStateParam(pathStateParam);

      } else {

        final Path path = new Path(pathEdge);
        PathUtils.convertToRoadBelief(initialMotionStateDist, path,
            pathEdge, true, null, null);

        final MultivariateGaussian initialObservationState =
            motionStateEstimatorPredictor.getObservationDistribution(
                initialMotionStateDist, pathEdge);

        final SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam =
            SimpleBayesianParameter.create(
                initialObservationState.getMean(),
                initialObservationState, initialMotionStateDist);

        state.setMotionStateParam(motionStateParam);

        final PathStateDistribution initialPathStateDist =
            new PathStateDistribution(path, initialMotionStateDist);
        final PathStateMixtureDensityModel pathStateMixture =
            new PathStateMixtureDensityModel(
                Collections.singleton(initialPathStateDist),
                new double[] { 0d });
        final SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution> edgeStateParam =
            SimpleBayesianParameter.create(
                initialPathStateDist.getPathState(),
                pathStateMixture, initialPathStateDist);

        state.setPathStateParam(edgeStateParam);
      }

      /*
       * Now set the transition distribution, since it needed a path state
       * for construction.
       */
      final OnOffEdgeTransDistribution initialTransDist =
          new OnOffEdgeTransDistribution(graph, state
              .getPathStateParam().getValue(), state
              .getObservationCovarianceParam().getValue(),
              initialPriorTransDist.getEdgeMotionTransProbPrior()
                  .getMean(), initialPriorTransDist
                  .getFreeMotionTransProbPrior().getMean());
      final SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> edgeTransitionParam =
          SimpleBayesianParameter.create(transitionProbMatrix,
              initialTransDist, initialPriorTransDist);

      state.setEdgeTransitionParam(edgeTransitionParam);

      return state;
    }
  }

  private static final long serialVersionUID = 3229140254421801273L;

  public static Vector getNonVelocityVector(Vector vector) {
    final Vector res;
    if (vector.getDimensionality() == 4) {
      res = MotionStateEstimatorPredictor.getOg().times(vector);
    } else {
      res = MotionStateEstimatorPredictor.getOr().times(vector);
    }
    return res;
  }

  public static long getSerialversionuid() {
    return VehicleStateDistribution.serialVersionUID;
  }

  protected static int oneStateCompareTo(
    VehicleStateDistribution<? extends GpsObservation> t,
    VehicleStateDistribution<? extends GpsObservation> o) {
    if (t == o) {
      return 0;
    }

    if (t == null) {
      if (o != null) {
        return -1;
      } else {
        return 0;
      }
    } else if (o == null) {
      return 1;
    }

    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(t.observation, o.observation);
    comparator.append(t.motionStateParam.value.toArray(),
        o.motionStateParam.value.toArray());
    comparator.append(t.pathStateParam.prior.getMotionDistribution()
        .convertToVector().toArray(), o.pathStateParam.prior
        .getMotionDistribution().convertToVector().toArray());
    comparator.append(t.observationCovarianceParam.value.toArray(),
        o.observationCovarianceParam.value.toArray());
    comparator.append(t.onRoadModelCovarianceParam.value.toArray(),
        o.onRoadModelCovarianceParam.value.toArray());
    comparator.append(t.offRoadModelCovarianceParam.value.toArray(),
        o.offRoadModelCovarianceParam.value.toArray());
    comparator.append(t.edgeTransitionParam.value,
        o.edgeTransitionParam.value);

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
    final VehicleStateDistribution<? extends GpsObservation> thisState =
        (VehicleStateDistribution<? extends GpsObservation>) thisObj;
    final VehicleStateDistribution<? extends GpsObservation> other =
        (VehicleStateDistribution<? extends GpsObservation>) obj;
    if (thisState.motionStateParam == null) {
      if (other.motionStateParam != null) {
        return false;
      }
    } else if (!thisState.motionStateParam
        .equals(other.motionStateParam)) {
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
    VehicleStateDistribution<? extends GpsObservation> state) {
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

  protected double edgeTransitionLogLikelihood;

  /*-
   * E.g. edge transition priors 
   * 1. edge off 
   * 2. edge on 
   * 3. edges transitions to others (one for all)
   * edges
   */
  protected SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> edgeTransitionParam;

  protected InferenceGraph graph = null;

  /*
   * Allow ourself to hold on to estimator predictors in case we want
   * to use them again.
   */
  protected MotionStateEstimatorPredictor motionStateEstimatorPredictor =
      null;

  /*-
   * These parameters are for the motion state, comprised of the 4D ground-coordinates dist. for free motion or the 2D
   * road-coordinates.  This distribution, however, has nothing to do with paths or edges, only
   * generalized motion on or off of a road. 
   */
  protected SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam;

  protected Observation observation = null;

  /*-
   * E.g. GPS error distribution 
   */
  protected SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> observationCovarianceParam;

  protected SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> offRoadModelCovarianceParam;

  /*-
   * E.g. acceleration error distributions
   */
  protected SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> onRoadModelCovarianceParam;
  protected VehicleStateDistribution<Observation> parentState = null;

  protected double pathStateDistLogLikelihood;

  /*-
   * These parameters are for the PathState  
   */
  protected SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution> pathStateParam;

  /*
   * Debug values
   */
  protected double predictiveLogLikelihood;
  protected VehicleStateDistribution<Observation> priorPredictiveState;
  /*
   * This holds the prior predictive transition states.  Again, debug.
   */
  protected CountedDataDistribution<? extends VehicleStateDistribution<Observation>> transitionStateDistribution;

  public VehicleStateDistribution(
    InferenceGraph inferredGraph,
    Observation observation,
    SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam,
    SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution> pathStateParam,
    SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> observationCovParam,
    SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> onRoadModelCovParam,
    SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> offRoadModelCovParam,
    SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> edgeTransitionDist,
    VehicleStateDistribution<Observation> parentState) {

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

  public VehicleStateDistribution(
    VehicleStateDistribution<Observation> other) {
    this.graph = other.graph;
    this.observation = other.observation;
    this.parentState = other.parentState;
    this.motionStateEstimatorPredictor =
        other.motionStateEstimatorPredictor;

    this.motionStateParam =
        ObjectUtil.cloneSmart(other.motionStateParam);
    this.pathStateParam = ObjectUtil.cloneSmart(other.pathStateParam);
    this.observationCovarianceParam =
        ObjectUtil.cloneSmart(other.observationCovarianceParam);
    this.onRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(other.onRoadModelCovarianceParam);
    this.offRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(other.offRoadModelCovarianceParam);
    this.edgeTransitionParam =
        ObjectUtil.cloneSmart(other.edgeTransitionParam);
    this.edgeTransitionLogLikelihood =
        other.edgeTransitionLogLikelihood;
    this.pathStateDistLogLikelihood =
        other.pathStateDistLogLikelihood;
    this.predictiveLogLikelihood = other.predictiveLogLikelihood;
  }

  @Override
  public VehicleStateDistribution<Observation> clone() {
    final VehicleStateDistribution<Observation> clone =
        (VehicleStateDistribution<Observation>) super.clone();
    clone.graph = this.graph;
    clone.observation = this.observation;
    clone.parentState = this.parentState;
    clone.motionStateEstimatorPredictor =
        this.motionStateEstimatorPredictor;

    clone.motionStateParam =
        ObjectUtil.cloneSmart(this.motionStateParam);
    clone.pathStateParam = ObjectUtil.cloneSmart(this.pathStateParam);
    clone.observationCovarianceParam =
        ObjectUtil.cloneSmart(this.observationCovarianceParam);
    clone.onRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(this.onRoadModelCovarianceParam);
    clone.offRoadModelCovarianceParam =
        ObjectUtil.cloneSmart(this.offRoadModelCovarianceParam);
    clone.edgeTransitionParam =
        ObjectUtil.cloneSmart(this.edgeTransitionParam);
    clone.edgeTransitionLogLikelihood =
        this.edgeTransitionLogLikelihood;
    clone.pathStateDistLogLikelihood =
        this.pathStateDistLogLikelihood;
    clone.predictiveLogLikelihood = this.predictiveLogLikelihood;

    return clone;
  }

  @Override
  public int compareTo(VehicleStateDistribution<Observation> arg0) {
    return VehicleStateDistribution.oneStateCompareTo(this, arg0);
  }

  @Override
  public boolean equals(Object obj) {
    /*
     * We do this to avoid evaluating every parent down the chain.
     */
    if (!VehicleStateDistribution.oneStateEquals(this, obj)) {
      return false;
    }

    final VehicleStateDistribution<Observation> other =
        (VehicleStateDistribution<Observation>) obj;
    if (this.parentState == null) {
      if (other.parentState != null) {
        return false;
      }
    } else if (!VehicleStateDistribution.oneStateEquals(
        this.parentState, other.parentState)) {
      return false;
    }

    return true;
  }

  public double getEdgeTransitionLogLikelihood() {
    return this.edgeTransitionLogLikelihood;
  }

  public
      SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution>
      getEdgeTransitionParam() {
    return this.edgeTransitionParam;
  }

  public InferenceGraph getGraph() {
    return this.graph;
  }

  /**
   * Returns ground-coordinate mean location
   * 
   * @return
   */
  public Vector getMeanLocation() {
    return this.motionStateParam.getValue();
  }

  public MotionStateEstimatorPredictor
      getMotionStateEstimatorPredictor() {
    return this.motionStateEstimatorPredictor;
  }

  public
      SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian>
      getMotionStateParam() {
    return this.motionStateParam;
  }

  public Observation getObservation() {
    return this.observation;
  }

  public
      SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>>
      getObservationCovarianceParam() {
    return this.observationCovarianceParam;
  }

  public
      SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>>
      getOffRoadModelCovarianceParam() {
    return this.offRoadModelCovarianceParam;
  }

  public
      SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>>
      getOnRoadModelCovarianceParam() {
    return this.onRoadModelCovarianceParam;
  }

  public VehicleStateDistribution<Observation> getParentState() {
    return this.parentState;
  }

  public double getPathStateDistLogLikelihood() {
    return this.pathStateDistLogLikelihood;
  }

  public
      SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution>
      getPathStateParam() {
    return this.pathStateParam;
  }

  public double getPredictiveLogLikelihood() {
    return this.predictiveLogLikelihood;
  }

  public
      CountedDataDistribution<? extends VehicleStateDistribution<Observation>>
      getTransitionStateDistribution() {
    return this.transitionStateDistribution;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime * result
            + VehicleStateDistribution.oneStateHashCode(this);
    if (this.parentState != null) {
      result =
          prime
              * result
              + VehicleStateDistribution
                  .oneStateHashCode(this.parentState);
    }
    return result;
  }

  public void setEdgeTransitionLogLikelihood(
    double edgeTransitionLogLikelihood) {
    this.edgeTransitionLogLikelihood = edgeTransitionLogLikelihood;
  }

  public
      void
      setEdgeTransitionParam(
        SimpleBayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> edgeTransitionParam) {
    this.edgeTransitionParam = edgeTransitionParam;
  }

  public void setGraph(InferenceGraph graph) {
    this.graph = graph;
  }

  public void setMotionStateEstimatorPredictor(
    MotionStateEstimatorPredictor motionStateEstimatorPredictor) {
    this.motionStateEstimatorPredictor =
        motionStateEstimatorPredictor;
  }

  public
      void
      setMotionStateParam(
        SimpleBayesianParameter<Vector, MultivariateGaussian, MultivariateGaussian> motionStateParam) {
    this.motionStateParam = motionStateParam;
  }

  public void setObservation(Observation observation) {
    this.observation = observation;
  }

  public
      void
      setObservationCovarianceParam(
        SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> observationCovarianceParam) {
    this.observationCovarianceParam = observationCovarianceParam;
  }

  public
      void
      setOffRoadModelCovarianceParam(
        SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> offRoadModelCovarianceParam) {
    this.offRoadModelCovarianceParam = offRoadModelCovarianceParam;
  }

  public
      void
      setOnRoadModelCovarianceParam(
        SimpleBayesianParameter<Matrix, MultivariateGaussian, DistributionWithMean<Matrix>> onRoadModelCovarianceParam) {
    this.onRoadModelCovarianceParam = onRoadModelCovarianceParam;
  }

  public void setParentState(
    VehicleStateDistribution<Observation> parentState) {
    this.parentState = parentState;
  }

  public void setPathStateDistLogLikelihood(
    double pathStateDistLogLikelihood) {
    this.pathStateDistLogLikelihood = pathStateDistLogLikelihood;
    ;
  }

  public
      void
      setPathStateParam(
        SimpleBayesianParameter<PathState, PathStateMixtureDensityModel, PathStateDistribution> pathStateParam) {
    this.pathStateParam = pathStateParam;
  }

  public void setPredictiveLogLikelihood(
    double predictiveLogLikelihood) {
    this.predictiveLogLikelihood = predictiveLogLikelihood;
  }

  public void setPriorPredictiveState(
    VehicleStateDistribution<Observation> priorPredState) {
    this.priorPredictiveState = priorPredState;
  }

  public
      void
      setTransitionStateDistribution(
        CountedDataDistribution<? extends VehicleStateDistribution<Observation>> transitionStateDistribution) {
    this.transitionStateDistribution = transitionStateDistribution;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder =
        new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.append("pathState",
        this.pathStateParam.getParameterPrior());
    builder.append("observation", this.observation);
    return builder.toString();
  }

}
