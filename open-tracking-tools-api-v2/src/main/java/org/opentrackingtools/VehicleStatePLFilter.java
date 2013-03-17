package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Random;

import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.estimators.OnOffEdgeTransitionEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.updater.VehicleStatePLUpdater;
import org.opentrackingtools.util.TrueObservation;

import com.google.common.base.Preconditions;

public class VehicleStatePLFilter<O extends GpsObservation> extends
    AbstractParticleFilter<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = -8257075186193062150L;

  protected final InferenceGraph inferredGraph;
  protected final Boolean isDebug;

  /*
   * Keep the last particle resample distribution (when debugging is on).
   * This distribution contains all the last evaluated transition states.
   */
  protected CountedDataDistribution<VehicleStateDistribution<O>> lastResampleDistribution;

  public CountedDataDistribution<VehicleStateDistribution<O>>
      getLastResampleDistribution() {
    return lastResampleDistribution;
  }

  public
      void
      setLastResampleDistribution(
        CountedDataDistribution<VehicleStateDistribution<O>> lastResampleDistribution) {
    this.lastResampleDistribution = lastResampleDistribution;
  }

  public Boolean getIsDebug() {
    return isDebug;
  }

  public VehicleStatePLFilter(O obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleStatePLUpdater<O>(obs,
        inferredGraph, parameters, rng));
    this.setNumParticles(parameters.getNumParticles());
    this.setRandom(rng);
  }

  @Override
  public void update(DataDistribution<VehicleStateDistribution<O>> target, O obs) {

    /*
     * Compute predictive distributions, and create a distribution out of those and
     * their likelihoods for the new observation.
     */
    final CountedDataDistribution<VehicleStateDistribution<O>> resampleDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

    for (final VehicleStateDistribution<O> state : target.getDomain()) {

      final int count;
      if (target instanceof CountedDataDistribution<?>) {
        count =
            ((CountedDataDistribution) target).getCount(state);
      } else {
        count = 1;
      }

      final double logCount = Math.log(count);

      VehicleStateDistribution<O> predictedState = new VehicleStateDistribution<O>(state);
      predictedState.setObservation(obs);
      predictedState = this.updater.update(predictedState);

      final PathStateMixtureDensityModel predictedPathStateMixture =
          predictedState.getPathStateParam()
              .getConditionalDistribution();
      for (int i = 0; i < predictedPathStateMixture
          .getDistributionCount(); i++) {
        final PathStateDistribution predictedPathStateDist =
            predictedPathStateMixture.getDistributions().get(i);
        final double pathStateDistLogLikelihood =
            predictedPathStateMixture.getPriorWeights()[i];
        final double edgeTransitionLogLikelihood =
            predictedState
                .getEdgeTransitionParam()
                .getConditionalDistribution()
                .getProbabilityFunction()
                .logEvaluate(
                    predictedPathStateDist.getPathState().getEdge()
                        .getInferenceGraphEdge());
        
        /*
         * Create a new vehicle state for this possible transition
         * path state, and, since it starts as a copy of the original
         * state, make sure we update the pertinent parameters.
         */
        final VehicleStateDistribution<O> predictedChildState =
            new VehicleStateDistribution<O>(predictedState);
        predictedChildState
            .setPathStateParam(SimpleBayesianParameter.create(predictedPathStateDist.getPathState(),
                predictedPathStateMixture, predictedPathStateDist));

        final MultivariateGaussian measurementPredictionDist =
            predictedState.getMotionStateEstimatorPredictor()
                .getMeasurementBelief(predictedPathStateDist);

        predictedChildState
            .setMotionStateParam(SimpleBayesianParameter.create(
                measurementPredictionDist.getMean(),
                measurementPredictionDist, 
                predictedPathStateDist.getMotionStateDistribution()));

        final double predictiveLogLikelihood =
            this.getUpdater().computeLogLikelihood(predictedChildState,
                obs);
        
        predictedChildState.setParentState(state);

        resampleDist.increment(predictedChildState,
            predictiveLogLikelihood + pathStateDistLogLikelihood
                + edgeTransitionLogLikelihood + logCount);
      }
    }

    final Random rng = this.getRandom();

    Preconditions.checkState(!resampleDist.isEmpty());
    
    if (this.isDebug)
      this.lastResampleDistribution = resampleDist;

    /*
     * Resample the predictive distributions.  Now we're dealing with the "best" states.
     */
    final ArrayList<VehicleStateDistribution<O>> smoothedStates =
        resampleDist.sample(rng, this.getNumParticles());

    target.clear();

    /*
     * Propagate/smooth the best states. 
     */
    for (final VehicleStateDistribution<O> state : smoothedStates) {

      final VehicleStateDistribution<O> updatedState = state.clone();
      final PathStateDistribution pathStateDist =
          updatedState.getPathStateParam().getParameterPrior();

      /*
       * Update motion and path state by first updating the ground
       * coordinates, then reprojecting onto the edge.
       */
      final MultivariateGaussian updatedMotionState =
          pathStateDist.getGroundBelief().clone();
      updatedState.getMotionStateEstimatorPredictor().update(
          updatedMotionState, obs.getProjectedPoint());

      final PathStateEstimatorPredictor pathStateEstimator =
          new PathStateEstimatorPredictor(state, pathStateDist
              .getPathState().getPath());

      pathStateDist.setGroundDistribution(updatedMotionState);

      pathStateEstimator.update(pathStateDist,
          obs.getProjectedPoint());

      /*
       * Update parameters
       */
      updatedState.getPathStateParam().setValue(
          pathStateDist.getPathState());
      updatedState.getPathStateParam().setParameterPrior(pathStateDist);
      updatedState.getMotionStateParam().setParameterPrior(pathStateDist.getMotionStateDistribution());
      MultivariateGaussian obsMotionDist =
        updatedState.getMotionStateEstimatorPredictor().getObservationDistribution(
            pathStateDist.getMotionStateDistribution(), pathStateDist.getPathState().getEdge());
      updatedState.getMotionStateParam().setConditionalDistribution(obsMotionDist);
      updatedState.getMotionStateParam().setValue(obsMotionDist.getMean());

      final InferenceGraphEdge graphEdge =
          updatedState.getParentState().getPathStateParam().getValue().getEdge().getInferenceGraphEdge();
      final OnOffEdgeTransitionEstimatorPredictor edgeTransitionEstimatorPredictor =
          new OnOffEdgeTransitionEstimatorPredictor(updatedState,
              graphEdge);
      
      OnOffEdgeTransPriorDistribution prior = updatedState
          .getEdgeTransitionParam().getParameterPrior();
      edgeTransitionEstimatorPredictor.update(prior, graphEdge);
      updatedState.getEdgeTransitionParam().getConditionalDistribution().
        setFreeMotionTransProbs(prior.getFreeMotionTransProbPrior().getMean());
      updatedState.getEdgeTransitionParam().getConditionalDistribution().
        setEdgeMotionTransProbs(prior.getEdgeMotionTransProbPrior().getMean());
      updatedState.getEdgeTransitionParam().setValue(prior.getMean());
      
      // TODO covariance updates

      target.increment(updatedState, 0d);
    }
    
    Preconditions.checkState(target.getDomainSize() > 0);

  }

}