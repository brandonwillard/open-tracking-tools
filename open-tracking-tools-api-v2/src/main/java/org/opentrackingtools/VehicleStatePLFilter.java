package org.opentrackingtools;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Random;

import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.estimators.OnOffEdgeTransitionEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.updater.VehicleStatePLUpdater;

import com.google.common.base.Preconditions;

public class VehicleStatePLFilter<O extends GpsObservation> extends
    AbstractParticleFilter<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = -8257075186193062150L;

  protected final InferenceGraph inferredGraph;
  protected final Boolean isDebug;

  public VehicleStatePLFilter(O obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleStatePLUpdater<O>(obs,
        inferredGraph, parameters, rng));
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

      final VehicleStateDistribution<O> predictedState =
          new VehicleStateDistribution<O>(state);
      predictedState.setObservation(obs);
      this.updater.update(predictedState);

      final PathStateMixtureDensityModel<PathStateDistribution> predictedPathStateMixture =
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
            this.getUpdater().computeLogLikelihood(predictedState,
                obs);

        resampleDist.increment(predictedChildState,
            predictiveLogLikelihood + pathStateDistLogLikelihood
                + edgeTransitionLogLikelihood + logCount);
      }
    }

    final Random rng = this.getRandom();

    Preconditions.checkState(!resampleDist.isEmpty());

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

      final VehicleStateDistribution<O> updateadState = state.clone();
      final PathStateDistribution pathStateDist =
          updateadState.getPathStateParam().getParameterPrior();

      /*
       * Update motion and path state by first updating the ground
       * coordinates, then reprojecting onto the edge.
       */
      final MultivariateGaussian updatedMotionState =
          pathStateDist.getGroundBelief().clone();
      updateadState.getMotionStateEstimatorPredictor().update(
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
      updateadState.getPathStateParam().setValue(
          pathStateDist.getPathState());

      final InferenceGraphEdge graphEdge =
          pathStateDist.getPathState().getEdge().getInferenceGraphEdge();
      final OnOffEdgeTransitionEstimatorPredictor edgeTransitionEstimatorPredictor =
          new OnOffEdgeTransitionEstimatorPredictor(updateadState,
              graphEdge);
      edgeTransitionEstimatorPredictor.update(updateadState
          .getEdgeTransitionParam().getParameterPrior(), graphEdge);

      target.increment(updateadState, 0d);
    }

  }

}