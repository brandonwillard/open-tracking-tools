package org.opentrackingtools;

import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.DefaultBayesianParameter;
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
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.updater.VehicleTrackingPLFilterUpdater;

import com.google.common.base.Preconditions;

public class VehicleStatePLFilter<O extends GpsObservation> extends
    AbstractParticleFilter<O, VehicleState<O>> {

  private static final long serialVersionUID = -8257075186193062150L;

  protected final InferenceGraph inferredGraph;
  protected final Boolean isDebug;

  public VehicleStatePLFilter(O obs, InferenceGraph inferredGraph,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleTrackingPLFilterUpdater<O>(obs,
        inferredGraph, parameters, rng));
  }

  @Override
  public void update(DataDistribution<VehicleState<O>> target, O obs) {

    /*
     * Compute predictive distributions, and create a distribution out of those and
     * their likelihoods for the new observation.
     */
    final CountedDataDistribution<VehicleState<O>> resampleDist =
        new CountedDataDistribution<VehicleState<O>>(true);

    for (final VehicleState<O> state : target.getDomain()) {

      final int count;
      if (target instanceof CountedDataDistribution<?>) {
        count =
            ((CountedDataDistribution) target).getCount(state);
      } else {
        count = 1;
      }

      final double logCount = Math.log(count);

      final VehicleState<O> predictedState =
          new VehicleState<O>(state);
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
        final VehicleState<O> predictedChildState =
            new VehicleState<O>(predictedState);
        predictedChildState
            .setPathStateParam(DefaultBayesianParameter.create(
                predictedPathStateMixture, predictedChildState
                    .getPathStateParam().getName(),
                predictedPathStateDist));

        final MultivariateGaussian measurementPredictionDist =
            predictedState.getMotionStateEstimatorPredictor()
                .getMeasurementBelief(predictedPathStateDist);

        predictedChildState
            .setMotionStateParam(DefaultBayesianParameter.create(
                measurementPredictionDist, predictedChildState
                    .getMotionStateParam().getName(),
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
    final ArrayList<VehicleState<O>> smoothedStates =
        resampleDist.sample(rng, this.getNumParticles());

    target.clear();

    /*
     * Propagate/smooth the best states. 
     */
    for (final VehicleState<O> state : smoothedStates) {

      final VehicleState<O> updateadState = state.clone();
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