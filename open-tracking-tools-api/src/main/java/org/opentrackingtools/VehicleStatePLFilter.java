package org.opentrackingtools;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.bayesian.AbstractKalmanFilter;
import gov.sandia.cognition.statistics.bayesian.AbstractParticleFilter;
import gov.sandia.cognition.statistics.bayesian.BayesianCredibleInterval;
import gov.sandia.cognition.statistics.distribution.InverseWishartDistribution;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map.Entry;
import java.util.Random;

import org.opentrackingtools.distributions.CountedDataDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.distributions.PathStateDistribution;
import org.opentrackingtools.distributions.PathStateMixtureDensityModel;
import org.opentrackingtools.distributions.TruncatedRoadGaussian;
import org.opentrackingtools.estimators.MotionStateEstimatorPredictor;
import org.opentrackingtools.estimators.OnOffEdgeTransitionEstimatorPredictor;
import org.opentrackingtools.estimators.PathStateEstimatorPredictor;
import org.opentrackingtools.estimators.RoadMeasurementCovarianceEstimatorPredictor;
import org.opentrackingtools.estimators.RoadModelCovarianceEstimatorPredictor;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.SimpleBayesianParameter;
import org.opentrackingtools.model.VehicleStateDistribution;
import org.opentrackingtools.model.VehicleStateDistribution.VehicleStateDistributionFactory;
import org.opentrackingtools.paths.PathState;
import org.opentrackingtools.updater.VehicleStatePLUpdater;
import org.opentrackingtools.util.PathUtils;
import org.opentrackingtools.util.TrueObservation;
import org.opentrackingtools.util.model.MutableDoubleCount;

import com.beust.jcommander.internal.Lists;
import com.google.common.base.Preconditions;

public class VehicleStatePLFilter<O extends GpsObservation, G extends InferenceGraph> extends
    AbstractParticleFilter<O, VehicleStateDistribution<O>> {

  private static final long serialVersionUID = -8257075186193062150L;

  protected final G inferredGraph;
  protected final Boolean isDebug;

  /*
   * Keep the last particle resample distribution (when debugging is on).
   * This distribution contains all the last evaluated transition states.
   */
  protected CountedDataDistribution<VehicleStateDistribution<O>> lastResampleDistribution;

  protected VehicleStateDistributionFactory<O, G> vehicleStateFactory;

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

  public VehicleStatePLFilter(O obs, G inferredGraph, VehicleStateDistributionFactory<O,G> vehicleStateFactory,
    VehicleStateInitialParameters parameters, Boolean isDebug,
    Random rng) {
    this.vehicleStateFactory = vehicleStateFactory;
    this.inferredGraph = inferredGraph;
    this.isDebug = isDebug;
    this.setUpdater(new VehicleStatePLUpdater<O, G>(obs, inferredGraph, 
        vehicleStateFactory,
        parameters, rng));
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

      VehicleStateDistribution<O> predictedState = state.clone();
      predictedState.setParentState(state);
      predictedState.setObservation(obs);
      predictedState = this.updater.update(predictedState);
      
      CountedDataDistribution<VehicleStateDistribution<O>> childDist = internalPriorPrediction(predictedState, obs);
      predictedState.setTransitionStateDistribution(childDist);
      
      resampleDist.increment(predictedState, childDist.getTotal() + logCount);
      
    }

    Preconditions.checkState(!resampleDist.isEmpty());
    
    if (this.isDebug)
      this.lastResampleDistribution = resampleDist;
    
    // TODO debug. remove.
    if (obs instanceof TrueObservation) {
      final VehicleStateDistribution<?> trueState = ((TrueObservation)obs).getTrueState();
      if (!trueState.getPathStateParam().getValue().getEdge().
          equals(resampleDist.getMaxValueKey().getTransitionStateDistribution().getMaxValueKey().
              getPathStateParam().getValue().getEdge())) {
        List<Entry<VehicleStateDistribution<O>, MutableDouble>> sortedDist = Lists.newArrayList(resampleDist.asMap().entrySet());
        Collections.sort(sortedDist, 
            new Comparator<Entry<VehicleStateDistribution<O>, MutableDouble>>() {
              @Override
              public int compare(
                Entry<VehicleStateDistribution<O>, MutableDouble> o1,
                Entry<VehicleStateDistribution<O>, MutableDouble> o2) {
                final double adjValue1 = o1.getValue().getValue() + Math.log(((MutableDoubleCount)o1.getValue()).getCount());
                final double adjValue2 = o2.getValue().getValue() + Math.log(((MutableDoubleCount)o2.getValue()).getCount());
                return -Double.compare(adjValue1, adjValue2);
              }
            } 
        );
        for (Entry<VehicleStateDistribution<O>, MutableDouble>  entry : sortedDist) {
          final VehicleStateDistribution<O> state = entry.getKey().getTransitionStateDistribution().getMaxValueKey();
          final Vector distToTrueState = state.getPathStateParam().getParameterPrior().getGroundDistribution().
              getMean().minus(trueState.getPathStateParam().getValue().getGroundState());
          final int count = ((MutableDoubleCount)entry.getValue()).getCount();
          final double adjValue = entry.getValue().getValue() + Math.log(count);
          System.out.println(resampleDist.getLogFraction(entry.getKey()) + " [" + adjValue + "] (" + count + "),\n\t [" 
            + distToTrueState + "]:\n\t" 
            + state.getPathStateParam().getValue() + 
            "\n\t pathStateLik=" + state.getPathStateDistLogLikelihood() +
            "\n\t obsLik=" + state.getPredictiveLogLikelihood()
          );
        }
        System.out.println(trueState.getPathStateParam().getValue());
      }
    }
    
    /*
     * Resample the predictive distributions.  Now we're dealing with the "best" states.
     */
    final ArrayList<VehicleStateDistribution<O>> smoothedStates =
        resampleDist.sample(this.random, this.getNumParticles());

    List<VehicleStateDistribution<O>> updatedStates = Lists.newArrayList();
    /*
     * Propagate/smooth the best states. 
     */
    for (final VehicleStateDistribution<O> state : smoothedStates) {
      final VehicleStateDistribution<O> sampledTransitionState = state.getTransitionStateDistribution().sample(
          this.random);
      VehicleStateDistribution<O> updatedState = internalUpdate(sampledTransitionState, obs);
      updatedState.setTransitionStateDistribution(state.getTransitionStateDistribution());
      updatedState.setPriorPredictiveState(sampledTransitionState);
      updatedStates.add(updatedState);
    }
    
    target.clear();
    target.incrementAll(updatedStates);
    
    Preconditions.checkState(target.getDomainSize() > 0);
    if (target instanceof CountedDataDistribution<?>) {
      Preconditions.checkState(((CountedDataDistribution<?>)target).getTotalCount() == this.numParticles);
    }

  }

  /**
   * This method takes a prior predictive vehicle state distribution and
   * returns a distribution over its possible transition
   * states, with prior predictive likelihood weights.
   * 
   * @param predictedState
   * @param obs
   * @return
   */
  protected CountedDataDistribution<VehicleStateDistribution<O>> internalPriorPrediction(
      VehicleStateDistribution<O> predictedState, O obs) {
    final CountedDataDistribution<VehicleStateDistribution<O>> childDist =
        new CountedDataDistribution<VehicleStateDistribution<O>>(true);

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
      final VehicleStateDistribution<O> predictedChildState = predictedState.clone();
      
      predictedChildState
          .setPathStateParam(SimpleBayesianParameter.create(predictedPathStateDist.getPathState(),
              predictedPathStateMixture, predictedPathStateDist));

      final MultivariateGaussian measurementPredictionDist =
          predictedState.getMotionStateEstimatorPredictor()
              .getMeasurementDistribution(predictedPathStateDist);

      predictedChildState
          .setMotionStateParam(SimpleBayesianParameter.create(
              measurementPredictionDist.getMean(),
              measurementPredictionDist, 
              predictedPathStateDist.getMotionDistribution()));

      final double predictiveLogLikelihood =
          this.getUpdater().computeLogLikelihood(predictedChildState,
              obs);
      
      predictedChildState.setParentState(predictedState.getParentState());

      /*
       * Set these for debugging.
       */
      predictedChildState.setPredictiveLogLikelihood(predictiveLogLikelihood);
      predictedChildState.setPathStateDistLogLikelihood(pathStateDistLogLikelihood);
      predictedChildState.setEdgeTransitionLogLikelihood(edgeTransitionLogLikelihood);
      
      final double childLogLikTotal = predictiveLogLikelihood + pathStateDistLogLikelihood
              + edgeTransitionLogLikelihood;
      
      childDist.increment(predictedChildState, childLogLikTotal);
    }
    
    return childDist;
  }

  /**
   * This method performs the Bayes update for a single vehicle state.
   * 
   * @param state
   * @param obs
   * @return
   */
  protected VehicleStateDistribution<O> internalUpdate(VehicleStateDistribution<O> state, O obs) {
    final VehicleStateDistribution<O> updatedState = state.clone();
    final PathStateDistribution priorPredictivePathStateDist =
        updatedState.getPathStateParam().getParameterPrior().clone();

    /*
     * Update motion and path state by first updating the ground
     * coordinates, then reprojecting onto the edge.
     * Scratch that.
     * For on-road, update by projecting the observation onto the path, then
     * perform a 2d update.
     */
    final MultivariateGaussian updatedMotionState = priorPredictivePathStateDist.getMotionDistribution().clone();
    if (priorPredictivePathStateDist.getPathState().isOnRoad()) {
//      PathUtils.convertToGroundBelief(updatedMotionState, pathStateDist.getPathState().getEdge(), true, false, true);
//      updatedState.getMotionStateEstimatorPredictor().update(
//          updatedMotionState, obs.getProjectedPoint());
      
      AbstractKalmanFilter roadFilter = updatedState.getMotionStateEstimatorPredictor().getRoadFilter().clone();
      
      final MultivariateGaussian obsProj =
          PathUtils.getRoadObservation(
              obs.getProjectedPoint(), updatedState.getObservationCovarianceParam().getValue(), 
              priorPredictivePathStateDist.getPathState().getPath(), 
              priorPredictivePathStateDist.getPathState().getEdge());

      roadFilter.setMeasurementCovariance(obsProj.getCovariance());
      roadFilter.measure(updatedMotionState, obsProj.getMean());
      
    } else {
      updatedState.getMotionStateEstimatorPredictor().update(
          updatedMotionState, obs.getProjectedPoint());
    }

    final PathStateDistribution posteriorPathStateDist = new PathStateDistribution(
        priorPredictivePathStateDist.getPathState().getPath(), updatedMotionState);
//    final PathStateEstimatorPredictor pathStateEstimator =
//        new PathStateEstimatorPredictor(state, pathStateDist);
//
//    pathStateDist.setGroundDistribution(updatedMotionState);
//
//    pathStateEstimator.update(pathStateDist,
//        obs.getProjectedPoint());

    /*
     * Update parameters
     */
    updatedState.getPathStateParam().setValue(
        posteriorPathStateDist.getPathState());
    updatedState.getPathStateParam().setParameterPrior(posteriorPathStateDist);
    updatedState.getMotionStateParam().setParameterPrior(posteriorPathStateDist.getMotionDistribution());
    MultivariateGaussian obsMotionDist =
      updatedState.getMotionStateEstimatorPredictor().getObservationDistribution(
          posteriorPathStateDist.getMotionDistribution(), posteriorPathStateDist.getPathState().getEdge());
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
    
    if (!(posteriorPathStateDist.getPathState().isOnRoad() &&
          state.getOnRoadModelCovarianceParam().getParameterPrior().getDegreesOfFreedom()
          >= Integer.MAX_VALUE-1)
          && !(!posteriorPathStateDist.getPathState().isOnRoad() &&
            state.getOffRoadModelCovarianceParam().getParameterPrior().getDegreesOfFreedom()
            >= Integer.MAX_VALUE-1)) {
      RoadModelCovarianceEstimatorPredictor modelCovarianceEstimator = new RoadModelCovarianceEstimatorPredictor(updatedState, 
          state.getMotionStateEstimatorPredictor(), this.random);
      
      final InverseWishartDistribution currentModelCovDistribution = posteriorPathStateDist.getPathState().isOnRoad() ?
          updatedState.getOnRoadModelCovarianceParam().getParameterPrior().clone() :
            updatedState.getOffRoadModelCovarianceParam().getParameterPrior().clone();
      modelCovarianceEstimator.update(currentModelCovDistribution, obs.getProjectedPoint());
      
      /*
       * After updating the covariance priors, we need to sample our new covariance matrix.
       * Also, note that we really have separate on/off covariances.  We could project
       * back and forth, then we'd really only have one.
       */
      if (posteriorPathStateDist.getPathState().isOnRoad()) {
        updatedState.getOnRoadModelCovarianceParam().setParameterPrior(currentModelCovDistribution);
        final Matrix stateCovSample = currentModelCovDistribution.sample(this.random);
        updatedState.getOnRoadModelCovarianceParam().setValue(stateCovSample);
        updatedState.getOnRoadModelCovarianceParam().setConditionalDistribution(
            new MultivariateGaussian(VectorFactory.getDefault().createVector1D(), 
                stateCovSample));
      } else {
        updatedState.getOffRoadModelCovarianceParam().setParameterPrior(currentModelCovDistribution);
        final Matrix stateCovSample = currentModelCovDistribution.sample(this.random);
        updatedState.getOffRoadModelCovarianceParam().setValue(stateCovSample);
        updatedState.getOffRoadModelCovarianceParam().setConditionalDistribution(
            new MultivariateGaussian(VectorFactory.getDefault().createVector1D(), 
                stateCovSample));
      }
      
      Vector newObsStateSample = MotionStateEstimatorPredictor.getOg().times(
        modelCovarianceEstimator.getNewPosteriorStateSample().getGroundDistribution().getMean());
      RoadMeasurementCovarianceEstimatorPredictor measurementCovarianceEstimator = new RoadMeasurementCovarianceEstimatorPredictor(
          updatedState, newObsStateSample);
      final InverseWishartDistribution currentObsCovDistribution = updatedState.getObservationCovarianceParam()
          .getParameterPrior().clone();
      measurementCovarianceEstimator.update(currentObsCovDistribution, obs.getProjectedPoint());
      
      updatedState.getObservationCovarianceParam().setParameterPrior(currentObsCovDistribution);
      final Matrix obsCovSample = currentObsCovDistribution.sample(this.random);
      updatedState.getObservationCovarianceParam().setValue(obsCovSample);
      updatedState.getObservationCovarianceParam().setConditionalDistribution(
          new MultivariateGaussian(VectorFactory.getDefault().createVector1D(), 
              obsCovSample));
    }
    
    return updatedState;
  }

}