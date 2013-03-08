package org.opentrackingtools.estimators;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;

import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;

public class OnOffEdgeTransitionEstimatorPredictor
    extends AbstractCloneableSerializable implements RecursiveBayesianEstimatorPredictor<InferenceGraphEdge, OnOffEdgeTransDistribution> {

  MultinomialBayesianEstimator estimator;

  protected BayesianParameter<InferenceGraphEdge, OnOffEdgeTransDistribution, OnOffEdgeTransDistribution> fromParameter;

  private VehicleState<? extends GpsObservation> currentState;

  public <V extends VehicleState<? extends GpsObservation>> OnOffEdgeTransitionEstimatorPredictor(
    V currentState) {
    MultinomialDistribution conditionalDist;
    DirichletDistribution priorDist;
    
    if (currentState.getPathStateParam().getValue().getDimensionality() == 2) {
      conditionalDist =
          currentState.getEdgeTransitionParam()
              .getParameterPrior()
              .getEdgeMotionTransProb();
      priorDist =
          currentState.getEdgeTransitionParam()
              .getParameterPrior()
              .getEdgeMotionTransProbPrior();
    } else {
      conditionalDist =
          currentState.getEdgeTransitionParam()
              .getParameterPrior()
              .getFreeMotionTransProb();
      priorDist =
          currentState.getEdgeTransitionParam()
              .getParameterPrior()
              .getFreeMotionTransProbPrior();
    }
    this.estimator =
        new MultinomialBayesianEstimator(conditionalDist, priorDist);
    this.currentState = currentState;
  }

  @Override
  public OnOffEdgeTransDistribution learn(
    Collection<? extends InferenceGraphEdge> data) {
    return null;
  }

  /**
   * @see OnOffEdgeTransitionEstimatorPredictor#update(OnOffEdgeTransDistribution,
   *      InferenceGraphEdge)
   */
  @Override
  public void update(OnOffEdgeTransDistribution target,
    Iterable<? extends InferenceGraphEdge> to) {
    for (InferenceGraphEdge toEdge : to) {
      update(target, toEdge);
    }
  }

  /**
   * This method updates the target with the conditional assumptions within
   * this estimator.
   */
  @Override
  public void
      update(OnOffEdgeTransDistribution prior, InferenceGraphEdge toEdge) {
    final Vector transType =
        OnOffEdgeTransDistribution.getTransitionType(fromParameter.getValue(), toEdge);
    if (fromParameter.getValue().isNullEdge()) {
      estimator.update(prior.getFreeMotionTransProbPrior(),
          transType);
      prior.getFreeMotionTransProb().setParameters(
          prior.getFreeMotionTransProbPrior().getMean());
    } else {
      estimator.update(prior.getEdgeMotionTransProbPrior(),
          transType);
      prior.getEdgeMotionTransProb().setParameters(
          prior.getEdgeMotionTransProbPrior().getMean());
    }
  }

  @Override
  public OnOffEdgeTransDistribution
      createPredictiveDistribution(
        OnOffEdgeTransDistribution posterior) {
    return posterior;
    //      if (fromParameter.getValue().isNullEdge()) {
    //        estimator.createPredictiveDistribution(posterior.freeMotionTransProbPrior);
    //      } else {
    //        estimator.createPredictiveDistribution(posterior.edgeMotionTransProbPrior);
    //      }
  }

  @Override
  public OnOffEdgeTransDistribution createInitialLearnedObject() {
    return new OnOffEdgeTransDistribution(this.currentState.getGraph(), null, 
        estimator.createInitialLearnedObject().getMean(), estimator.createInitialLearnedObject().getMean());
  }
}