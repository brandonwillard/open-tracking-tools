package org.opentrackingtools.estimators;

import gov.sandia.cognition.learning.algorithm.IncrementalLearner;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.bayesian.BayesianEstimatorPredictor;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Collection;
import java.util.Random;

import org.opentrackingtools.distributions.OnOffEdgeTransDistribution;
import org.opentrackingtools.distributions.OnOffEdgeTransPriorDistribution;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.util.model.TransitionProbMatrix;

public class OnOffEdgeTransitionEstimatorPredictor extends
    AbstractCloneableSerializable
    implements
    BayesianEstimatorPredictor<InferenceGraphEdge, TransitionProbMatrix, OnOffEdgeTransPriorDistribution>,
    IncrementalLearner<InferenceGraphEdge, OnOffEdgeTransPriorDistribution> {

  public static class Parameter extends AbstractCloneableSerializable
      implements
      BayesianParameter<TransitionProbMatrix, OnOffEdgeTransDistribution, OnOffEdgeTransPriorDistribution> {

    private static final long serialVersionUID =
        -1188148823031520367L;
    protected OnOffEdgeTransDistribution conditional;
    protected OnOffEdgeTransPriorDistribution prior;
    protected TransitionProbMatrix value = null;

    public Parameter(OnOffEdgeTransDistribution conditional,
      OnOffEdgeTransPriorDistribution prior) {
      this.conditional = conditional;
      this.prior = prior;
    }

    @Override
    public OnOffEdgeTransDistribution getConditionalDistribution() {
      return this.conditional;
    }

    @Override
    public String getName() {
      return "transitionProbMatrix";
    }

    @Override
    public OnOffEdgeTransPriorDistribution getParameterPrior() {
      return this.prior;
    }

    @Override
    public TransitionProbMatrix getValue() {
      return this.value;
    }

    @Override
    public void setValue(TransitionProbMatrix value) {
      this.value = value;
    }

    @Override
    public void updateConditionalDistribution(Random random) {
      final TransitionProbMatrix transMatrix =
          this.prior.sample(random);
      this.conditional.setEdgeMotionTransProbs(transMatrix
          .getEdgeMotionTransProbs());
      this.conditional.setEdgeMotionTransProbs(transMatrix
          .getFreeMotionTransProbs());

    }

  }

  private static final long serialVersionUID = -5770425044080860091L;

  protected InferenceGraphEdge currentEdge;

  protected VehicleState<?> currentState;

  MultinomialBayesianEstimator estimator;
  protected BayesianParameter<InferenceGraphEdge, OnOffEdgeTransDistribution, OnOffEdgeTransDistribution> fromParameter;

  public OnOffEdgeTransitionEstimatorPredictor(
    VehicleState<?> currentState, InferenceGraphEdge currentEdge) {

    this.currentState = currentState;
    this.currentEdge = currentEdge;

    MultinomialDistribution conditionalDist;
    DirichletDistribution priorDist;

    if (!currentEdge.isNullEdge()) {
      conditionalDist =
          currentState.getEdgeTransitionParam()
              .getConditionalDistribution().getEdgeMotionTransProbs();
      priorDist =
          currentState.getEdgeTransitionParam().getParameterPrior()
              .getEdgeMotionTransProbPrior();
    } else {
      conditionalDist =
          currentState.getEdgeTransitionParam()
              .getConditionalDistribution().getFreeMotionTransProbs();
      priorDist =
          currentState.getEdgeTransitionParam().getParameterPrior()
              .getFreeMotionTransProbPrior();
    }
    this.estimator =
        new MultinomialBayesianEstimator(conditionalDist, priorDist);
  }

  @Override
  public OnOffEdgeTransPriorDistribution createInitialLearnedObject() {
    return new OnOffEdgeTransPriorDistribution(this.currentState,
        this.currentEdge, this.estimator.createInitialLearnedObject()
            .getMean(), this.estimator.createInitialLearnedObject()
            .getMean());
  }

  @Override
  public OnOffEdgeTransDistribution createPredictiveDistribution(
    OnOffEdgeTransPriorDistribution posterior) {
    return null;
    //      if (fromParameter.getValue().isNullEdge()) {
    //        estimator.createPredictiveDistribution(posterior.freeMotionTransProbPrior);
    //      } else {
    //        estimator.createPredictiveDistribution(posterior.edgeMotionTransProbPrior);
    //      }
  }

  @Override
  public OnOffEdgeTransPriorDistribution learn(
    Collection<? extends InferenceGraphEdge> data) {
    // TODO Auto-generated method stub
    return null;
  }

  /**
   * This method updates the target with the conditional assumptions within this
   * estimator.
   */
  @Override
  public void update(OnOffEdgeTransPriorDistribution prior,
    InferenceGraphEdge toEdge) {
    final Vector transType =
        OnOffEdgeTransDistribution.getTransitionType(
            this.fromParameter.getValue(), toEdge);
    if (this.currentEdge.isNullEdge()) {
      this.estimator.update(prior.getFreeMotionTransProbPrior(),
          transType);
    } else {
      this.estimator.update(prior.getEdgeMotionTransProbPrior(),
          transType);
    }
  }

  /**
   * @see OnOffEdgeTransitionEstimatorPredictor#update(OnOffEdgeTransDistribution,
   *      InferenceGraphEdge)
   */
  @Override
  public void update(OnOffEdgeTransPriorDistribution target,
    Iterable<? extends InferenceGraphEdge> to) {
    for (final InferenceGraphEdge toEdge : to) {
      this.update(target, toEdge);
    }
  }
}