package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;

public class OnOffEdgeTransProbabilityFunction extends
    AbstractCloneableSerializable implements
    ProbabilityMassFunction<InferenceGraphEdge> {

  /**
   * 
   */
  private static final long serialVersionUID = 824339714203059115L;
  protected VehicleState<? extends GpsObservation> currentState;
  protected OnOffEdgeTransDistribution distribution;

  public OnOffEdgeTransProbabilityFunction(
    OnOffEdgeTransDistribution onOffEdgeTransDistribution) {
    this.distribution = onOffEdgeTransDistribution;
  }

  @Override
  public Double evaluate(InferenceGraphEdge input) {
    return Math.exp(this.logEvaluate(input));
  }

  @Override
  public Set<? extends InferenceGraphEdge> getDomain() {
    return this.distribution.getDomain();
  }

  @Override
  public int getDomainSize() {
    return this.distribution.getDomainSize();
  }

  @Override
  public double getEntropy() {
    return 0;
  }

  @Override
  public ProbabilityMassFunction<InferenceGraphEdge>
      getProbabilityFunction() {
    return new OnOffEdgeTransProbabilityFunction(this.distribution);
  }

  @Override
  public double logEvaluate(InferenceGraphEdge to) {

    if (this.currentState == null) {

      final double totalProb =
          Math.log(this.distribution.getFreeMotionTransProbs()
              .getParameters().sum()
              + this.distribution.getEdgeMotionTransProbs()
                  .getParameters().sum());
      if (to.isNullEdge()) {
        return LogMath
            .add(
                this.distribution
                    .getFreeMotionTransProbs()
                    .getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOff),
                this.distribution
                    .getEdgeMotionTransProbs()
                    .getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOff))
            - totalProb;
      } else {
        return LogMath
            .add(
                this.distribution
                    .getFreeMotionTransProbs()
                    .getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOn),
                this.distribution
                    .getEdgeMotionTransProbs()
                    .getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOn))
            - totalProb;
      }
    } else {
      if (this.distribution.currentEdge.isNullEdge()) {
        return this.distribution
            .getFreeMotionTransProbs()
            .getProbabilityFunction()
            .logEvaluate(
                OnOffEdgeTransDistribution.getTransitionType(
                    this.distribution.currentEdge, to));
      } else {
        return this.distribution
            .getEdgeMotionTransProbs()
            .getProbabilityFunction()
            .logEvaluate(
                OnOffEdgeTransDistribution.getTransitionType(
                    this.distribution.currentEdge, to));
      }
    }
  }

  @Override
  public InferenceGraphEdge sample(Random random) {
    return this.distribution.sample(random);
  }

  @Override
  public ArrayList<? extends InferenceGraphEdge> sample(
    Random random, int numSamples) {
    return this.distribution.sample(random, numSamples);
  }

}
