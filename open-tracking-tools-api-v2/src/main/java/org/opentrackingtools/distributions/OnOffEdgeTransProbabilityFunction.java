package org.opentrackingtools.distributions;

import java.util.ArrayList;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ProbabilityDensityFunction;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

public class OnOffEdgeTransProbabilityFunction 
  extends AbstractCloneableSerializable implements ProbabilityMassFunction<InferenceGraphEdge> {

  protected VehicleState<? extends GpsObservation> currentState;
  protected OnOffEdgeTransDistribution distribution;
  
  public OnOffEdgeTransProbabilityFunction(
    OnOffEdgeTransDistribution onOffEdgeTransDistribution) {
    this.distribution = onOffEdgeTransDistribution;
  }

  @Override
  public double logEvaluate(InferenceGraphEdge to) {

    if (currentState == null) {

      final double totalProb =
          Math.log(distribution.getFreeMotionTransProb().getParameters().sum()
              + distribution.getEdgeMotionTransProb().getParameters().sum());
      if (to.isNullEdge()) {
        return LogMath
            .add(
                distribution.getFreeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOff),
                distribution.getEdgeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOff))
            - totalProb;
      } else {
        return LogMath
            .add(
                distribution.getFreeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOffToOn),
                distribution.getEdgeMotionTransProb().getProbabilityFunction()
                    .logEvaluate(
                        OnOffEdgeTransDistribution.stateOnToOn))
            - totalProb;
      }
    } else {
      if (distribution.currentEdge.isNullEdge()) {
        return distribution.getFreeMotionTransProb().getProbabilityFunction()
            .logEvaluate(OnOffEdgeTransDistribution.getTransitionType(distribution.currentEdge, to));
      } else {
        return distribution.getEdgeMotionTransProb().getProbabilityFunction()
            .logEvaluate(OnOffEdgeTransDistribution.getTransitionType(distribution.currentEdge, to));
      }
    }
  }

  @Override
  public Double evaluate(InferenceGraphEdge input) {
    return Math.exp(this.logEvaluate(input));
  }
  
  @Override
  public InferenceGraphEdge sample(Random random) {
    return this.distribution.sample(random);
  }

  @Override
  public ArrayList<? extends InferenceGraphEdge> sample(Random random, int numSamples) {
    return this.distribution.sample(random, numSamples);
  }

  @Override
  public ProbabilityMassFunction<InferenceGraphEdge>
      getProbabilityFunction() {
    return new OnOffEdgeTransProbabilityFunction(distribution);
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

}
