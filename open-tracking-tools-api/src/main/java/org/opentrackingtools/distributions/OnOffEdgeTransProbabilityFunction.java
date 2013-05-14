package org.opentrackingtools.distributions;

import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Random;
import java.util.Set;

import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.graph.InferenceGraphSegment;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleStateDistribution;

public class OnOffEdgeTransProbabilityFunction extends
    AbstractCloneableSerializable implements
    ProbabilityMassFunction<InferenceGraphEdge> {

  private static final long serialVersionUID = 824339714203059115L;
  
  private static final double logUTurnProbability = Math.log(0.05d);
  private static final double logNoUTurnProbability = Math.log(1d - 0.05d);
  
  protected VehicleStateDistribution<? extends GpsObservation> currentState;
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

      final InferenceGraphEdge currentEdge =
          this.distribution.currentEdge;
      if (currentEdge.isNullEdge()) {
        return this.distribution
            .getFreeMotionTransProbs()
            .getProbabilityFunction()
            .logEvaluate(
                OnOffEdgeTransDistribution.getTransitionType(
                    currentEdge, to));
      } else {
        /*
         * Since we don't want, nor expect, an
         * infinite momentum situation, we believe it's
         * less likely to have, say, looped around an edge
         * only to end up on the mirrored location (when
         * the observation is still, for example, and we have
         * the right velocity, this is a priori most likely), than
         * to have simply stayed in place.
         */
        if (!currentEdge.equals(to)
            && !currentEdge.isNullEdge() && !to.isNullEdge()) {
          final double generalTransProb = this.distribution
                .getEdgeMotionTransProbs()
                .getProbabilityFunction()
                .logEvaluate(
                    OnOffEdgeTransDistribution.getTransitionType(
                        currentEdge, to));
          if ((currentEdge instanceof InferenceGraphSegment)
              && (to instanceof InferenceGraphSegment)
              ) {
            InferenceGraphSegment fromSegment = (InferenceGraphSegment) currentEdge;
            InferenceGraphSegment toSegment = (InferenceGraphSegment) to;
            final Coordinate toEndOnFrom = fromSegment.getLine().project(toSegment.getLine().p1);
            final double localLogUTurnProb = (toEndOnFrom.distance(toSegment.getLine().p1) < 7d) ? 
               logUTurnProbability : logNoUTurnProbability;
            
            return generalTransProb + localLogUTurnProb;
          } else {
            /*
             * If we're not dealing with segments, then we can't really
             * tell if we're ending up on the oppositely oriented edge,
             * so we sum over the possibilities.
             */
            return generalTransProb;
          }
          
        } else {
          return this.distribution
              .getEdgeMotionTransProbs()
              .getProbabilityFunction()
              .logEvaluate(
                  OnOffEdgeTransDistribution.getTransitionType(
                      currentEdge, to));
        }
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
