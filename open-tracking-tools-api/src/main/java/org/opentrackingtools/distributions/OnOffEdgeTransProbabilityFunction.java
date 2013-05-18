package org.opentrackingtools.distributions;

import com.vividsolutions.jts.algorithm.Angle;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineSegment;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.CloneableSerializable;

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
  
  protected InferenceGraphEdge fromEdge;
  protected OnOffEdgeTransDistribution distribution;

  public OnOffEdgeTransProbabilityFunction(
    OnOffEdgeTransDistribution onOffEdgeTransDistribution, InferenceGraphEdge fromEdge) {
    this.distribution = onOffEdgeTransDistribution;
    this.fromEdge = fromEdge;
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
    return new OnOffEdgeTransProbabilityFunction(this.distribution, this.fromEdge);
  }

  @Override
  public double logEvaluate(InferenceGraphEdge to) {

    if (this.fromEdge == null) {

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

      if (this.fromEdge.isNullEdge()) {
        return this.distribution
            .getFreeMotionTransProbs()
            .getProbabilityFunction()
            .logEvaluate(
                OnOffEdgeTransDistribution.getTransitionType(
                    this.fromEdge, to));
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
        if (!this.fromEdge.equals(to)
            && !this.fromEdge.isNullEdge() && !to.isNullEdge()) {
          final double generalTransProb = this.distribution
                .getEdgeMotionTransProbs()
                .getProbabilityFunction()
                .logEvaluate(
                    OnOffEdgeTransDistribution.getTransitionType(
                        this.fromEdge, to));
          if ((this.fromEdge instanceof InferenceGraphSegment)
              && (to instanceof InferenceGraphSegment)) {
            InferenceGraphSegment fromSegment = (InferenceGraphSegment) this.fromEdge;
            InferenceGraphSegment toSegment = (InferenceGraphSegment) to;
            final Coordinate toEndOnFrom = fromSegment.getLine().project(toSegment.getLine().p1);
            final double localLogUTurnProb;
            if (!this.fromEdge.getSegments().contains(toSegment)
                && toEndOnFrom.distance(toSegment.getLine().p1) < 7d) {
              localLogUTurnProb = logUTurnProbability;
            } else {
              localLogUTurnProb = logNoUTurnProbability;
            }
            
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
                      this.fromEdge, to));
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

  @Override
  public OnOffEdgeTransProbabilityFunction clone() {
    OnOffEdgeTransProbabilityFunction clone = (OnOffEdgeTransProbabilityFunction) super.clone();
    clone.distribution = this.distribution;
    clone.fromEdge = this.fromEdge;
    return clone;
  }

  public InferenceGraphEdge getFromEdge() {
    return fromEdge;
  }

  public void setFromEdge(InferenceGraphEdge fromEdge) {
    this.fromEdge = fromEdge;
  }

}
