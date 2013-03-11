package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.ArrayList;
import java.util.Random;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.util.model.TransitionProbMatrix;

import com.google.common.collect.Lists;

/**
 * Class representing the transition from one edge to another. For now we use
 * three transition types: 1. off-road to off-road/on-road to on-road 2.
 * off-road to on-road 3. on-road to off-road
 * 
 * @author bwillard
 * 
 */
public class OnOffEdgeTransPriorDistribution extends
    AbstractCloneableSerializable implements
    ComputableDistribution<TransitionProbMatrix>,
    Comparable<OnOffEdgeTransPriorDistribution> {

  public static class OnOffEdgeTransPriorProbabilityFunction extends
      OnOffEdgeTransPriorDistribution implements
      ProbabilityFunction<TransitionProbMatrix> {

    /**
     * 
     */
    private static final long serialVersionUID =
        -1089109482751467343L;

    public OnOffEdgeTransPriorProbabilityFunction(
      OnOffEdgeTransPriorDistribution onOffEdgeTransPriorDistribution) {
      super(onOffEdgeTransPriorDistribution);
    }

    @Override
    public Double evaluate(TransitionProbMatrix input) {
      return Math.exp(this.logEvaluate(input));
    }

    @Override
    public OnOffEdgeTransPriorProbabilityFunction
        getProbabilityFunction() {
      return null;
    }

    @Override
    public double logEvaluate(TransitionProbMatrix input) {
      final double edgeMotionLogProb =
          this.edgeMotionTransProbPrior.getProbabilityFunction()
              .logEvaluate(input.getEdgeMotionTransProbs());
      final double freeMotionLogProb =
          this.freeMotionTransProbPrior.getProbabilityFunction()
              .logEvaluate(input.getFreeMotionTransProbs());
      return LogMath.add(edgeMotionLogProb, freeMotionLogProb);
    }

  }

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  protected DirichletDistribution edgeMotionTransProbPrior;
  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  protected DirichletDistribution freeMotionTransProbPrior;

  public OnOffEdgeTransPriorDistribution(
    OnOffEdgeTransPriorDistribution onOffEdgeTransPriorDistribution) {
    this.edgeMotionTransProbPrior =
        onOffEdgeTransPriorDistribution.edgeMotionTransProbPrior;
    this.freeMotionTransProbPrior =
        onOffEdgeTransPriorDistribution.freeMotionTransProbPrior;
  }

  /**
   * Initialize prior from the given hyper prior parameters, using the hyper
   * prior means (i.e. the given parameters).
   * 
   * @param graph
   * @param edgeMotionPriorParams
   * @param freeMotionPriorParams
   * @param rng
   */
  public OnOffEdgeTransPriorDistribution(
    Vector edgeMotionPriorParams, Vector freeMotionPriorParams) {
    this.setFreeMotionTransProbPrior(new DirichletDistribution(
        freeMotionPriorParams));
    this.setEdgeMotionTransProbPrior(new DirichletDistribution(
        edgeMotionPriorParams));
  }

  @Override
  public OnOffEdgeTransPriorDistribution clone() {
    final OnOffEdgeTransPriorDistribution transDist =
        (OnOffEdgeTransPriorDistribution) super.clone();
    transDist.setEdgeMotionTransProbPrior(this
        .getEdgeMotionTransProbPrior().clone());
    transDist.setFreeMotionTransProbPrior(this
        .getFreeMotionTransProbPrior().clone());
    return transDist;
  }

  @Override
  public int compareTo(OnOffEdgeTransPriorDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.getEdgeMotionTransProbPrior()
        .getParameters().toArray(), o.getEdgeMotionTransProbPrior()
        .getParameters().toArray());
    comparator.append(this.getFreeMotionTransProbPrior()
        .getParameters().toArray(), o.getFreeMotionTransProbPrior()
        .getParameters().toArray());
    comparator.append(this.getFreeMotionTransProbPrior()
        .getParameters().toArray(), o.getFreeMotionTransProbPrior()
        .getParameters().toArray());

    return comparator.toComparison();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final OnOffEdgeTransPriorDistribution other =
        (OnOffEdgeTransPriorDistribution) obj;
    if (this.getEdgeMotionTransProbPrior() == null) {
      if (other.getEdgeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!this.getEdgeMotionTransProbPrior().getParameters()
        .equals(other.getEdgeMotionTransProbPrior().getParameters())) {
      return false;
    }
    if (this.getFreeMotionTransProbPrior() == null) {
      if (other.getFreeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!this.getFreeMotionTransProbPrior().getParameters()
        .equals(other.getFreeMotionTransProbPrior().getParameters())) {
      return false;
    }
    return true;
  }

  public DirichletDistribution getEdgeMotionTransProbPrior() {
    return this.edgeMotionTransProbPrior;
  }

  public DirichletDistribution getFreeMotionTransProbPrior() {
    return this.freeMotionTransProbPrior;
  }

  @Override
  public OnOffEdgeTransPriorProbabilityFunction
      getProbabilityFunction() {
    return new OnOffEdgeTransPriorProbabilityFunction(this);
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.getEdgeMotionTransProbPrior() == null) ? 0
                : this.getEdgeMotionTransProbPrior().getParameters()
                    .hashCode());
    result =
        prime
            * result
            + ((this.getFreeMotionTransProbPrior() == null) ? 0
                : this.getFreeMotionTransProbPrior().getParameters()
                    .hashCode());
    return result;
  }

  @Override
  public TransitionProbMatrix sample(Random random) {

    final Vector freeMotionProbs =
        this.freeMotionTransProbPrior.sample(random);
    final Vector edgeMotionProbs =
        this.edgeMotionTransProbPrior.sample(random);

    return new TransitionProbMatrix(edgeMotionProbs, freeMotionProbs);
  }

  @Override
  public ArrayList<TransitionProbMatrix> sample(Random random,
    int numSamples) {
    final ArrayList<TransitionProbMatrix> samples =
        Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(this.sample(random));
    }
    return samples;
  }

  public void setEdgeMotionTransProbPrior(
    DirichletDistribution edgeMotionTransProbPrior) {
    this.edgeMotionTransProbPrior = edgeMotionTransProbPrior;
  }

  public void setFreeMotionTransProbPrior(
    DirichletDistribution freeMotionTransProbPrior) {
    this.freeMotionTransProbPrior = freeMotionTransProbPrior;
  }

  @Override
  public String toString() {
    return "EdgeTransitionDistributions [freeMotionTransPrior="
        + this.getFreeMotionTransProbPrior().getParameters()
        + ", edgeMotionTransPrior="
        + this.getEdgeMotionTransProbPrior().getParameters() + "]";
  }

}
