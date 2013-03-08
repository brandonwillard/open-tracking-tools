package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ClosedFormComputableDiscreteDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.DiscreteDistribution;
import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.ProbabilityMassFunction;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.model.GpsObservation;
import org.opentrackingtools.model.VehicleState;
import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.model.TransitionProbMatrix;
import org.testng.collections.Sets;

import com.google.common.base.Preconditions;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterators;
import com.google.common.collect.Lists;

/**
 * Class representing the transition from one edge to another. For now we use
 * three transition types: 1. off-road to off-road/on-road to on-road 2.
 * off-road to on-road 3. on-road to off-road
 * 
 * @author bwillard
 * 
 */
public class OnOffEdgeTransPriorDistribution
    extends AbstractCloneableSerializable implements
    ComputableDistribution<TransitionProbMatrix>, Comparable<OnOffEdgeTransPriorDistribution> {

  public static class OnOffEdgeTransPriorProbabilityFunction extends OnOffEdgeTransPriorDistribution 
    implements ProbabilityFunction<TransitionProbMatrix> {

    public OnOffEdgeTransPriorProbabilityFunction(
      OnOffEdgeTransPriorDistribution onOffEdgeTransPriorDistribution) {
      super(onOffEdgeTransPriorDistribution);
    }

    @Override
    public OnOffEdgeTransPriorProbabilityFunction
        getProbabilityFunction() {
      return null;
    }

    @Override
    public Double evaluate(TransitionProbMatrix input) {
      return Math.exp(logEvaluate(input));
    }

    @Override
    public double logEvaluate(TransitionProbMatrix input) {
      final double edgeMotionLogProb = this.edgeMotionTransProbPrior.getProbabilityFunction().logEvaluate(
          input.getEdgeMotionTransProbs());
      final double freeMotionLogProb = this.freeMotionTransProbPrior.getProbabilityFunction().logEvaluate(
          input.getFreeMotionTransProbs());
      return LogMath.add(edgeMotionLogProb, freeMotionLogProb);
    }

  }

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  protected DirichletDistribution freeMotionTransProbPrior;

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  protected DirichletDistribution edgeMotionTransProbPrior;
  
  protected InferenceGraph graph;
  protected InferenceGraphEdge currentEdge;
  protected VehicleState<? extends GpsObservation> currentState;
  
  public OnOffEdgeTransPriorDistribution(
    OnOffEdgeTransPriorDistribution onOffEdgeTransPriorDistribution) {
    this.currentEdge = onOffEdgeTransPriorDistribution.currentEdge;
    this.currentState = onOffEdgeTransPriorDistribution.currentState;
    this.edgeMotionTransProbPrior = onOffEdgeTransPriorDistribution.edgeMotionTransProbPrior;
    this.freeMotionTransProbPrior = onOffEdgeTransPriorDistribution.freeMotionTransProbPrior;
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
  public OnOffEdgeTransPriorDistribution(VehicleState<?> currentState, 
    InferenceGraphEdge currentEdge, Vector edgeMotionPriorParams, Vector freeMotionPriorParams) {
    
    this.currentState = currentState;
    this.graph = currentState.getGraph();
    this.currentEdge = currentEdge;
    
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
    transDist.currentEdge = ObjectUtil.cloneSmart(this.currentEdge);
    transDist.graph = this.graph;
    return transDist;
  }

  @Override
  public int compareTo(OnOffEdgeTransPriorDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.getEdgeMotionTransProbPrior().getParameters().toArray(),
        o.getEdgeMotionTransProbPrior().getParameters().toArray());
    comparator.append(this.getFreeMotionTransProbPrior().getParameters().toArray(),
        o.getFreeMotionTransProbPrior().getParameters().toArray());
    comparator.append(this.getFreeMotionTransProbPrior().getParameters().toArray(),
        o.getFreeMotionTransProbPrior().getParameters().toArray());
    comparator.append(this.currentEdge, o.currentEdge);

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
    if (getClass() != obj.getClass()) {
      return false;
    }
    final OnOffEdgeTransPriorDistribution other =
        (OnOffEdgeTransPriorDistribution) obj;
    if (getEdgeMotionTransProbPrior() == null) {
      if (other.getEdgeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!getEdgeMotionTransProbPrior().getParameters().equals(other
            .getEdgeMotionTransProbPrior().getParameters())) {
      return false;
    }
    if (getFreeMotionTransProbPrior() == null) {
      if (other.getFreeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!getFreeMotionTransProbPrior().getParameters().equals(other
            .getFreeMotionTransProbPrior().getParameters())) {
      return false;
    }
    if (this.currentEdge == null) {
      if (other.currentEdge != null) {
        return false;
      }
    } else if (!this.currentEdge.equals(other.currentEdge)) {
      return false;
    }
    return true;
  }

  public DirichletDistribution getEdgeMotionTransProbPrior() {
    return edgeMotionTransProbPrior;
  }

  public DirichletDistribution getFreeMotionTransProbPrior() {
    return freeMotionTransProbPrior;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((getEdgeMotionTransProbPrior() == null) ? 0
                : getEdgeMotionTransProbPrior()
                        .getParameters().hashCode());
    result =
        prime
            * result
            + ((getFreeMotionTransProbPrior() == null) ? 0
                : getFreeMotionTransProbPrior()
                        .getParameters().hashCode());
    result =
        prime
            * result
            + ((this.currentEdge == null) ? 0
                : this.currentEdge.hashCode());
    return result;
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
        + getFreeMotionTransProbPrior().getParameters()
        + ", edgeMotionTransPrior="
        + getEdgeMotionTransProbPrior().getParameters() + "]";
  }

  @Override
  public TransitionProbMatrix sample(Random random) {
    
    Vector freeMotionProbs = this.freeMotionTransProbPrior.sample(random);
    Vector edgeMotionProbs = this.edgeMotionTransProbPrior.sample(random);
    
    return new TransitionProbMatrix(edgeMotionProbs, freeMotionProbs);
  }

  @Override
  public ArrayList<TransitionProbMatrix> sample(Random random, int numSamples) {
    ArrayList<TransitionProbMatrix> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(sample(random));
    }
    return samples;
  }

  @Override
  public OnOffEdgeTransPriorProbabilityFunction getProbabilityFunction() {
    return new OnOffEdgeTransPriorProbabilityFunction(this);
  }
  
}
