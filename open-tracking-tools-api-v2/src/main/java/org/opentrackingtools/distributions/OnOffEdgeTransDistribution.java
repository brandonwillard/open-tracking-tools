package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ClosedFormComputableDiscreteDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ComputableDistribution;
import gov.sandia.cognition.statistics.DiscreteDistribution;
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
public class OnOffEdgeTransDistribution
    extends AbstractCloneableSerializable implements
    DiscreteDistribution<InferenceGraphEdge>, Comparable<OnOffEdgeTransDistribution> {

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  protected MultinomialDistribution freeMotionTransProbs =
      new MultinomialDistribution(2, 1);

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  protected MultinomialDistribution edgeMotionTransProbs =
      new MultinomialDistribution(2, 1);
  
  protected InferenceGraph graph;
  protected InferenceGraphEdge currentEdge;
  protected VehicleState<?> currentState;

  static final Vector stateOffToOff = VectorFactory
      .getDefault().copyValues(1d, 0d);
  static final Vector stateOffToOn = VectorFactory
      .getDefault().copyValues(0d, 1d);

  static final Vector stateOnToOn = VectorFactory
      .getDefault().copyValues(1d, 0d);
  static final Vector stateOnToOff = VectorFactory
      .getDefault().copyValues(0d, 1d);

  static final Double zeroTolerance = 1e-6;

  public OnOffEdgeTransDistribution(
    VehicleState<?> currentState, InferenceGraphEdge currentEdge,
    Vector edgeMotionProbs, Vector freeMotionProbs) {
    
    this.currentState = currentState;
    this.graph = currentState.getGraph();
    this.currentEdge = currentEdge;
    
    /*
     * Start by setting the means.
     */
    getFreeMotionTransProbs().setParameters(freeMotionProbs);
    getEdgeMotionTransProbs().setParameters(edgeMotionProbs);
  }

  public OnOffEdgeTransDistribution(
    OnOffEdgeTransDistribution onOffEdgeTransProbsDistribution) {
    this.currentEdge = onOffEdgeTransProbsDistribution.currentEdge;
    this.currentState = onOffEdgeTransProbsDistribution.currentState;
    this.edgeMotionTransProbs = onOffEdgeTransProbsDistribution.edgeMotionTransProbs;
    this.freeMotionTransProbs = onOffEdgeTransProbsDistribution.freeMotionTransProbs;
  }

  @Override
  public OnOffEdgeTransDistribution clone() {
    final OnOffEdgeTransDistribution transDist =
        (OnOffEdgeTransDistribution) super.clone();
    transDist.edgeMotionTransProbs = this.getEdgeMotionTransProbs().clone();
    transDist.freeMotionTransProbs = this.getFreeMotionTransProbs().clone();
    transDist.currentEdge = ObjectUtil.cloneSmart(this.currentEdge);
    transDist.graph = this.graph;
    return transDist;
  }

  @Override
  public int compareTo(OnOffEdgeTransDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.getEdgeMotionTransProbs().getParameters().toArray(), 
        o.getEdgeMotionTransProbs().getParameters().toArray());
    comparator.append(this.getFreeMotionTransProbs().getParameters().toArray(), 
        o.getFreeMotionTransProbs().getParameters().toArray());
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
    final OnOffEdgeTransDistribution other =
        (OnOffEdgeTransDistribution) obj;
    if (getEdgeMotionTransProbs() == null) {
      if (other.getEdgeMotionTransProbs() != null) {
        return false;
      }
    } else if (!getEdgeMotionTransProbs()
        .getParameters().equals(other.getEdgeMotionTransProbs()
        .getParameters())) {
      return false;
    }
    if (getFreeMotionTransProbs() == null) {
      if (other.getFreeMotionTransProbs() != null) {
        return false;
      }
    } else if (!getFreeMotionTransProbs()
        .getParameters().equals(other.getFreeMotionTransProbs()
        .getParameters())) {
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


  public MultinomialDistribution getEdgeMotionTransProbs() {
    return edgeMotionTransProbs;
  }

  public MultinomialDistribution getFreeMotionTransProbs() {
    return freeMotionTransProbs;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((getEdgeMotionTransProbs() == null) ? 0
                : getEdgeMotionTransProbs()
                        .getParameters().hashCode());
    result =
        prime
            * result
            + ((getFreeMotionTransProbs() == null) ? 0
                : getFreeMotionTransProbs()
                        .getParameters().hashCode());
    result =
        prime
            * result
            + ((this.currentEdge == null) ? 0
                : this.currentEdge.hashCode());
    return result;
  }

  /**
   * This method restricts sampling when probabilities have collapsed on one
   * value in the domain. It allows us to emulate a deterministic distribution
   * and eliminate on- or off-roading. Also, it prevents Foundry's gamma sampler
   * from taking forever.
   * 
   * @param dist
   * @param rng
   * @return
   */
  public static Vector checkedSample(
    final ClosedFormComputableDistribution<Vector> dist, Random rng) {

    if (dist instanceof ClosedFormComputableDiscreteDistribution) {
      ClosedFormComputableDiscreteDistribution<Vector> distDiscrete =
          (ClosedFormComputableDiscreteDistribution<Vector>) dist;
      Vector one =
          Iterators.find(distDiscrete.getDomain().iterator(),
              new Predicate<Vector>() {
                @Override
                public boolean apply(Vector input) {
                  return Math.abs(1 - dist.getProbabilityFunction()
                      .evaluate(input)) <= zeroTolerance;
                }
              }, null);

      return one == null ? dist.sample(rng) : one;

    } else if (dist instanceof DirichletDistribution) {
      for (int i = 0; i < dist.getMean().getDimensionality(); i++) {
        final double prob = dist.getMean().getElement(i);
        if (Double.isNaN(prob) || Math.abs(1 - prob) <= zeroTolerance) {
          Vector result =
              VectorFactory.getDefault().createVector(
                  dist.getMean().getDimensionality());
          result.setElement(i, 1d);
          return result;
        }
      }
    }

    return dist.sample(rng);
  }

  public void setEdgeMotionTransProbs(
    Vector edgeMotionTransProbs) {
    this.edgeMotionTransProbs.setParameters(edgeMotionTransProbs);
  }

  public void setFreeMotionTransProbs(
    Vector freeMotionTransProbs) {
    this.freeMotionTransProbs.setParameters(freeMotionTransProbs);
  }

  @Override
  public String toString() {
    return "EdgeTransitionDistributions [freeMotionTransProbs="
        + getFreeMotionTransProbs().getParameters()
        + ", edgeMotionTransProbs="
        + getEdgeMotionTransProbs().getParameters() + "]";
  }

  public static Vector getStateOffToOff() {
    return stateOffToOff;
  }

  public static Vector getStateOffToOn() {
    return stateOffToOn;
  }

  public static Vector getStateOnToOff() {
    return stateOnToOff;
  }

  public static Vector getStateOnToOn() {
    return stateOnToOn;
  }

  public static Vector getTransitionType(InferenceGraphEdge from,
    InferenceGraphEdge to) {
    if (from.isNullEdge()) {
      if (to.isNullEdge()) {
        return stateOffToOff;
      } else {
        return stateOffToOn;
      }
    } else {
      if (!to.isNullEdge()) {
        return stateOnToOn;
      } else {
        return stateOnToOff;
      }
    }
  }

  @Override
  public InferenceGraphEdge sample(Random random) {
    
    List<InferenceGraphEdge> domain = Lists.newArrayList(this.getDomain());

    if (this.currentEdge.isNullEdge()) {
      /*
       * We're currently in free-motion. If there are transfer edges, then
       * sample from those.
       */
      if (domain.isEmpty()) {
        return this.graph.getNullInferredEdge();
      } else {
        final Vector sample =
            checkedSample(this.getFreeMotionTransProbs(), random);

        if (sample.equals(stateOffToOn)) {
          domain.remove(this.graph.getNullInferredEdge());
          return domain.get(random.nextInt(domain.size()));
        } else {
          return this.graph.getNullInferredEdge();
        }
      }
    } else {
      /*
       * We're on an edge, so sample whether we go off-road, or transfer/stay on.
       * If the empty edge is contained in the support then we sample for that.
       */
      final Vector sample =
          domain.contains(this.graph.getNullInferredEdge())
              ? checkedSample(this.getEdgeMotionTransProbs(), random)
              : stateOnToOn;

      if (sample.equals(stateOnToOff) || domain.isEmpty()) {
        return this.graph.getNullInferredEdge();
      } else {
        domain.remove(this.graph.getNullInferredEdge());
        return domain.get(random.nextInt(domain.size()));
      }
    }
  }

  @Override
  public ArrayList<InferenceGraphEdge> sample(Random random, int numSamples) {
    ArrayList<InferenceGraphEdge> samples = Lists.newArrayList();
    for (int i = 0; i < numSamples; i++) {
      samples.add(sample(random));
    }
    return samples;
  }

  @Override
  public OnOffEdgeTransProbabilityFunction getProbabilityFunction() {
    return new OnOffEdgeTransProbabilityFunction(this);
  }
  
  protected Set<? extends InferenceGraphEdge> domain = null;
  
  @Override
  public Set<? extends InferenceGraphEdge> getDomain() {
    if (domain == null) {
      domain = Sets.newHashSet(this.graph.getOutgoingTransferableEdges(currentEdge));
    }
    return domain;
  }

  @Override
  public int getDomainSize() {
    return this.getDomain().size();
  }

}
