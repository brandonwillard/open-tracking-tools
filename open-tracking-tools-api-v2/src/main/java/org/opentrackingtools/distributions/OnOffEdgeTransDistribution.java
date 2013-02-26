package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.LogMath;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.math.matrix.mtj.DenseVector;
import gov.sandia.cognition.statistics.ClosedFormComputableDiscreteDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.bayesian.conjugate.MultinomialBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.DirichletDistribution;
import gov.sandia.cognition.statistics.distribution.MultinomialDistribution;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.List;
import java.util.Random;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

import org.apache.commons.lang.builder.CompareToBuilder;
import org.opentrackingtools.edges.InferredEdge;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.util.StatisticsUtil;

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
public class OnOffEdgeTransDistribution extends AbstractCloneableSerializable implements
    Comparable<OnOffEdgeTransDistribution> {

  private static final long serialVersionUID = -8329433263373783485L;

  /*
   * Distribution corresponding to free-movement -> free-movement and
   * free-movement -> edge-movement
   */
  private DirichletDistribution freeMotionTransProbPrior;
  private MultinomialDistribution freeMotionTransPrior = new MultinomialDistribution(2, 1);
  MultinomialBayesianEstimator freeMotionTransEstimator = new MultinomialBayesianEstimator(getFreeMotionTransPrior(),
      getFreeMotionTransProbPrior());

  /*
   * Distribution corresponding to edge-movement -> free-movement and
   * edge-movement -> edge-movement
   */
  private DirichletDistribution edgeMotionTransProbPrior;
  private MultinomialDistribution edgeMotionTransPrior = new MultinomialDistribution(2, 1);
  MultinomialBayesianEstimator edgeMotionTransEstimator = new MultinomialBayesianEstimator(getFreeMotionTransPrior(),
      getFreeMotionTransProbPrior());

  private final InferenceGraph graph;

  private static final Vector stateOffToOff = VectorFactory.getDefault().copyValues(1d, 0d);
  private static final Vector stateOffToOn = VectorFactory.getDefault().copyValues(0d, 1d);

  private static final Vector stateOnToOn = VectorFactory.getDefault().copyValues(1d, 0d);
  private static final Vector stateOnToOff = VectorFactory.getDefault().copyValues(0d, 1d);

  private static final Double zeroTolerance = 1e-6;

  /**
   * Initialize prior from the given hyper prior parameters, using the hyper
   * prior means (i.e. the given parameters).
   * 
   * @param graph
   * @param edgeMotionPriorParams
   * @param freeMotionPriorParams
   * @param rng
   */
  public OnOffEdgeTransDistribution(InferenceGraph graph, Vector edgeMotionPriorParams, Vector freeMotionPriorParams) {
    this.graph = graph;
    this.setFreeMotionTransProbPrior(new DirichletDistribution(freeMotionPriorParams));
    this.setEdgeMotionTransProbPrior(new DirichletDistribution(edgeMotionPriorParams));
    /*
     * Start by setting the means.
     */
    getFreeMotionTransPrior().setParameters(getFreeMotionTransProbPrior().getMean());
    getEdgeMotionTransPrior().setParameters(getEdgeMotionTransProbPrior().getMean());
  }

  public OnOffEdgeTransDistribution(InferenceGraph inferenceGraph, Vector onTransitionProbs, Vector offTransitionProbs,
    Random random) {
    this(inferenceGraph, onTransitionProbs, offTransitionProbs);

    /*
     * Sample an initial prior for the transition probabilities
     */
    final Vector edgePriorParams = OnOffEdgeTransDistribution.checkedSample(this.getEdgeMotionTransProbPrior(), random);
    final Vector freeDriorParams = OnOffEdgeTransDistribution.checkedSample(this.getFreeMotionTransProbPrior(), random);
    this.getEdgeMotionTransPrior().setParameters(edgePriorParams);
    this.getFreeMotionTransPrior().setParameters(freeDriorParams);
  }

  @Override
  public OnOffEdgeTransDistribution clone() {
    final OnOffEdgeTransDistribution transDist = (OnOffEdgeTransDistribution) super.clone();
    transDist.edgeMotionTransEstimator = (MultinomialBayesianEstimator) this.edgeMotionTransEstimator.clone();
    transDist.freeMotionTransEstimator = (MultinomialBayesianEstimator) this.freeMotionTransEstimator.clone();
    transDist.setEdgeMotionTransPrior(this.getEdgeMotionTransPrior().clone());
    transDist.setFreeMotionTransPrior(this.getFreeMotionTransPrior().clone());
    transDist.setEdgeMotionTransProbPrior(this.getEdgeMotionTransProbPrior().clone());
    transDist.setFreeMotionTransProbPrior(this.getFreeMotionTransProbPrior().clone());

    return transDist;
  }

  @Override
  public int compareTo(OnOffEdgeTransDistribution o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(((DenseVector) this.getEdgeMotionTransPrior().getParameters()).getArray(), ((DenseVector) o
        .getEdgeMotionTransPrior().getParameters()).getArray());
    comparator.append(((DenseVector) this.getFreeMotionTransPrior().getParameters()).getArray(), ((DenseVector) o
        .getFreeMotionTransPrior().getParameters()).getArray());

    comparator.append(((DenseVector) this.getEdgeMotionTransProbPrior().getParameters()).getArray(), ((DenseVector) o
        .getEdgeMotionTransProbPrior().getParameters()).getArray());
    comparator.append(((DenseVector) this.getFreeMotionTransProbPrior().getParameters()).getArray(), ((DenseVector) o
        .getFreeMotionTransProbPrior().getParameters()).getArray());

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
    final OnOffEdgeTransDistribution other = (OnOffEdgeTransDistribution) obj;
    if (getEdgeMotionTransPrior() == null) {
      if (other.getEdgeMotionTransPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getEdgeMotionTransPrior().getParameters(), other.getEdgeMotionTransPrior()
        .getParameters())) {
      return false;
    }
    if (getEdgeMotionTransProbPrior() == null) {
      if (other.getEdgeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getEdgeMotionTransProbPrior().getParameters(), other
        .getEdgeMotionTransProbPrior().getParameters())) {
      return false;
    }
    if (getFreeMotionTransPrior() == null) {
      if (other.getFreeMotionTransPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getFreeMotionTransPrior().getParameters(), other.getFreeMotionTransPrior()
        .getParameters())) {
      return false;
    }
    if (getFreeMotionTransProbPrior() == null) {
      if (other.getFreeMotionTransProbPrior() != null) {
        return false;
      }
    } else if (!StatisticsUtil.vectorEquals(getFreeMotionTransProbPrior().getParameters(), other
        .getFreeMotionTransProbPrior().getParameters())) {
      return false;
    }
    return true;
  }

  public double evaluate(InferredEdge from, InferredEdge to) {
    if (from.isNullEdge()) {
      return getFreeMotionTransPrior().getProbabilityFunction().evaluate(getTransitionType(from, to));
    } else {
      return getEdgeMotionTransPrior().getProbabilityFunction().evaluate(getTransitionType(from, to));
    }
  }

  public MultinomialDistribution getEdgeMotionTransPrior() {
    return edgeMotionTransPrior;
  }

  public DirichletDistribution getEdgeMotionTransProbPrior() {
    return edgeMotionTransProbPrior;
  }

  public MultinomialDistribution getFreeMotionTransPrior() {
    return freeMotionTransPrior;
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
            + ((getEdgeMotionTransPrior() == null) ? 0 : StatisticsUtil.hashCodeVector(getEdgeMotionTransPrior()
                .getParameters()));
    result =
        prime
            * result
            + ((getEdgeMotionTransProbPrior() == null) ? 0 : StatisticsUtil
                .hashCodeVector(getEdgeMotionTransProbPrior().getParameters()));
    result =
        prime
            * result
            + ((getFreeMotionTransPrior() == null) ? 0 : StatisticsUtil.hashCodeVector(getFreeMotionTransPrior()
                .getParameters()));
    result =
        prime
            * result
            + ((getFreeMotionTransProbPrior() == null) ? 0 : StatisticsUtil
                .hashCodeVector(getFreeMotionTransProbPrior().getParameters()));
    return result;
  }

  public double logEvaluate(@Nullable InferredEdge from, @Nonnull InferredEdge to) {

    if (from == null) {

      final double totalProb =
          Math.log(getFreeMotionTransPrior().getParameters().sum() + getEdgeMotionTransPrior().getParameters().sum());
      if (to.isNullEdge()) {
        return LogMath.add(
            getFreeMotionTransPrior().getProbabilityFunction().logEvaluate(OnOffEdgeTransDistribution.stateOffToOff),
            getEdgeMotionTransPrior().getProbabilityFunction().logEvaluate(OnOffEdgeTransDistribution.stateOnToOff))
            - totalProb;
      } else {
        return LogMath.add(
            getFreeMotionTransPrior().getProbabilityFunction().logEvaluate(OnOffEdgeTransDistribution.stateOffToOn),
            getEdgeMotionTransPrior().getProbabilityFunction().logEvaluate(OnOffEdgeTransDistribution.stateOnToOn))
            - totalProb;
      }
    } else {
      if (from.isNullEdge()) {
        return getFreeMotionTransPrior().getProbabilityFunction().logEvaluate(getTransitionType(from, to));
      } else {
        return getEdgeMotionTransPrior().getProbabilityFunction().logEvaluate(getTransitionType(from, to));
      }
    }
  }

  public InferredEdge sample(Random rng, List<InferredEdge> transferEdges, InferredEdge inferredEdge) {

    Preconditions.checkNotNull(inferredEdge);
    Preconditions.checkNotNull(transferEdges);

    if (inferredEdge.isNullEdge()) {
      /*
       * We're currently in free-motion. If there are transfer edges, then
       * sample from those.
       */
      if (transferEdges.isEmpty()) {
        return graph.getNullInferredEdge();
      } else {
        final Vector sample = checkedSample(this.getFreeMotionTransPrior(), rng);

        if (sample.equals(stateOffToOn)) {
          final List<InferredEdge> support = Lists.newArrayList(transferEdges);
          support.remove(graph.getNullInferredEdge());
          return support.get(rng.nextInt(support.size()));
        } else {
          return graph.getNullInferredEdge();
        }
      }
    } else {
      /*
       * We're on an edge, so sample whether we go off-road, or transfer/stay
       * on.
       * If the empty edge is contained in the support (transferEdges)
       * then we sample for that.
       */
      final Vector sample =
          transferEdges.contains(graph.getNullInferredEdge()) ? checkedSample(this.getEdgeMotionTransPrior(), rng)
              : stateOnToOn;

      if (sample.equals(stateOnToOff) || transferEdges.isEmpty()) {
        return graph.getNullInferredEdge();
      } else {
        final List<InferredEdge> support = Lists.newArrayList(transferEdges);
        support.remove(graph.getNullInferredEdge());
        return support.get(rng.nextInt(support.size()));
      }

    }

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
  public static Vector checkedSample(final ClosedFormComputableDistribution<Vector> dist, Random rng) {

    if (dist instanceof ClosedFormComputableDiscreteDistribution) {
      ClosedFormComputableDiscreteDistribution<Vector> distDiscrete =
          (ClosedFormComputableDiscreteDistribution<Vector>) dist;
      Vector one = Iterators.find(distDiscrete.getDomain().iterator(), new Predicate<Vector>() {
        @Override
        public boolean apply(Vector input) {
          return Math.abs(1 - dist.getProbabilityFunction().evaluate(input)) <= zeroTolerance;
        }
      }, null);

      return one == null ? dist.sample(rng) : one;

    } else if (dist instanceof DirichletDistribution) {
      for (int i = 0; i < dist.getMean().getDimensionality(); i++) {
        final double prob = dist.getMean().getElement(i);
        if (Double.isNaN(prob) || Math.abs(1 - prob) <= zeroTolerance) {
          Vector result = VectorFactory.getDefault().createVector(dist.getMean().getDimensionality());
          result.setElement(i, 1d);
          return result;
        }
      }
    }

    return dist.sample(rng);
  }

  public void setEdgeMotionTransPrior(MultinomialDistribution edgeMotionTransPrior) {
    this.edgeMotionTransPrior = edgeMotionTransPrior;
  }

  public void setEdgeMotionTransProbPrior(DirichletDistribution edgeMotionTransProbPrior) {
    this.edgeMotionTransProbPrior = edgeMotionTransProbPrior;
  }

  public void setFreeMotionTransPrior(MultinomialDistribution freeMotionTransPrior) {
    this.freeMotionTransPrior = freeMotionTransPrior;
  }

  public void setFreeMotionTransProbPrior(DirichletDistribution freeMotionTransProbPrior) {
    this.freeMotionTransProbPrior = freeMotionTransProbPrior;
  }

  @Override
  public String toString() {
    return "EdgeTransitionDistributions [freeMotionTransPrior=" + getFreeMotionTransPrior().getParameters()
        + ", edgeMotionTransPrior=" + getEdgeMotionTransPrior().getParameters() + "]";
  }

  public void update(InferredEdge from, InferredEdge to) {
    final Vector transType = getTransitionType(from, to);
    if (from.isNullEdge()) {
      freeMotionTransEstimator.update(getFreeMotionTransProbPrior(), transType);
      freeMotionTransPrior.setParameters(freeMotionTransProbPrior.getMean());
    } else {
      edgeMotionTransEstimator.update(getEdgeMotionTransProbPrior(), transType);
      edgeMotionTransPrior.setParameters(edgeMotionTransProbPrior.getMean());
    }
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

  public static Vector getTransitionType(InferredEdge from, InferredEdge to) {
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

}
