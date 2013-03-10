package org.opentrackingtools.distributions;

import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.ArrayList;
import java.util.Random;

import org.opentrackingtools.graph.InferenceGraphEdge;
import org.opentrackingtools.paths.PathEdge;

public class PathEdgeProbabilityFunction<E extends PathEdge<? extends InferenceGraphEdge>>
    extends AbstractCloneableSerializable implements
    ProbabilityFunction<E> {

  /**
   * 
   */
  private static final long serialVersionUID = -4724467993995429525L;
  protected E currentEdge;

  public PathEdgeProbabilityFunction(E pathEdge) {
    this.currentEdge = pathEdge;
  }

  @Override
  public Double evaluate(E input) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ProbabilityFunction<E> getProbabilityFunction() {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public double logEvaluate(E input) {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public E sample(Random random) {
    // TODO Auto-generated method stub
    return null;
  }

  @Override
  public ArrayList<E> sample(Random random, int numSamples) {
    // TODO Auto-generated method stub
    return null;
  }

}
