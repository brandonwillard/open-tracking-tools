package org.opentrackingtools.util.model;

import gov.sandia.cognition.math.matrix.Vector;

public class TransitionProbMatrix {
  
  protected Vector edgeTransitionProbs;
  protected Vector freeTransitionProbs;

  public TransitionProbMatrix(Vector edgeTransitionProbs, Vector freeTransitionProbs) {
    this.edgeTransitionProbs = edgeTransitionProbs;
    this.freeTransitionProbs = freeTransitionProbs;
  }

  public Vector getEdgeMotionTransProbs() {
    return this.edgeTransitionProbs;
  }

  public Vector getFreeMotionTransProbs() {
    return this.freeTransitionProbs;
  }

}
