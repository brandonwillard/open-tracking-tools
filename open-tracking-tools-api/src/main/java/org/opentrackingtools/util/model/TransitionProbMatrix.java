package org.opentrackingtools.util.model;

import org.apache.commons.lang3.builder.CompareToBuilder;

import gov.sandia.cognition.math.matrix.Vector;

public class TransitionProbMatrix implements Comparable<TransitionProbMatrix>{

  protected Vector edgeTransitionProbs;
  protected Vector freeTransitionProbs;

  public TransitionProbMatrix(Vector edgeTransitionProbs,
    Vector freeTransitionProbs) {
    this.edgeTransitionProbs = edgeTransitionProbs;
    this.freeTransitionProbs = freeTransitionProbs;
  }

  public Vector getEdgeMotionTransProbs() {
    return this.edgeTransitionProbs;
  }

  public Vector getFreeMotionTransProbs() {
    return this.freeTransitionProbs;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result
        + ((edgeTransitionProbs == null) ? 0 : edgeTransitionProbs.hashCode());
    result = prime * result
        + ((freeTransitionProbs == null) ? 0 : freeTransitionProbs.hashCode());
    return result;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (!(obj instanceof TransitionProbMatrix)) {
      return false;
    }
    TransitionProbMatrix other = (TransitionProbMatrix) obj;
    if (edgeTransitionProbs == null) {
      if (other.edgeTransitionProbs != null) {
        return false;
      }
    } else if (!edgeTransitionProbs.equals(other.edgeTransitionProbs)) {
      return false;
    }
    if (freeTransitionProbs == null) {
      if (other.freeTransitionProbs != null) {
        return false;
      }
    } else if (!freeTransitionProbs.equals(other.freeTransitionProbs)) {
      return false;
    }
    return true;
  }

  @Override
  public int compareTo(TransitionProbMatrix o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edgeTransitionProbs.toArray(), o.edgeTransitionProbs.toArray());
    comparator.append(this.freeTransitionProbs.toArray(), o.freeTransitionProbs.toArray());
    return comparator.build();
  }

}
