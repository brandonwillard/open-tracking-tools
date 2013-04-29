package org.opentrackingtools.util.model;

import gov.sandia.cognition.math.matrix.Vector;

import org.apache.commons.lang3.builder.CompareToBuilder;

public class TransitionProbMatrix implements
    Comparable<TransitionProbMatrix> {

  protected Vector edgeTransitionProbs;
  protected Vector freeTransitionProbs;

  public TransitionProbMatrix(Vector edgeTransitionProbs,
    Vector freeTransitionProbs) {
    this.edgeTransitionProbs = edgeTransitionProbs;
    this.freeTransitionProbs = freeTransitionProbs;
  }

  @Override
  public int compareTo(TransitionProbMatrix o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.edgeTransitionProbs.toArray(),
        o.edgeTransitionProbs.toArray());
    comparator.append(this.freeTransitionProbs.toArray(),
        o.freeTransitionProbs.toArray());
    return comparator.build();
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
    final TransitionProbMatrix other = (TransitionProbMatrix) obj;
    if (this.edgeTransitionProbs == null) {
      if (other.edgeTransitionProbs != null) {
        return false;
      }
    } else if (!this.edgeTransitionProbs
        .equals(other.edgeTransitionProbs)) {
      return false;
    }
    if (this.freeTransitionProbs == null) {
      if (other.freeTransitionProbs != null) {
        return false;
      }
    } else if (!this.freeTransitionProbs
        .equals(other.freeTransitionProbs)) {
      return false;
    }
    return true;
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
    result =
        prime
            * result
            + ((this.edgeTransitionProbs == null) ? 0
                : this.edgeTransitionProbs.hashCode());
    result =
        prime
            * result
            + ((this.freeTransitionProbs == null) ? 0
                : this.freeTransitionProbs.hashCode());
    return result;
  }

}
