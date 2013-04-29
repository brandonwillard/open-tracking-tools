package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Vector;

public class VehicleStateInitialParameters {

  protected final Vector initialMotionState;

  protected final double initialObsFreq;
  protected final int numParticles;

  protected final Vector obsCov;
  protected final int obsCovDof;

  protected final int offRoadCovDof;
  protected final Vector offRoadStateCov;

  protected final Vector offTransitionProbsPrior;
  protected final int onRoadCovDof;
  protected final Vector onRoadStateCov;

  protected final Vector onTransitionProbsPrior;
  protected final long seed;

  public VehicleStateInitialParameters(Vector initialMotionState,
    Vector obsCov, int obsCovDof, Vector onRoadStateCov,
    int onRoadCovDof, Vector offRoadStateCov, int offRoadCovDof,
    Vector offProbs, Vector onProbs, int numParticles,
    double initialObsFreq, long seed) {

    this.initialMotionState = initialMotionState;
    this.obsCovDof = obsCovDof;
    this.onRoadCovDof = onRoadCovDof;
    this.offRoadCovDof = offRoadCovDof;
    this.numParticles = numParticles;
    this.obsCov = obsCov;
    this.onRoadStateCov = onRoadStateCov;
    this.offRoadStateCov = offRoadStateCov;
    this.offTransitionProbsPrior = offProbs;
    this.onTransitionProbsPrior = onProbs;
    this.seed = seed;
    this.initialObsFreq = initialObsFreq;
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
    final VehicleStateInitialParameters other =
        (VehicleStateInitialParameters) obj;
    if (this.initialMotionState == null) {
      if (other.initialMotionState != null) {
        return false;
      }
    } else if (!this.initialMotionState
        .equals(other.initialMotionState)) {
      return false;
    }
    if (Double.doubleToLongBits(this.initialObsFreq) != Double
        .doubleToLongBits(other.initialObsFreq)) {
      return false;
    }
    if (this.numParticles != other.numParticles) {
      return false;
    }
    if (this.obsCov == null) {
      if (other.obsCov != null) {
        return false;
      }
    } else if (!this.obsCov.equals(other.obsCov)) {
      return false;
    }
    if (this.obsCovDof != other.obsCovDof) {
      return false;
    }
    if (this.offRoadCovDof != other.offRoadCovDof) {
      return false;
    }
    if (this.offRoadStateCov == null) {
      if (other.offRoadStateCov != null) {
        return false;
      }
    } else if (!this.offRoadStateCov.equals(other.offRoadStateCov)) {
      return false;
    }
    if (this.offTransitionProbsPrior == null) {
      if (other.offTransitionProbsPrior != null) {
        return false;
      }
    } else if (!this.offTransitionProbsPrior
        .equals(other.offTransitionProbsPrior)) {
      return false;
    }
    if (this.onRoadCovDof != other.onRoadCovDof) {
      return false;
    }
    if (this.onRoadStateCov == null) {
      if (other.onRoadStateCov != null) {
        return false;
      }
    } else if (!this.onRoadStateCov.equals(other.onRoadStateCov)) {
      return false;
    }
    if (this.onTransitionProbsPrior == null) {
      if (other.onTransitionProbsPrior != null) {
        return false;
      }
    } else if (!this.onTransitionProbsPrior
        .equals(other.onTransitionProbsPrior)) {
      return false;
    }
    if (this.seed != other.seed) {
      return false;
    }
    return true;
  }

  public Vector getInitialMotionState() {
    return this.initialMotionState;
  }

  public double getInitialObsFreq() {
    return this.initialObsFreq;
  }

  public int getNumParticles() {
    return this.numParticles;
  }

  public Vector getObsCov() {
    return this.obsCov;
  }

  public int getObsCovDof() {
    return this.obsCovDof;
  }

  public int getOffRoadCovDof() {
    return this.offRoadCovDof;
  }

  public Vector getOffRoadStateCov() {
    return this.offRoadStateCov;
  }

  public Vector getOffTransitionProbsPrior() {
    return this.offTransitionProbsPrior;
  }

  public int getOnRoadCovDof() {
    return this.onRoadCovDof;
  }

  public Vector getOnRoadStateCov() {
    return this.onRoadStateCov;
  }

  public Vector getOnTransitionProbsPrior() {
    return this.onTransitionProbsPrior;
  }

  public long getSeed() {
    return this.seed;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.initialMotionState == null) ? 0
                : this.initialMotionState.hashCode());
    long temp;
    temp = Double.doubleToLongBits(this.initialObsFreq);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    result = prime * result + this.numParticles;
    result =
        prime * result
            + ((this.obsCov == null) ? 0 : this.obsCov.hashCode());
    result = prime * result + this.obsCovDof;
    result = prime * result + this.offRoadCovDof;
    result =
        prime
            * result
            + ((this.offRoadStateCov == null) ? 0
                : this.offRoadStateCov.hashCode());
    result =
        prime
            * result
            + ((this.offTransitionProbsPrior == null) ? 0
                : this.offTransitionProbsPrior.hashCode());
    result = prime * result + this.onRoadCovDof;
    result =
        prime
            * result
            + ((this.onRoadStateCov == null) ? 0
                : this.onRoadStateCov.hashCode());
    result =
        prime
            * result
            + ((this.onTransitionProbsPrior == null) ? 0
                : this.onTransitionProbsPrior.hashCode());
    result = prime * result + (int) (this.seed ^ (this.seed >>> 32));
    return result;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder.append("VehicleStateInitialParameters [obsCov=")
        .append(this.obsCov).append(", onRoadStateCov=")
        .append(this.onRoadStateCov).append(", offRoadStateCov=")
        .append(this.offRoadStateCov).append(", offTransitionProbs=")
        .append(this.offTransitionProbsPrior)
        .append(", onTransitionProbs=")
        .append(this.onTransitionProbsPrior).append(", seed=")
        .append(this.seed).append(", numParticles=")
        .append(this.numParticles).append(", initialMotionState=")
        .append(this.initialMotionState).append(", initialObsFreq=")
        .append(this.initialObsFreq).append(", obsCovDof=")
        .append(this.obsCovDof).append(", onRoadCovDof=")
        .append(this.onRoadCovDof).append(", offRoadCovDof=")
        .append(this.offRoadCovDof).append("]");
    return builder.toString();
  }
}