package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.Vector1D;
import gov.sandia.cognition.math.matrix.Vector2D;

public class VehicleStateInitialParameters {

  protected final Vector initialMotionState;
  protected final double initialObsFreq;
  protected final int numParticles;
  
  protected final Vector obsCov;
  protected final int obsCovDof;
  
  protected final int offRoadCovDof;
  protected final Vector offRoadStateCov;
  
  protected final int onRoadCovDof;
  protected final Vector onRoadStateCov;
  protected final long seed;
  
  protected final Vector onTransitionProbsPrior;
  protected final Vector offTransitionProbsPrior;

  public VehicleStateInitialParameters(
    Vector initialMotionState,
    Vector obsCov, int obsCovDof,
    Vector onRoadStateCov, int onRoadCovDof, Vector offRoadStateCov,
    int offRoadCovDof, Vector offProbs, Vector onProbs,
    int numParticles, double initialObsFreq, long seed) {
    
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
    if (this == obj)
      return true;
    if (obj == null)
      return false;
    if (getClass() != obj.getClass())
      return false;
    VehicleStateInitialParameters other =
        (VehicleStateInitialParameters) obj;
    if (initialMotionState == null) {
      if (other.initialMotionState != null)
        return false;
    } else if (!initialMotionState.equals(other.initialMotionState))
      return false;
    if (Double.doubleToLongBits(initialObsFreq) != Double
        .doubleToLongBits(other.initialObsFreq))
      return false;
    if (numParticles != other.numParticles)
      return false;
    if (obsCov == null) {
      if (other.obsCov != null)
        return false;
    } else if (!obsCov.equals(other.obsCov))
      return false;
    if (obsCovDof != other.obsCovDof)
      return false;
    if (offRoadCovDof != other.offRoadCovDof)
      return false;
    if (offRoadStateCov == null) {
      if (other.offRoadStateCov != null)
        return false;
    } else if (!offRoadStateCov.equals(other.offRoadStateCov))
      return false;
    if (offTransitionProbsPrior == null) {
      if (other.offTransitionProbsPrior != null)
        return false;
    } else if (!offTransitionProbsPrior
        .equals(other.offTransitionProbsPrior))
      return false;
    if (onRoadCovDof != other.onRoadCovDof)
      return false;
    if (onRoadStateCov == null) {
      if (other.onRoadStateCov != null)
        return false;
    } else if (!onRoadStateCov.equals(other.onRoadStateCov))
      return false;
    if (onTransitionProbsPrior == null) {
      if (other.onTransitionProbsPrior != null)
        return false;
    } else if (!onTransitionProbsPrior
        .equals(other.onTransitionProbsPrior))
      return false;
    if (seed != other.seed)
      return false;
    return true;
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
            + ((initialMotionState == null) ? 0 : initialMotionState
                .hashCode());
    long temp;
    temp = Double.doubleToLongBits(initialObsFreq);
    result = prime * result + (int) (temp ^ (temp >>> 32));
    result = prime * result + numParticles;
    result =
        prime * result + ((obsCov == null) ? 0 : obsCov.hashCode());
    result = prime * result + obsCovDof;
    result = prime * result + offRoadCovDof;
    result =
        prime
            * result
            + ((offRoadStateCov == null) ? 0 : offRoadStateCov
                .hashCode());
    result =
        prime
            * result
            + ((offTransitionProbsPrior == null) ? 0
                : offTransitionProbsPrior.hashCode());
    result = prime * result + onRoadCovDof;
    result =
        prime
            * result
            + ((onRoadStateCov == null) ? 0 : onRoadStateCov
                .hashCode());
    result =
        prime
            * result
            + ((onTransitionProbsPrior == null) ? 0
                : onTransitionProbsPrior.hashCode());
    result = prime * result + (int) (seed ^ (seed >>> 32));
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
        .append(this.numParticles)
        .append(", initialMotionState=")
        .append(this.initialMotionState)
        .append(", initialObsFreq=")
        .append(this.initialObsFreq).append(", obsCovDof=")
        .append(this.obsCovDof).append(", onRoadCovDof=")
        .append(this.onRoadCovDof).append(", offRoadCovDof=")
        .append(this.offRoadCovDof).append("]");
    return builder.toString();
  }
}