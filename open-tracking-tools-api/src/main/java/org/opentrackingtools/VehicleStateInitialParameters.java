package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Vector;

public class VehicleStateInitialParameters {

  protected Vector initialMotionState;

  protected double initialObsFreq;
  protected int numParticles;

  protected Vector obsCov;
  protected int obsCovDof;

  protected int offRoadCovDof;
  protected Vector offRoadStateCov;

  protected Vector offTransitionProbsPrior;
  protected int onRoadCovDof;
  protected Vector onRoadStateCov;

  protected Vector onTransitionProbsPrior;
  protected long seed;

  protected Double obsCovarianceThreshold;
  protected Double stateCovarianceThreshold;

  protected double pathDistanceSearchUpperBound;
  
  public VehicleStateInitialParameters(Vector initialMotionState,
    Vector obsCov, int obsCovDof, Vector onRoadStateCov,
    int onRoadCovDof, Vector offRoadStateCov, int offRoadCovDof, 
    Vector offProbs, Vector onProbs, int numParticles,
    double initialObsFreq, long seed) {
    this(initialMotionState, obsCov, obsCovDof, null, onRoadStateCov, onRoadCovDof, 
        offRoadStateCov, offRoadCovDof, null, offProbs, onProbs, numParticles, 
        initialObsFreq, 54d * 30d, seed);
  }

  public VehicleStateInitialParameters(Vector initialMotionState,
    Vector obsCov, int obsCovDof, Double obsCovarianceThreshold, Vector onRoadStateCov,
    int onRoadCovDof, Vector offRoadStateCov, int offRoadCovDof, 
    Double stateCovarianceThreshold, Vector offProbs, Vector onProbs, int numParticles,
    double initialObsFreq, double pathDistanceSearchUpperBound, long seed) {

    this.pathDistanceSearchUpperBound = pathDistanceSearchUpperBound;
    this.stateCovarianceThreshold = stateCovarianceThreshold;
    this.obsCovarianceThreshold = obsCovarianceThreshold;
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

  public VehicleStateInitialParameters(VehicleStateInitialParameters parameters) {
    this.initialMotionState = parameters.initialMotionState;
    this.obsCovDof = parameters.obsCovDof;
    this.onRoadCovDof = parameters.onRoadCovDof;
    this.offRoadCovDof = parameters.offRoadCovDof;
    this.numParticles = parameters.numParticles;
    this.obsCov = parameters.obsCov;
    this.onRoadStateCov = parameters.onRoadStateCov;
    this.offRoadStateCov = parameters.offRoadStateCov;
    this.offTransitionProbsPrior = parameters.offTransitionProbsPrior;
    this.onTransitionProbsPrior = parameters.onTransitionProbsPrior;
    this.seed = parameters.seed;
    this.initialObsFreq = parameters.initialObsFreq;
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

  public void setInitialMotionState(Vector initialMotionState) {
    this.initialMotionState = initialMotionState;
  }

  public void setInitialObsFreq(double initialObsFreq) {
    this.initialObsFreq = initialObsFreq;
  }

  public void setNumParticles(int numParticles) {
    this.numParticles = numParticles;
  }

  public void setObsCov(Vector obsCov) {
    this.obsCov = obsCov;
  }

  public void setObsCovDof(int obsCovDof) {
    this.obsCovDof = obsCovDof;
  }

  public void setOffRoadCovDof(int offRoadCovDof) {
    this.offRoadCovDof = offRoadCovDof;
  }

  public void setOffRoadStateCov(Vector offRoadStateCov) {
    this.offRoadStateCov = offRoadStateCov;
  }

  public void setOffTransitionProbsPrior(Vector offTransitionProbsPrior) {
    this.offTransitionProbsPrior = offTransitionProbsPrior;
  }

  public void setOnRoadCovDof(int onRoadCovDof) {
    this.onRoadCovDof = onRoadCovDof;
  }

  public void setOnRoadStateCov(Vector onRoadStateCov) {
    this.onRoadStateCov = onRoadStateCov;
  }

  public void setOnTransitionProbsPrior(Vector onTransitionProbsPrior) {
    this.onTransitionProbsPrior = onTransitionProbsPrior;
  }

  public void setSeed(long seed) {
    this.seed = seed;
  }

  public Double getObsCovarianceThreshold() {
    return obsCovarianceThreshold;
  }

  public void setObsCovarianceThreshold(Double obsCovarianceThreshold) {
    this.obsCovarianceThreshold = obsCovarianceThreshold;
  }

  public Double getStateCovarianceThreshold() {
    return stateCovarianceThreshold;
  }

  public void setStateCovarianceThreshold(Double stateCovarianceThreshold) {
    this.stateCovarianceThreshold = stateCovarianceThreshold;
  }

  public double getPathDistanceSearchUpperBound() {
    return this.pathDistanceSearchUpperBound;
  }

  public void setPathDistanceSearchUpperBound(double pathDistanceSearchUpperBound) {
    this.pathDistanceSearchUpperBound = pathDistanceSearchUpperBound;
  }
}