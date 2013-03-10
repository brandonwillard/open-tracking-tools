package org.opentrackingtools;

import gov.sandia.cognition.math.matrix.Vector;

public class VehicleStateInitialParameters {

  private final int initialObsFreq;
  private final int numParticles;
  private final Vector obsCov;
  private final int obsCovDof;
  private final int offRoadCovDof;
  private final Vector offRoadStateCov;
  private final Vector offTransitionProbs;
  private final int onRoadCovDof;
  private final Vector onRoadStateCov;
  private final Vector onTransitionProbs;
  private final String particleFilterTypeName;
  private final String roadFilterTypeName;
  private final long seed;

  public VehicleStateInitialParameters(Vector obsCov, int obsCovDof,
    Vector onRoadStateCov, int onRoadCovDof, Vector offRoadStateCov,
    int offRoadCovDof, Vector offProbs, Vector onProbs,
    String particleFilterTypeName, String roadFilterTypeName,
    int numParticles, int initialObsFreq, long seed) {
    this.obsCovDof = obsCovDof;
    this.onRoadCovDof = onRoadCovDof;
    this.offRoadCovDof = offRoadCovDof;
    this.numParticles = numParticles;
    this.obsCov = obsCov;
    this.onRoadStateCov = onRoadStateCov;
    this.offRoadStateCov = offRoadStateCov;
    this.offTransitionProbs = offProbs;
    this.onTransitionProbs = onProbs;
    this.seed = seed;
    this.particleFilterTypeName = particleFilterTypeName;
    this.roadFilterTypeName = roadFilterTypeName;
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
    if (this.particleFilterTypeName == null) {
      if (other.particleFilterTypeName != null) {
        return false;
      }
    } else if (!this.particleFilterTypeName
        .equals(other.particleFilterTypeName)) {
      return false;
    }
    if (this.roadFilterTypeName == null) {
      if (other.roadFilterTypeName != null) {
        return false;
      }
    } else if (!this.roadFilterTypeName
        .equals(other.roadFilterTypeName)) {
      return false;
    }
    if (this.initialObsFreq != other.initialObsFreq) {
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
    if (this.offTransitionProbs == null) {
      if (other.offTransitionProbs != null) {
        return false;
      }
    } else if (!this.offTransitionProbs
        .equals(other.offTransitionProbs)) {
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
    if (this.onTransitionProbs == null) {
      if (other.onTransitionProbs != null) {
        return false;
      }
    } else if (!this.onTransitionProbs
        .equals(other.onTransitionProbs)) {
      return false;
    }
    if (this.seed != other.seed) {
      return false;
    }
    return true;
  }

  public int getInitialObsFreq() {
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

  public Vector getOffTransitionProbs() {
    return this.offTransitionProbs;
  }

  public int getOnRoadCovDof() {
    return this.onRoadCovDof;
  }

  public Vector getOnRoadStateCov() {
    return this.onRoadStateCov;
  }

  public Vector getOnTransitionProbs() {
    return this.onTransitionProbs;
  }

  public String getParticleFilterTypeName() {
    return this.particleFilterTypeName;
  }

  public String getRoadFilterTypeName() {
    return this.roadFilterTypeName;
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
            + ((this.particleFilterTypeName == null) ? 0
                : this.particleFilterTypeName.hashCode());
    result =
        prime
            * result
            + ((this.roadFilterTypeName == null) ? 0
                : this.roadFilterTypeName.hashCode());
    result = prime * result + this.initialObsFreq;
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
            + ((this.offTransitionProbs == null) ? 0
                : this.offTransitionProbs.hashCode());
    result = prime * result + this.onRoadCovDof;
    result =
        prime
            * result
            + ((this.onRoadStateCov == null) ? 0
                : this.onRoadStateCov.hashCode());
    result =
        prime
            * result
            + ((this.onTransitionProbs == null) ? 0
                : this.onTransitionProbs.hashCode());
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
        .append(this.offTransitionProbs)
        .append(", onTransitionProbs=")
        .append(this.onTransitionProbs).append(", seed=")
        .append(this.seed).append(", numParticles=")
        .append(this.numParticles)
        .append(", particleFilterTypeName=")
        .append(this.particleFilterTypeName)
        .append(", roadFilterTypeName=")
        .append(this.roadFilterTypeName).append(", initialObsFreq=")
        .append(this.initialObsFreq).append(", obsCovDof=")
        .append(this.obsCovDof).append(", onRoadCovDof=")
        .append(this.onRoadCovDof).append(", offRoadCovDof=")
        .append(this.offRoadCovDof).append("]");
    return builder.toString();
  }
}