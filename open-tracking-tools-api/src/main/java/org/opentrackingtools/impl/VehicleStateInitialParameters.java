package org.opentrackingtools.impl;

import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import org.apache.commons.lang.builder.CompareToBuilder;

public class VehicleStateInitialParameters extends
    AbstractCloneableSerializable implements Comparable<VehicleStateInitialParameters> {
  private static final long serialVersionUID =
      3613725475525876941L;
  private final Vector obsCov;
  private final Vector onRoadStateCov;
  private final Vector offRoadStateCov;
  private final Vector offTransitionProbs;
  private final Vector onTransitionProbs;
  private final long seed;
  private final int numParticles;
  private final String filterTypeName;
  private final int initialObsFreq;
  private final int obsCovDof;
  private final int onRoadCovDof;
  private final int offRoadCovDof;

  public VehicleStateInitialParameters(Vector obsCov,
    int obsCovDof, Vector onRoadStateCov,
    int onRoadCovDof, Vector offRoadStateCov,
    int offRoadCovDof, Vector offProbs, Vector onProbs,
    String filterTypeName, int numParticles,
    int initialObsFreq, long seed) {
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
    this.filterTypeName = filterTypeName;
    this.initialObsFreq = initialObsFreq;
  }

  @Override
  public VehicleStateInitialParameters clone() {
    final VehicleStateInitialParameters clone =
        (VehicleStateInitialParameters) super.clone();
    // TODO
    return clone;
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
    final VehicleStateInitialParameters other =
        (VehicleStateInitialParameters) obj;
    if (filterTypeName == null) {
      if (other.filterTypeName != null) {
        return false;
      }
    } else if (!filterTypeName
        .equals(other.filterTypeName)) {
      return false;
    }
    if (initialObsFreq != other.initialObsFreq) {
      return false;
    }
    if (numParticles != other.numParticles) {
      return false;
    }
    if (obsCov == null) {
      if (other.obsCov != null) {
        return false;
      }
    } else if (!obsCov.equals(other.obsCov)) {
      return false;
    }
    if (obsCovDof != other.obsCovDof) {
      return false;
    }
    if (offRoadCovDof != other.offRoadCovDof) {
      return false;
    }
    if (offRoadStateCov == null) {
      if (other.offRoadStateCov != null) {
        return false;
      }
    } else if (!offRoadStateCov
        .equals(other.offRoadStateCov)) {
      return false;
    }
    if (offTransitionProbs == null) {
      if (other.offTransitionProbs != null) {
        return false;
      }
    } else if (!offTransitionProbs
        .equals(other.offTransitionProbs)) {
      return false;
    }
    if (onRoadCovDof != other.onRoadCovDof) {
      return false;
    }
    if (onRoadStateCov == null) {
      if (other.onRoadStateCov != null) {
        return false;
      }
    } else if (!onRoadStateCov
        .equals(other.onRoadStateCov)) {
      return false;
    }
    if (onTransitionProbs == null) {
      if (other.onTransitionProbs != null) {
        return false;
      }
    } else if (!onTransitionProbs
        .equals(other.onTransitionProbs)) {
      return false;
    }
    if (seed != other.seed) {
      return false;
    }
    return true;
  }

  public String getFilterTypeName() {
    return filterTypeName;
  }

  public int getInitialObsFreq() {
    return this.initialObsFreq;
  }

  public int getNumParticles() {
    return numParticles;
  }

  public Vector getObsCov() {
    return obsCov;
  }

  public int getObsCovDof() {
    return this.obsCovDof;
  }

  public int getOffRoadCovDof() {
    return offRoadCovDof;
  }

  public Vector getOffRoadStateCov() {
    return offRoadStateCov;
  }

  public Vector getOffTransitionProbs() {
    return offTransitionProbs;
  }

  public int getOnRoadCovDof() {
    return onRoadCovDof;
  }

  public Vector getOnRoadStateCov() {
    return onRoadStateCov;
  }

  public Vector getOnTransitionProbs() {
    return onTransitionProbs;
  }

  public long getSeed() {
    return seed;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((filterTypeName == null) ? 0
                : filterTypeName.hashCode());
    result = prime * result + initialObsFreq;
    result = prime * result + numParticles;
    result =
        prime * result
            + ((obsCov == null) ? 0 : obsCov.hashCode());
    result = prime * result + obsCovDof;
    result = prime * result + offRoadCovDof;
    result =
        prime
            * result
            + ((offRoadStateCov == null) ? 0
                : offRoadStateCov.hashCode());
    result =
        prime
            * result
            + ((offTransitionProbs == null) ? 0
                : offTransitionProbs.hashCode());
    result = prime * result + onRoadCovDof;
    result =
        prime
            * result
            + ((onRoadStateCov == null) ? 0
                : onRoadStateCov.hashCode());
    result =
        prime
            * result
            + ((onTransitionProbs == null) ? 0
                : onTransitionProbs.hashCode());
    result =
        prime * result + (int) (seed ^ (seed >>> 32));
    return result;
  }

  @Override
  public String toString() {
    final StringBuilder builder = new StringBuilder();
    builder
        .append("VehicleStateInitialParameters [obsCov=")
        .append(obsCov).append(", onRoadStateCov=")
        .append(onRoadStateCov)
        .append(", offRoadStateCov=")
        .append(offRoadStateCov)
        .append(", offTransitionProbs=")
        .append(offTransitionProbs)
        .append(", onTransitionProbs=")
        .append(onTransitionProbs).append(", seed=")
        .append(seed).append(", numParticles=")
        .append(numParticles).append(", filterTypeName=")
        .append(filterTypeName)
        .append(", initialObsFreq=")
        .append(initialObsFreq).append(", obsCovDof=")
        .append(obsCovDof).append(", onRoadCovDof=")
        .append(onRoadCovDof).append(", offRoadCovDof=")
        .append(offRoadCovDof).append("]");
    return builder.toString();
  }

  @Override
  public int compareTo(VehicleStateInitialParameters o) {
   return new CompareToBuilder()
     .append(this.filterTypeName, o.filterTypeName)
     .append(this.initialObsFreq, o.initialObsFreq)
     .append(this.numParticles, o.numParticles)
     .append(this.obsCov.toArray(), o.obsCov.toArray())
     .append(this.obsCovDof, o.obsCovDof)
     .append(this.offRoadStateCov.toArray(), o.offRoadStateCov.toArray())
     .append(this.offRoadCovDof, o.offRoadCovDof)
     .append(this.onRoadStateCov.toArray(), o.onRoadStateCov.toArray())
     .append(this.onRoadCovDof, o.onRoadCovDof)
     .append(this.initialObsFreq, o.initialObsFreq)
     .append(this.onTransitionProbs.toArray(), o.onTransitionProbs.toArray())
     .append(this.offTransitionProbs.toArray(), o.offTransitionProbs.toArray())
     .toComparison();
  }
}