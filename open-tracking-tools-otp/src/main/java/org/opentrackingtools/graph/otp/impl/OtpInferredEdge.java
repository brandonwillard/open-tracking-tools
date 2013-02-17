package org.opentrackingtools.graph.otp.impl;

import javax.annotation.Nonnull;

import gov.sandia.cognition.statistics.bayesian.conjugate.UnivariateGaussianMeanVarianceBayesianEstimator;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.statistics.distribution.NormalInverseGammaDistribution;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.opentrackingtools.graph.InferenceGraph;
import org.opentrackingtools.graph.edges.InferredEdge;
import org.opentrackingtools.graph.edges.impl.SimpleInferredEdge;

import com.vividsolutions.jts.geom.Geometry;

public class OtpInferredEdge extends SimpleInferredEdge {

  protected final static InferredEdge emptyEdge =
      new OtpInferredEdge();
  
  protected final NormalInverseGammaDistribution velocityPrecisionDist;
  protected final UnivariateGaussianMeanVarianceBayesianEstimator velocityEstimator;

  protected OtpInferredEdge() {
    super();
    this.velocityEstimator = null;
    this.velocityPrecisionDist = null;
  }
  
  protected OtpInferredEdge(@Nonnull Geometry geom, 
    @Nonnull Object backingEdge,
    @Nonnull Integer edgeId, 
    @Nonnull InferenceGraph graph) {
    super(geom, backingEdge, edgeId, graph);
    this.velocityPrecisionDist =
        // ~4.4 m/s, std. dev ~ 30 m/s, Gamma with exp. value = 30 m/s
        // TODO perhaps variance of velocity should be in m/s^2. yeah...
        new NormalInverseGammaDistribution(4.4d,
            1d / Math.pow(30d, 2d),
            1d / Math.pow(30d, 2d) + 1d, Math.pow(30d, 2d));
    this.velocityEstimator =
        new UnivariateGaussianMeanVarianceBayesianEstimator(
            velocityPrecisionDist);
  }
    
  @Override
  public int compareTo(InferredEdge o) {
    final CompareToBuilder comparator =
        new CompareToBuilder();
    comparator.appendSuper(super.compareTo(o));
    if (o instanceof OtpInferredEdge) {
      comparator.append(this.velocityPrecisionDist.convertToVector(),
          ((OtpInferredEdge)o).velocityPrecisionDist.convertToVector());
    }
    return comparator.toComparison();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj)
      return true;
    if (!super.equals(obj))
      return false;
    if (getClass() != obj.getClass())
      return false;
    OtpInferredEdge other = (OtpInferredEdge) obj;
    if (velocityPrecisionDist == null) {
      if (other.velocityPrecisionDist != null)
        return false;
    } else if (!velocityPrecisionDist.convertToVector()
        .equals(other.velocityPrecisionDist.convertToVector()))
      return false;
    return true;
  }

  public UnivariateGaussianMeanVarianceBayesianEstimator
      getVelocityEstimator() {
    return velocityEstimator;
  }

  public NormalInverseGammaDistribution
      getVelocityPrecisionDist() {
    return velocityPrecisionDist;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result =
        prime
            * result
            + ((velocityPrecisionDist == null) ? 0
                : velocityPrecisionDist.convertToVector().hashCode());
    return result;
  }

  @Override
  public boolean isNullEdge() {
    return this == emptyEdge;
  }

  @Override
  public String toString() {
    return super.toString();
  }

  @Override
  public void update(MultivariateGaussian stateBelief) {
    super.update(stateBelief);
    final double velocity =
        stateBelief.getMean().getElement(1);
    this.getVelocityEstimator()
        .update(this.getVelocityPrecisionDist(), Math.abs(velocity));
  }


}
