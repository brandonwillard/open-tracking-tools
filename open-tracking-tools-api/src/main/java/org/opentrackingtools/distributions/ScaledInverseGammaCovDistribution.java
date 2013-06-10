package org.opentrackingtools.distributions;

import com.google.common.collect.Lists;

import org.apache.commons.lang3.builder.ToStringBuilder;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.MatrixFactory;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.AbstractDistribution;
import gov.sandia.cognition.statistics.ClosedFormComputableDistribution;
import gov.sandia.cognition.statistics.ProbabilityFunction;
import gov.sandia.cognition.statistics.distribution.InverseGammaDistribution;
import gov.sandia.cognition.util.CloneableSerializable;

public class ScaledInverseGammaCovDistribution 
    extends AbstractDistribution<Matrix> 
    implements ClosedFormComputableDistribution<Matrix> {

  protected int dimensionality = 0;
  protected InverseGammaDistribution inverseGammaDist; ;
  
  public ScaledInverseGammaCovDistribution(int dim, double shape, double scale) {
    this.dimensionality = dim;
    this.inverseGammaDist = new InverseGammaDistribution(shape, scale);
  }

  public ScaledInverseGammaCovDistribution(
      ScaledInverseGammaCovDistribution scaledInverseGammaCovDistribution) {
    this.dimensionality = scaledInverseGammaCovDistribution.dimensionality;
    this.inverseGammaDist = scaledInverseGammaCovDistribution.inverseGammaDist;
  }

  @Override
  public Matrix getMean() {
    return MatrixFactory.getDefault().createIdentity(dimensionality, dimensionality).scale(
        inverseGammaDist.getMean());
  }

  @Override
  public ArrayList<? extends Matrix> sample(Random random, int numSamples) {
    ArrayList<Matrix> result = Lists.newArrayList();
    for (Double sample : this.inverseGammaDist.sample(random, numSamples)) {
      result.add(MatrixFactory.getDefault().createIdentity(dimensionality, dimensionality)
          .scale(sample));
    }
    return result;
  }

  @Override
  public Vector convertToVector() {
    // TODO add dimension
    return null;
  }

  @Override
  public void convertFromVector(Vector parameters) {
    // TODO 
  }

  @Override
  public PDF getProbabilityFunction() {
    return new PDF(this);
  }
  
  public static class PDF extends ScaledInverseGammaCovDistribution implements ProbabilityFunction<Matrix>  {

    public PDF(
        ScaledInverseGammaCovDistribution scaledInverseGammaCovDistribution) {
      super(scaledInverseGammaCovDistribution);
    }

    @Override
    public Double evaluate(Matrix input) {
      // TODO
      return null;
    }

    @Override
    public double logEvaluate(Matrix input) {
      // TODO
      return Double.NaN;
    }
    
  }



  @Override
  public ScaledInverseGammaCovDistribution clone() {
    ScaledInverseGammaCovDistribution clone =  (ScaledInverseGammaCovDistribution) super.clone();
    clone.dimensionality = this.dimensionality;
    clone.inverseGammaDist = this.inverseGammaDist.clone();
    return clone;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result = prime * result + dimensionality;
    result = prime * result
        + ((inverseGammaDist == null) ? 0 : inverseGammaDist.hashCode());
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
    if (!(obj instanceof ScaledInverseGammaCovDistribution)) {
      return false;
    }
    ScaledInverseGammaCovDistribution other = (ScaledInverseGammaCovDistribution) obj;
    if (dimensionality != other.dimensionality) {
      return false;
    }
    if (inverseGammaDist == null) {
      if (other.inverseGammaDist != null) {
        return false;
      }
    } else if (!inverseGammaDist.equals(other.inverseGammaDist)) {
      return false;
    }
    return true;
  }

  public int getDimensionality() {
    return dimensionality;
  }

  public InverseGammaDistribution getInverseGammaDist() {
    return inverseGammaDist;
  }

  @Override
  public String toString() {
    ToStringBuilder builder = new ToStringBuilder(this);
    builder.append("dimensionality", dimensionality);
    builder.append("inverseGammaDist", inverseGammaDist);
    builder.append("mean", inverseGammaDist.getMean());
    return builder.toString();
  }

}
