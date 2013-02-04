package org.opentrackingtools.statistics.distributions.impl;

import java.util.Random;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

public class FoldedMultivariateGaussian extends AdjMultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;

  public FoldedMultivariateGaussian() {
  }

  public FoldedMultivariateGaussian(int dimensionality) {
    super(dimensionality);
  }

  public FoldedMultivariateGaussian(MultivariateGaussian other) {
    super(other);
  }

  public FoldedMultivariateGaussian(Vector mean, Matrix covariance) {
    super(mean, covariance);
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient() + Math.log(2);
  }

  @Override
  public Vector sample(Random random) {
    final Vector sample = super.sample(random);
    for (VectorEntry entry : sample) {
      entry.setValue(Math.abs(entry.getValue()));
    }
    return sample;
  }

  @Override
  public PDF getProbabilityFunction() {
    return new FoldedPDF(this);
  }
  
  public static class FoldedPDF extends PDF {

    public FoldedPDF(
      FoldedMultivariateGaussian other) {
      super(other);
    }

    @Override
    public Double evaluate(Vector input) {
      for (VectorEntry entry : input) {
        if (entry.getValue() < 0d)
          return 0d;
      }
      return super.evaluate(input);
    }

    @Override
    public double logEvaluate(Vector input) {
      for (VectorEntry entry : input) {
        if (entry.getValue() < 0d)
          return Double.NEGATIVE_INFINITY;
      }
      return super.logEvaluate(input);
    }
    
  }
}
