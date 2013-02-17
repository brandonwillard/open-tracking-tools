package org.opentrackingtools.statistics.distributions.impl;

import java.util.Random;

import umontreal.iro.lecuyer.probdist.NormalDist;
import umontreal.iro.lecuyer.probdist.TruncatedDist;

import com.google.common.base.Preconditions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.math.matrix.VectorEntry;
import gov.sandia.cognition.math.matrix.VectorFactory;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

/**
 * 
 * Truncated velocities when on-road, not when off.
 * 
 * @author bwillard
 *
 */
public class TruncatedRoadGaussian extends AdjMultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;

  protected double velocityUpper = Double.POSITIVE_INFINITY;
  protected double velocityLower = Double.NEGATIVE_INFINITY;
  protected TruncatedDist truncDist;
  
  public TruncatedRoadGaussian() {
  }

  public TruncatedRoadGaussian(MultivariateGaussian other, double upper, double lower) {
    super(other);
    Preconditions.checkArgument(other.getInputDimensionality() == 2
        || other.getInputDimensionality() == 4);
    Preconditions.checkArgument(upper > lower);
    this.velocityUpper = upper;
    this.velocityLower = lower;
    this.setMean(truncateVector(this.getMean()));
  }

  public TruncatedRoadGaussian(Vector mean, Matrix covariance, 
    double velocityUpper, double velocityLower) {
    super(mean, covariance);
    Preconditions.checkArgument(velocityUpper > velocityLower);
    Preconditions.checkArgument(mean.getDimensionality() == 2 ||
        mean.getDimensionality() == 4);
    Preconditions.checkArgument(covariance.getNumColumns() == 2 ||
        covariance.getNumColumns() == 4);
    this.velocityUpper = velocityUpper;
    this.velocityLower = velocityLower;
    this.setMean(truncateVector(this.getMean()));
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient();
  }

  protected Vector truncateVector(Vector mean) {
    if (mean.getDimensionality() == 2) {
      final Vector adjMean = mean.clone();
      adjMean.setElement(1, 
          Math.min(velocityUpper, 
              Math.max(velocityLower, adjMean.getElement(1))));
      return adjMean;
    } 
    return mean;
  }
  
  @Override
  public void setMean(Vector mean) {
    this.truncDist = null;
    super.setMean(truncateVector(mean));
  }

  @Override
  public void setCovSqrt(Matrix covSqrt) {
    super.setCovSqrt(covSqrt);
    this.truncDist = null;
  }

  @Override
  public void setCovariance(Matrix covariance) {
    super.setCovariance(covariance);
    this.truncDist = null;
  }

  @Override
  public void setCovariance(Matrix covariance,
    double symmetryTolerance) {
    super.setCovariance(covariance, symmetryTolerance);
    this.truncDist = null;
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    super.setCovarianceInverse(covarianceInverse);
    this.truncDist = null;
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse,
    double symmetryTolerance) {
    super.setCovarianceInverse(covarianceInverse, symmetryTolerance);
    this.truncDist = null;
  }

  @Override
  public Vector sample(Random random) {
    final Vector sample = super.sample(random);
    
    if (this.getMean().getDimensionality() <= 2) {
      int dim = this.getMean().getDimensionality();
      if (this.truncDist == null) {
        this.truncDist = new TruncatedDist(new NormalDist(this.getMean().getElement(dim - 1), 
           Math.sqrt(this.getCovariance().getElement(dim - 1, dim - 1))), velocityLower, velocityUpper);
      }
      final double truncSmpl = this.truncDist.inverseF(random.nextDouble());
      sample.setElement(dim - 1, truncSmpl);
    }
    
    return sample;
  }

  @Override
  public PDF getProbabilityFunction() {
    return new TruncatedRoadGaussianPDF(this);
  }
  
  public static class TruncatedRoadGaussianPDF extends PDF {

    public TruncatedRoadGaussianPDF(
      TruncatedRoadGaussian other) {
      super(other);
    }

    @Override
    public Double evaluate(Vector input) {
      // FIXME use truncated prob.
      return super.evaluate(input);
    }

    @Override
    public double logEvaluate(Vector input) {
      // FIXME use truncated prob.
      return super.logEvaluate(input);
    }
    
  }
}
