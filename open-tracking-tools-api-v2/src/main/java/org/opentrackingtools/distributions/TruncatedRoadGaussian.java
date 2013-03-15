package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;

import umontreal.iro.lecuyer.probdist.NormalDist;
import umontreal.iro.lecuyer.probdist.TruncatedDist;

import com.google.common.base.Preconditions;

/**
 * 
 * Truncated velocities when on-road, not when off.
 * 
 * @author bwillard
 * 
 */
public class TruncatedRoadGaussian extends AdjMultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;

  protected TruncatedDist truncDist;

  protected double velocityLower = Double.NEGATIVE_INFINITY;

  protected double velocityUpper = Double.POSITIVE_INFINITY;

  public TruncatedRoadGaussian(MultivariateGaussian other,
    double upper, double lower) {
    super(other);
    Preconditions.checkArgument(other.getInputDimensionality() == 2
        || other.getInputDimensionality() == 4);
    Preconditions.checkArgument(upper > lower);
    this.velocityUpper = upper;
    this.velocityLower = lower;
    this.setMean(this.truncateVector(this.getMean()));
  }

  public TruncatedRoadGaussian(Vector mean, Matrix covariance,
    double velocityUpper, double velocityLower) {
    super(mean.getDimensionality());
    Preconditions.checkArgument(velocityUpper > velocityLower);
    Preconditions.checkArgument(mean.getDimensionality() == 2
        || mean.getDimensionality() == 4);
    Preconditions.checkArgument(covariance.getNumColumns() == 2
        || covariance.getNumColumns() == 4);
    this.velocityUpper = velocityUpper;
    this.velocityLower = velocityLower;
    this.setMean(mean);
    this.setCovariance(covariance);
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient();
  }

  @Override
  public PDF getProbabilityFunction() {
    return new PDF(this);
  }

  @Override
  public Vector sample(Random random) {
    final Vector sample = super.sample(random);

    if (this.getMean().getDimensionality() <= 2) {
      final int dim = this.getMean().getDimensionality();
      if (this.truncDist == null) {
        this.truncDist =
            new TruncatedDist(new NormalDist(this.getMean()
                .getElement(dim - 1), Math.sqrt(this.getCovariance()
                .getElement(dim - 1, dim - 1))), this.velocityLower,
                this.velocityUpper);
      }
      final double truncSmpl =
          this.truncDist.inverseF(random.nextDouble());
      sample.setElement(dim - 1, truncSmpl);
    }

    return sample;
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
  public void setCovSqrt(Matrix covSqrt) {
    super.setCovSqrt(covSqrt);
    this.truncDist = null;
  }

  @Override
  public void setMean(Vector mean) {
    this.truncDist = null;
    super.setMean(this.truncateVector(mean));
  }

  protected Vector truncateVector(Vector mean) {
    if (mean.getDimensionality() == 2) {
      final Vector adjMean = mean.clone();
      adjMean.setElement(0, Math.max(0d, adjMean.getElement(0)));
      adjMean.setElement(
          1,
          Math.min(this.velocityUpper,
              Math.max(this.velocityLower, adjMean.getElement(1))));
      return adjMean;
    }
    return mean;
  }
}
