package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;
import gov.sandia.cognition.util.ObjectUtil;

import java.util.Random;

import org.opentrackingtools.util.StatisticsUtil;

public class AdjMultivariateGaussian extends MultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;
  
  protected Matrix covSqrt = null;

  public AdjMultivariateGaussian() {
    super();
  }

  public AdjMultivariateGaussian(int dimensionality) {
    super(dimensionality);
  }

  public AdjMultivariateGaussian(MultivariateGaussian other) {
    super(other);
  }
  
  public AdjMultivariateGaussian(AdjMultivariateGaussian other) {
    super(other);
    covSqrt = other.covSqrt.clone();
  }

  public AdjMultivariateGaussian(Vector mean, Matrix covariance) {
    super(mean, covariance);
  }

  @Override
  public double computeZSquared(Vector input) {
    return super.computeZSquared(input);
  }

  @Override
  public double getLogCovarianceDeterminant() {
    return super.getLogCovarianceDeterminant();
  }

  public Matrix getCovSqrt() {
    if (this.covSqrt == null) {
      covSqrt =
          StatisticsUtil.rootOfSemiDefinite(this.getCovariance());
    }
    return covSqrt;
  }

  public void setCovSqrt(Matrix covSqrt) {
    this.covSqrt = covSqrt;
    this.setCovariance(covSqrt.times(covSqrt.transpose()));
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient() + Math.log(2);
  }
  
  @Override
  public void setCovariance(Matrix covariance) {
    covSqrt = null;
    super.setCovariance(covariance);
  }

  @Override
  public void setCovariance(Matrix covariance,
    double symmetryTolerance) {
    covSqrt = null;
    super.setCovariance(covariance, symmetryTolerance);
  }

  @Override
  public MultivariateGaussian times(MultivariateGaussian other) {
    covSqrt = null;
    return super.times(other);
  }

  @Override
  public MultivariateGaussian convolve(MultivariateGaussian other) {
    covSqrt = null;
    return super.convolve(other);
  }

  @Override
  public MultivariateGaussian scale(Matrix premultiplyMatrix) {
    covSqrt = null;
    return super.scale(premultiplyMatrix);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    covSqrt = null;
    super.setCovarianceInverse(covarianceInverse);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse,
    double symmetryTolerance) {
    covSqrt = null;
    super.setCovarianceInverse(covarianceInverse, symmetryTolerance);
  }

  @Override
  public Vector sample(Random random) {
    if (this.covSqrt == null) {
      covSqrt =
          StatisticsUtil.rootOfSemiDefinite(this.getCovariance());
    }
    
    return MultivariateGaussian.sample(this.getMean(), this.covSqrt, random);
  }

  @Override
  public MultivariateGaussian clone() {
    AdjMultivariateGaussian clone = (AdjMultivariateGaussian) super.clone();
    clone.covSqrt = ObjectUtil.cloneSmart(this.covSqrt);
    return clone;
  }

}
