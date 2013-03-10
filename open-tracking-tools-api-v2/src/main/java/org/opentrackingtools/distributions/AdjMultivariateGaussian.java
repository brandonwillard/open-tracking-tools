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

  public AdjMultivariateGaussian(AdjMultivariateGaussian other) {
    super(other);
    this.covSqrt = other.covSqrt.clone();
  }

  public AdjMultivariateGaussian(int dimensionality) {
    super(dimensionality);
  }

  public AdjMultivariateGaussian(MultivariateGaussian other) {
    super(other);
  }

  public AdjMultivariateGaussian(Vector mean, Matrix covariance) {
    super(mean, covariance);
  }

  @Override
  public MultivariateGaussian clone() {
    final AdjMultivariateGaussian clone =
        (AdjMultivariateGaussian) super.clone();
    clone.covSqrt = ObjectUtil.cloneSmart(this.covSqrt);
    return clone;
  }

  @Override
  public double computeZSquared(Vector input) {
    return super.computeZSquared(input);
  }

  @Override
  public MultivariateGaussian convolve(MultivariateGaussian other) {
    this.covSqrt = null;
    return super.convolve(other);
  }

  public Matrix getCovSqrt() {
    if (this.covSqrt == null) {
      this.covSqrt =
          StatisticsUtil.rootOfSemiDefinite(this.getCovariance());
    }
    return this.covSqrt;
  }

  @Override
  public double getLogCovarianceDeterminant() {
    return super.getLogCovarianceDeterminant();
  }

  @Override
  public double getLogLeadingCoefficient() {
    return super.getLogLeadingCoefficient() + Math.log(2);
  }

  @Override
  public Vector sample(Random random) {
    if (this.covSqrt == null) {
      this.covSqrt =
          StatisticsUtil.rootOfSemiDefinite(this.getCovariance());
    }

    return MultivariateGaussian.sample(this.getMean(), this.covSqrt,
        random);
  }

  @Override
  public MultivariateGaussian scale(Matrix premultiplyMatrix) {
    this.covSqrt = null;
    return super.scale(premultiplyMatrix);
  }

  @Override
  public void setCovariance(Matrix covariance) {
    this.covSqrt = null;
    super.setCovariance(covariance);
  }

  @Override
  public void setCovariance(Matrix covariance,
    double symmetryTolerance) {
    this.covSqrt = null;
    super.setCovariance(covariance, symmetryTolerance);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    this.covSqrt = null;
    super.setCovarianceInverse(covarianceInverse);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse,
    double symmetryTolerance) {
    this.covSqrt = null;
    super.setCovarianceInverse(covarianceInverse, symmetryTolerance);
  }

  public void setCovSqrt(Matrix covSqrt) {
    this.covSqrt = covSqrt;
    this.setCovariance(covSqrt.times(covSqrt.transpose()));
  }

  @Override
  public MultivariateGaussian times(MultivariateGaussian other) {
    this.covSqrt = null;
    return super.times(other);
  }

}
