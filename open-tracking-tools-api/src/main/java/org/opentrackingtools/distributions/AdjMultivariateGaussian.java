package org.opentrackingtools.distributions;

import gov.sandia.cognition.math.matrix.Matrix;
import gov.sandia.cognition.math.matrix.Vector;
import gov.sandia.cognition.statistics.distribution.MultivariateGaussian;

import java.util.Random;

import org.opentrackingtools.util.StatisticsUtil;
import org.opentrackingtools.util.SvdMatrix;

public class AdjMultivariateGaussian extends MultivariateGaussian {

  private static final long serialVersionUID = -7465667744835664792L;

  public AdjMultivariateGaussian() {
    super();
  }

  public AdjMultivariateGaussian(AdjMultivariateGaussian other) {
    this.setMean(other.getMean());
    this.setCovariance(other.getCovariance());
  }

  public AdjMultivariateGaussian(int dimensionality) {
    super(dimensionality);
  }

  public AdjMultivariateGaussian(MultivariateGaussian other) {
    this.setMean(other.getMean());
    this.setCovariance((other.getCovariance() instanceof SvdMatrix)
        ? other.getCovariance()
        : new SvdMatrix(other.getCovariance()));
  }

  public AdjMultivariateGaussian(Vector mean, SvdMatrix covariance) {
    this.setMean(mean);
    this.setCovariance(covariance);
  }

  @Override
  public MultivariateGaussian clone() {
    final AdjMultivariateGaussian clone =
        (AdjMultivariateGaussian) super.clone();
    return clone;
  }

  @Override
  public double computeZSquared(Vector input) {
    return super.computeZSquared(input);
  }

  @Override
  public MultivariateGaussian convolve(MultivariateGaussian other) {
    return super.convolve(other);
  }

  @Override
  public SvdMatrix getCovariance() {
    return (SvdMatrix) super.getCovariance();
  }

  @Override
  public SvdMatrix getCovarianceInverse() {
    return this.getCovariance().pseudoInverse();
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
  public MultivariateGaussian plus(MultivariateGaussian other) {
    return super.plus(other);
  }

  @Override
  public Vector sample(Random random) {
    final Matrix covSqrt =
        this.getCovariance()
            .getSvd()
            .getU()
            .times(
                StatisticsUtil.getDiagonalSqrt(this.getCovariance()
                    .getSvd().getS(), 1e-7));
    return MultivariateGaussian.sample(this.getMean(), covSqrt,
        random);
  }

  @Override
  public MultivariateGaussian scale(Matrix cov) {
    return super.scale((cov instanceof SvdMatrix) ? cov
        : new SvdMatrix(cov));
  }

  @Override
  public void setCovariance(Matrix cov) {
    super.setCovariance((cov instanceof SvdMatrix) ? cov
        : new SvdMatrix(cov));
  }

  @Override
  public void setCovariance(Matrix cov, double symmetryTolerance) {
    super.setCovariance((cov instanceof SvdMatrix) ? cov
        : new SvdMatrix(cov), symmetryTolerance);
  }

  @Override
  public void setCovarianceInverse(Matrix covarianceInverse) {
    super.setCovarianceInverse(new SvdMatrix(covarianceInverse));
  }

  @Override
  public void setCovarianceInverse(Matrix cov,
    double symmetryTolerance) {
    super.setCovarianceInverse((cov instanceof SvdMatrix) ? cov
        : new SvdMatrix(cov), symmetryTolerance);
  }

  @Override
  public MultivariateGaussian times(MultivariateGaussian other) {
    return super.times(other);
  }

}
