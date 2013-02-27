package org.opentrackingtools.distributions;

import gov.sandia.cognition.factory.Factory;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ArgumentChecker;

/**
 * A factory for {@code DefaultCountedDataDistribution} objects using some given
 * initial capacity for them.
 * 
 * @param <DataType>
 *          The type of data for the factory.
 */
public class DefaultFactory<DataType> extends
    AbstractCloneableSerializable implements
    Factory<DefaultCountedDataDistribution<DataType>> {

  private static final long serialVersionUID = 681699655965182747L;

  /**
   * Default initial capacity, {@value} .
   */
  public static final int DEFAULT_INITIAL_CAPACITY = 10;

  /** The initial domain capacity. */
  protected int initialDomainCapacity;

  private boolean isLogScale;

  /**
   * Creates a new {@code DefaultFactory} with a default initial domain
   * capacity.
   */
  public DefaultFactory(boolean isLogScale) {
    this(DEFAULT_INITIAL_CAPACITY);
    this.isLogScale = isLogScale;
  }

  /**
   * Creates a new {@code DefaultFactory} with a given initial domain capacity.
   * 
   * @param initialDomainCapacity
   *          The initial capacity for the domain. Must be positive.
   */
  public DefaultFactory(final int initialDomainCapacity) {
    super();
    this.setInitialDomainCapacity(initialDomainCapacity);
  }

  @Override
  public DefaultCountedDataDistribution<DataType> create() {
    // Create the histogram.
    return new DefaultCountedDataDistribution<DataType>(
        this.getInitialDomainCapacity(), isLogScale);
  }

  /**
   * Gets the initial domain capacity.
   * 
   * @return The initial domain capacity. Must be positive.
   */
  public int getInitialDomainCapacity() {
    return this.initialDomainCapacity;
  }

  /**
   * Sets the initial domain capacity.
   * 
   * @param initialDomainCapacity
   *          The initial domain capacity. Must be positive.
   */
  public void setInitialDomainCapacity(int initialDomainCapacity) {
    ArgumentChecker.assertIsPositive("initialDomainCapacity",
        initialDomainCapacity);
    this.initialDomainCapacity = initialDomainCapacity;
  }

  public boolean isLogScale() {
    return isLogScale;
  }

  public void setLogScale(boolean isLogScale) {
    this.isLogScale = isLogScale;
  }
}