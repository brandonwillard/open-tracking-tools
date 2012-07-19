package org.openplans.tools.tracking.impl;

import gov.sandia.cognition.factory.Factory;
import gov.sandia.cognition.learning.algorithm.AbstractBatchAndIncrementalLearner;
import gov.sandia.cognition.math.MutableDouble;
import gov.sandia.cognition.statistics.AbstractDataDistribution;
import gov.sandia.cognition.statistics.DataDistribution;
import gov.sandia.cognition.statistics.DistributionEstimator;
import gov.sandia.cognition.statistics.DistributionWeightedEstimator;
import gov.sandia.cognition.util.AbstractCloneableSerializable;
import gov.sandia.cognition.util.ArgumentChecker;
import gov.sandia.cognition.util.WeightedValue;

import java.util.LinkedHashMap;
import java.util.Map;

public class DefaultCountedDataDistribution<KeyType> extends
    AbstractDataDistribution<KeyType> {

  /**
   * A factory for {@code LogDefaultDataDistribution} objects using some given
   * initial capacity for them.
   * 
   * @param <DataType>
   *          The type of data for the factory.
   */
  public static class DefaultFactory<DataType> extends
      AbstractCloneableSerializable implements
      Factory<DefaultCountedDataDistribution<DataType>> {

    /**
       * 
       */
    private static final long serialVersionUID = 681699655965182747L;
    /** The initial domain capacity. */
    protected int initialDomainCapacity;

    /**
     * Creates a new {@code DefaultFactory} with a default initial domain
     * capacity.
     */
    public DefaultFactory() {
      this(DEFAULT_INITIAL_CAPACITY);
    }

    /**
     * Creates a new {@code DefaultFactory} with a given initial domain
     * capacity.
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
          this.getInitialDomainCapacity());
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
    public void setInitialDomainCapacity(
      final int initialDomainCapacity) {
      ArgumentChecker.assertIsPositive("initialDomainCapacity",
          initialDomainCapacity);
      this.initialDomainCapacity = initialDomainCapacity;
    }

  }

  /**
   * Estimator for a LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class Estimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<KeyType, DefaultCountedDataDistribution.PMF<KeyType>>
      implements
      DistributionEstimator<KeyType, DefaultCountedDataDistribution.PMF<KeyType>> {

    /**
       * 
       */
    private static final long serialVersionUID = 8787720132790311008L;

    /**
     * Default constructor
     */
    public Estimator() {
      super();
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        createInitialLearnedObject() {
      return new DefaultCountedDataDistribution.PMF<KeyType>();
    }

    @Override
    public void update(
      final DefaultCountedDataDistribution.PMF<KeyType> target,
      final KeyType data) {
      target.increment(data, 1.0);
    }

  }

  /**
   * PMF of the LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class PMF<KeyType> extends
      DefaultCountedDataDistribution<KeyType> implements
      DataDistribution.PMF<KeyType> {

    /**
       * 
       */
    private static final long serialVersionUID = 355507596128913991L;

    /**
     * Default constructor
     */
    public PMF() {
      super();
    }

    /**
     * Copy constructor
     * 
     * @param other
     *          ScalarDataDistribution to copy
     */
    public PMF(final DataDistribution<KeyType> other) {
      super(other);
    }

    /**
     * Creates a new instance of LogDefaultDataDistribution
     * 
     * @param initialCapacity
     *          Initial capacity of the Map
     */
    public PMF(int initialCapacity) {
      super(initialCapacity);
    }

    /**
     * Creates a new instance of ScalarDataDistribution
     * 
     * @param data
     *          Data to create the distribution
     */
    public PMF(final Iterable<? extends KeyType> data) {
      super(data);
    }

    @Override
    public Double evaluate(KeyType input) {
      return this.getFraction(input);
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        getProbabilityFunction() {
      return this;
    }

    @Override
    public double logEvaluate(KeyType input) {
      return this.getLogFraction(input);
    }

  }

  /**
   * A weighted estimator for a LogDefaultDataDistribution
   * 
   * @param <KeyType>
   *          Type of Key in the distribution
   */
  public static class WeightedEstimator<KeyType>
      extends
      AbstractBatchAndIncrementalLearner<WeightedValue<? extends KeyType>, DefaultCountedDataDistribution.PMF<KeyType>>
      implements
      DistributionWeightedEstimator<KeyType, DefaultCountedDataDistribution.PMF<KeyType>> {

    /**
       * 
       */
    private static final long serialVersionUID =
        -9067384837227173014L;

    /**
     * Default constructor
     */
    public WeightedEstimator() {
      super();
    }

    @Override
    public DefaultCountedDataDistribution.PMF<KeyType>
        createInitialLearnedObject() {
      return new DefaultCountedDataDistribution.PMF<KeyType>();
    }

    @Override
    public void update(
      final DefaultCountedDataDistribution.PMF<KeyType> target,
      final WeightedValue<? extends KeyType> data) {
      target.increment(data.getValue(), data.getWeight());
    }

  }

  /**
   * 
   */
  private static final long serialVersionUID = 6596579376152342279L;

  /**
   * Default initial capacity, {@value} .
   */
  public static final int DEFAULT_INITIAL_CAPACITY = 10;

  /**
   * Total of the counts in the distribution
   */
  protected double total;

  /**
   * Default constructor
   */
  public DefaultCountedDataDistribution() {
    this(DEFAULT_INITIAL_CAPACITY);
  }

  /**
   * Creates a new instance of LogDefaultDataDistribution
   * 
   * @param other
   *          DataDistribution to copy
   */
  public DefaultCountedDataDistribution(
    final DataDistribution<? extends KeyType> other) {
    this(new LinkedHashMap<KeyType, MutableDouble>(other.size()), 0.0);
    this.incrementAll(other);
  }

  /**
   * Creates a new instance of LogDefaultDataDistribution
   * 
   * @param initialCapacity
   *          Initial capacity of the Map
   */
  public DefaultCountedDataDistribution(int initialCapacity) {
    this(new LinkedHashMap<KeyType, MutableDouble>(initialCapacity),
        0.0);
  }

  /**
   * Creates a new instance of ScalarDataDistribution
   * 
   * @param data
   *          Data to create the distribution
   */
  public DefaultCountedDataDistribution(
    final Iterable<? extends KeyType> data) {
    this();
    this.incrementAll(data);
  }

  /**
   * Creates a new instance of
   * 
   * @param map
   *          Backing Map that stores the data
   * @param total
   *          Sum of all values in the Map
   */
  protected DefaultCountedDataDistribution(
    final Map<KeyType, MutableDouble> map, final double total) {
    super(map);
    this.total = total;
  }

  @Override
  public void clear() {
    super.clear();
    this.total = 0.0;
  }

  @Override
  public DefaultCountedDataDistribution<KeyType> clone() {
    final DefaultCountedDataDistribution<KeyType> clone =
        new DefaultCountedDataDistribution<KeyType>(this.size());
    for (final java.util.Map.Entry<KeyType, MutableDouble> entry : this.map
        .entrySet()) {
      final MutableDoubleCount count =
          (MutableDoubleCount) entry.getValue();
      clone.set(entry.getKey(), count.getValue(), count.getCount());
    }

    assert this.getTotalCount() == clone.getTotalCount();

    clone.total = this.total;
    return clone;
  }

  public void copyAll(DataDistribution<KeyType> posteriorDist) {
    for (final java.util.Map.Entry<KeyType, ? extends Number> entry : posteriorDist
        .asMap().entrySet()) {
      final MutableDoubleCount count =
          (MutableDoubleCount) entry.getValue();
      this.set(entry.getKey(), count.getValue(), count.getCount());
    }
  }

  public int getCount(KeyType key) {
    return ((MutableDoubleCount) this.map.get(key)).getCount();
  }

  @Override
  public
      DistributionEstimator<KeyType, ? extends DataDistribution<KeyType>>
      getEstimator() {
    return new DefaultCountedDataDistribution.Estimator<KeyType>();
  }

  /**
   * Gets the average value of all keys in the distribution, that is, the total
   * value divided by the number of keys (even zero-value keys)
   * 
   * @return Average value of all keys in the distribution
   */
  public double getMeanValue() {
    final int ds = this.getDomainSize();
    if (ds > 0) {
      return this.getTotal() / ds;
    } else {
      return 0.0;
    }
  }

  @Override
  public DataDistribution.PMF<KeyType> getProbabilityFunction() {
    return new DefaultCountedDataDistribution.PMF<KeyType>(this);
  }

  @Override
  public double getTotal() {
    return this.total;
  }

  public int getTotalCount() {
    int total = 0;
    for (final MutableDouble value : this.map.values()) {
      final MutableDoubleCount count = (MutableDoubleCount) value;
      total += count.getCount();
    }
    return total;
  }

  @Override
  public double increment(KeyType key, final double value) {
    return this.increment(key, value, 1);
  }
  
  public double increment(KeyType key, final double value, int count) {
    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    double newValue;
    double delta;
    if (entry == null) {
      if (value > 0.0) {
        // It's best to avoid this.set() here as it could mess up
        // our total tracker in some subclasses...
        // Also it's more efficient this way (avoid another get)
        this.map.put(key, new MutableDoubleCount(value, count));
        delta = value;
      } else {
        delta = 0.0;
      }
      newValue = value;
    } else {
      if (entry.value + value >= 0.0) {
        delta = value;
        entry.plusEquals(value, count);
      } else {
        delta = -entry.value;
        entry.set(0d);
      }
      newValue = entry.value;
    }

    this.total += delta;
    return newValue;
  }

  @Override
  public void set(final KeyType key, final double value) {
    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    set(key, value, entry == null ? 1 : entry.getCount());
  }

  public void set(final KeyType key, final double value,
    final int count) {

    // TODO FIXME terrible hack!
    final MutableDoubleCount entry =
        (MutableDoubleCount) this.map.get(key);
    if (entry == null) {
      // Only need to allocate if it's not null
      if (value > 0.0) {
        this.map.put(key, new MutableDoubleCount(value, count));
        this.total += value;
      }
    } else if (value > 0.0) {
      this.total += value - entry.value;
      entry.set(value, count);
    } else {
      entry.set(0d, count);
    }
  }

}
