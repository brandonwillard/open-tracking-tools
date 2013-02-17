package org.opentrackingtools.impl;

import gov.sandia.cognition.math.MutableDouble;

public class MutableDoubleCount extends MutableDouble {
  public int count = 0;

  private static final long serialVersionUID =
      -6936453778285494680L;

  public MutableDoubleCount(double value) {
    super(value);
    this.count++;
  }

  public MutableDoubleCount(double value, int count) {
    super(value);
    this.count = count;
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (!super.equals(obj)) {
      return false;
    }
    if (getClass() != obj.getClass()) {
      return false;
    }
    final MutableDoubleCount other =
        (MutableDoubleCount) obj;
    if (count != other.count) {
      return false;
    }
    return true;
  }

  public int getCount() {
    return this.count;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = super.hashCode();
    result = prime * result + count;
    return result;
  }

  public void plusEquals(double value) {
    this.value += value;
    this.count++;
  }

  public void plusEquals(double value, int count) {
    this.value += value;
    this.count += count;
  }

  public void set(double value) {
    this.set(value, 1);
  }

  public void set(double value, int count) {
    this.value = value;
    this.count = count;
  }

  @Override
  public String toString() {
    return "MutableDoubleCount [count=" + count
        + ", value=" + value + "]";
  }
}