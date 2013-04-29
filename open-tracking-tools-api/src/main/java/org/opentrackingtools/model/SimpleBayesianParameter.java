package org.opentrackingtools.model;

import gov.sandia.cognition.statistics.Distribution;
import gov.sandia.cognition.statistics.bayesian.BayesianParameter;
import gov.sandia.cognition.util.AbstractCloneableSerializable;

import java.util.Random;

import org.apache.commons.lang3.builder.CompareToBuilder;
import org.apache.commons.lang3.builder.ToStringBuilder;
import org.apache.commons.lang3.builder.ToStringStyle;

public class SimpleBayesianParameter<ParameterType, ConditionalType extends Distribution<?>, PriorType extends Distribution<ParameterType>>
    extends AbstractCloneableSerializable implements
    BayesianParameter<ParameterType, ConditionalType, PriorType>,
    Comparable<SimpleBayesianParameter<?, ?, ?>> {

  /**
   * 
   */
  private static final long serialVersionUID = 6485062302557172060L;

  public static
      <Par, C extends Distribution<?>, P extends Distribution<Par>>
      SimpleBayesianParameter<Par, C, P> create(Par value,
        C conditional, P prior) {
    return new SimpleBayesianParameter<Par, C, P>(conditional, prior,
        value);
  }

  protected ConditionalType conditional;
  protected String name = null;
  protected PriorType prior;

  protected ParameterType value;

  public SimpleBayesianParameter(ConditionalType conditional,
    PriorType prior, ParameterType value) {
    super();
    this.conditional = conditional;
    this.prior = prior;
    this.value = value;
  }

  public SimpleBayesianParameter(
    SimpleBayesianParameter<ParameterType, ConditionalType, PriorType> other) {
    super();
    this.conditional = other.conditional;
    this.prior = other.prior;
    this.value = other.value;
  }

  /**
   * Shallow clone. Deep copies in here can be expensive.
   */
  @Override
  public
      SimpleBayesianParameter<ParameterType, ConditionalType, PriorType>
      clone() {
    final SimpleBayesianParameter<ParameterType, ConditionalType, PriorType> clone =
        (SimpleBayesianParameter<ParameterType, ConditionalType, PriorType>) super
            .clone();
    //    clone.conditional = ObjectUtil.cloneSmart(this.conditional);
    //    clone.prior = ObjectUtil.cloneSmart(this.prior);
    //    clone.value = ObjectUtil.cloneSmart(this.value);
    clone.conditional = this.conditional;
    clone.prior = this.prior;
    clone.value = this.value;
    clone.name = this.name;
    return clone;
  }

  @Override
  public int compareTo(SimpleBayesianParameter<?, ?, ?> o) {
    final CompareToBuilder comparator = new CompareToBuilder();
    comparator.append(this.name, o.name);
    comparator.append(this.conditional, o.conditional);
    comparator.append(this.prior, o.prior);
    comparator.append(this.value, o.value);
    return comparator.build();
  }

  @Override
  public boolean equals(Object obj) {
    if (this == obj) {
      return true;
    }
    if (obj == null) {
      return false;
    }
    if (this.getClass() != obj.getClass()) {
      return false;
    }
    final SimpleBayesianParameter<?, ?, ?> other =
        (SimpleBayesianParameter<?, ?, ?>) obj;
    if (this.conditional == null) {
      if (other.conditional != null) {
        return false;
      }
    } else if (!this.conditional.equals(other.conditional)) {
      return false;
    }
    if (this.name == null) {
      if (other.name != null) {
        return false;
      }
    } else if (!this.name.equals(other.name)) {
      return false;
    }
    if (this.prior == null) {
      if (other.prior != null) {
        return false;
      }
    } else if (!this.prior.equals(other.prior)) {
      return false;
    }
    if (this.value == null) {
      if (other.value != null) {
        return false;
      }
    } else if (!this.value.equals(other.value)) {
      return false;
    }
    return true;
  }

  @Override
  public ConditionalType getConditionalDistribution() {
    return this.conditional;
  }

  @Override
  public String getName() {
    return this.name;
  }

  @Override
  public PriorType getParameterPrior() {
    return this.prior;
  }

  @Override
  public ParameterType getValue() {
    return this.value;
  }

  @Override
  public int hashCode() {
    final int prime = 31;
    int result = 1;
    result =
        prime
            * result
            + ((this.conditional == null) ? 0 : this.conditional
                .hashCode());
    result =
        prime * result
            + ((this.name == null) ? 0 : this.name.hashCode());
    result =
        prime * result
            + ((this.prior == null) ? 0 : this.prior.hashCode());
    result =
        prime * result
            + ((this.value == null) ? 0 : this.value.hashCode());
    return result;
  }

  public ConditionalType setConditionalDistribution(
    ConditionalType conditional) {
    return this.conditional = conditional;
  }

  public void setName(String name) {
    this.name = name;
  }

  public void setParameterPrior(PriorType prior) {
    this.prior = prior;
  }

  @Override
  public void setValue(ParameterType value) {
    this.value = value;
  }

  @Override
  public String toString() {
    final ToStringBuilder builder =
        new ToStringBuilder(this, ToStringStyle.SHORT_PREFIX_STYLE);
    builder.append("value", this.value);
    builder.append("conditional", this.conditional);
    builder.append("prior", this.prior);
    builder.append("name", this.name);
    return builder.toString();
  }

  @Override
  public void updateConditionalDistribution(Random random) {
  }

}
